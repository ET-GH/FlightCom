//! THIS BINARY IS FOR RUNNING THE SIMULATION FROM data.rs!!!

#![no_main]
#![no_std]

use embassy_executor::Spawner;
use embassy_time::Timer;
use mcu::{G, data::FLIGHT_DATA, motor, sensors::imu};

/// Fast filter used for event detection.
///
/// tau = 10 ms corresponds to a cutoff ~16 Hz.
const FAST_FILTER_TAU_S: f32 = 0.010;

/// Slow filter: tracks sustained acceleration during motor burn.
const SLOW_FILTER_TAU_S: f32 = 0.100;

/// Magnitude must rise this far above the calibrated pad magnitude.
const LIFTOFF_MAG_DELTA_G: f32 = 1.25;

/// Positive Y acceleration must rise this far above its pad value.
///
/// Since Y is approximately +1 g on the pad, this initially requires +4.0 g on the Y axis.
const LIFTOFF_AXIAL_DELTA_G: f32 = 3.0;

/// Liftoff conditions must remain true for this long.
const LIFTOFF_HOLD_S: f32 = 0.050;

/// Do not permit burnout immediately after liftoff.
///
/// Should be below the shortest expected motor burn time.
const MIN_BURN_TIME_S: f32 = 0.250;

/// Required difference between the slow boost estimate and fast axial signal.
const BURNOUT_DROP_G: f32 = 0.75;

/// Fast axial acceleration must fall below this fraction of the slow boost estimate.
const BURNOUT_REMAINING_RATIO: f32 = 0.45;

/// A low absolute axial value is also strong evidence of coasting to freedom.
///
/// Depending on drag, this condition might not become true immediately, which
/// is why it is ORed with the relative-drop condition.
const COAST_AXIAL_THRESHOLD_G: f32 = 0.50;

/// Burnout conditions must remain true for this long.
const BURNOUT_HOLD_S: f32 = 0.100;

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let resources = mcu::init().await;
    defmt::info!("mcu resources assigned");

    spawner.spawn(motor::start(resources.motor).unwrap());
    spawner.spawn(imu::start(resources.imu).unwrap());

    motor::READY.wait().await;
    imu::READY.wait().await;
    Timer::after_secs(1).await;
    defmt::info!("starting");

    // Calibration from stationary prelaunch.
    let mut calibration_samples: u32 = 0;
    let mut magnitude_sum_g = 0.0;
    let mut axial_sum_g = 0.0;
    let mut idx: usize = 0;

    for f in &FLIGHT_DATA {
        if f.ts >= -0.12 {
            break;
        }

        let magnitude_g = libm::sqrtf(f.ax * f.ax + f.ay * f.ay + f.az * f.az) / G;

        // This dataset has thrust along negative Y. Convert it into the
        // thrust-positive convention expected by RocketStateMachine.
        // (this is how it is in the other brute_force)
        let axial_g = -f.ay / G;

        magnitude_sum_g += magnitude_g;
        axial_sum_g += axial_g;
        calibration_samples += 1;
        idx += 1;
    }

    if calibration_samples == 0 {
        defmt::panic!("what the fuck are you doing retard");
    }

    let sample_count = calibration_samples as f32;
    let baseline_magnitude_g = magnitude_sum_g / sample_count;
    let baseline_axial_g = axial_sum_g / sample_count;

    defmt::info!(
        "test data calibrated: samples={}, magnitude={} g, axial_y={} g",
        calibration_samples,
        baseline_magnitude_g,
        baseline_axial_g
    );

    let mut rocket = RocketStateMachine::new(baseline_magnitude_g, baseline_axial_g);

    while idx < FLIGHT_DATA.len() {
        let f = FLIGHT_DATA[idx];

        // Probably safe because calibration consumed at least one sample.
        let dt_s = f.ts - FLIGHT_DATA[idx - 1].ts;

        let magnitude_g = libm::sqrtf(f.ax * f.ax + f.ay * f.ay + f.az * f.az) / G;

        let axial_g = -f.ay / G;

        let done = rocket.update(magnitude_g, axial_g, dt_s, idx).await;
        if done {
            break;
        }

        idx += 1;

        if idx < FLIGHT_DATA.len() {
            let delay_s = FLIGHT_DATA[idx].ts - f.ts;
            if delay_s > 0.0 {
                let delay_ns = (delay_s * 1e9) as u64;
                Timer::after_nanos(delay_ns).await;
            }
        }
    }

    defmt::info!("done");
}

struct RocketStateMachine {
    phase: FlightPhase,

    baseline_magnitude_g: f32,
    baseline_axial_g: f32,

    filtered_magnitude_g: f32,
    filtered_axial_fast_g: f32,
    filtered_axial_slow_g: f32,

    event_hold_time_s: f32,
    time_since_liftoff_s: f32,
}

impl RocketStateMachine {
    fn new(baseline_magnitude_g: f32, baseline_axial_g: f32) -> Self {
        Self {
            phase: FlightPhase::default(),

            baseline_magnitude_g,
            baseline_axial_g,

            filtered_magnitude_g: baseline_magnitude_g,
            filtered_axial_fast_g: baseline_axial_g,
            filtered_axial_slow_g: baseline_axial_g,

            event_hold_time_s: 0.0,
            time_since_liftoff_s: 0.0,
        }
    }

    async fn update(&mut self, magnitude_g: f32, axial_g: f32, dt_s: f32, idx: usize) -> bool {
        self.filtered_magnitude_g = low_pass(
            self.filtered_magnitude_g,
            magnitude_g,
            dt_s,
            FAST_FILTER_TAU_S,
        );

        self.filtered_axial_fast_g =
            low_pass(self.filtered_axial_fast_g, axial_g, dt_s, FAST_FILTER_TAU_S);

        self.filtered_axial_slow_g =
            low_pass(self.filtered_axial_slow_g, axial_g, dt_s, SLOW_FILTER_TAU_S);

        match self.phase {
            FlightPhase::ReadyForLaunch => {
                let magnitude_high =
                    self.filtered_magnitude_g > self.baseline_magnitude_g + LIFTOFF_MAG_DELTA_G;

                let axial_high =
                    self.filtered_axial_fast_g > self.baseline_axial_g + LIFTOFF_AXIAL_DELTA_G;

                let liftoff_candidate = magnitude_high && axial_high;

                if condition_held_for(
                    liftoff_candidate,
                    dt_s,
                    LIFTOFF_HOLD_S,
                    &mut self.event_hold_time_s,
                ) {
                    self.phase = FlightPhase::Boosting;

                    self.event_hold_time_s = 0.0;
                    self.time_since_liftoff_s = 0.0;

                    defmt::info!(
                        "LIFTOFF: magnitude={} g, axial_y={} g",
                        self.filtered_magnitude_g,
                        self.filtered_axial_fast_g
                    );
                }
            }

            FlightPhase::Boosting => {
                self.time_since_liftoff_s += dt_s;

                let inside_burn_window = self.time_since_liftoff_s >= MIN_BURN_TIME_S;

                let acceleration_drop_g = self.filtered_axial_slow_g - self.filtered_axial_fast_g;

                let slow_signal_was_boosting =
                    self.filtered_axial_slow_g > self.baseline_axial_g + LIFTOFF_AXIAL_DELTA_G;

                let relative_drop = slow_signal_was_boosting
                    && self.filtered_axial_fast_g
                        < self.filtered_axial_slow_g * BURNOUT_REMAINING_RATIO;

                let coast_like_absolute_value =
                    self.filtered_axial_fast_g < COAST_AXIAL_THRESHOLD_G;

                let burnout_candidate = inside_burn_window
                    && acceleration_drop_g > BURNOUT_DROP_G
                    && (relative_drop || coast_like_absolute_value);

                if condition_held_for(
                    burnout_candidate,
                    dt_s,
                    BURNOUT_HOLD_S,
                    &mut self.event_hold_time_s,
                ) {
                    self.phase = FlightPhase::Complete;
                    self.event_hold_time_s = 0.0;

                    defmt::info!(
                        "BURNOUT: t={} s, axial_fast={} g, axial_slow={} g, drop={} g",
                        self.time_since_liftoff_s,
                        self.filtered_axial_fast_g,
                        self.filtered_axial_slow_g,
                        acceleration_drop_g
                    );
                    defmt::info!("deploying motor: {}", idx);
                    *motor::DEPLOYED.lock().await = true;
                }
            }

            FlightPhase::Complete => {
                Timer::after_secs(25).await;
                defmt::info!("APOGEE (probably lol)");
                defmt::info!("retracting motor");
                *motor::DEPLOYED.lock().await = false;
                return true;
            }
        }
        false
    }
}

/// First-order low-pass filter.
fn low_pass(previous: f32, input: f32, dt_s: f32, tau_s: f32) -> f32 {
    let alpha = dt_s / (tau_s + dt_s);
    previous + alpha * (input - previous)
}

/// Returns true once `condition` has remained true for `required_time_s`.
///
/// Timer resets completely when the condition becomes false.
fn condition_held_for(
    condition: bool,
    dt_s: f32,
    required_time_s: f32,
    accumulated_time_s: &mut f32,
) -> bool {
    if condition {
        *accumulated_time_s += dt_s;
    } else {
        *accumulated_time_s = 0.0;
    }
    *accumulated_time_s >= required_time_s
}

#[derive(Clone, Copy, Default)]
enum FlightPhase {
    #[default]
    ReadyForLaunch,
    Boosting,
    Complete,
}
