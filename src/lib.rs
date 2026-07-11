#![no_std]

#[cfg(all(feature = "debug-logging", feature = "local-logging"))]
compile_error!("only one logging feature should be enabled");

pub mod data;
pub mod motor;
pub mod resources;
pub mod sensors;

use defmt_rtt as _;
use embassy_stm32::{
    Config,
    rcc::{
        AHBPrescaler, APBPrescaler, Hse, HseMode, Hsi48Config, Pll, PllDiv, PllMul, PllPreDiv,
        PllSource, Sysclk, VoltageScale,
        mux::{I2c34sel, I2csel, Spi1sel, Spi2sel, Spi3sel, Usbsel},
    },
    time::Hertz,
};
use embassy_sync::{blocking_mutex::raw::ThreadModeRawMutex, signal::Signal};
use embassy_time::{Instant, TICK_HZ, Timer};
use resources::*;

/// The earth do be pulling frfr.
pub const G: f32 = 9.80665;

/// Recommended update rate of the MCU main loop.
pub const HZ: u64 = 100;

/// Can be signaled at any time to murder all sensor tasks,
/// as well as the estimator task that fuses their data.
///
/// This doesn't stop the sensors themselves from operating,
/// just the MCU from communicating with them.
pub static CANCEL_SENSORS: Signal<ThreadModeRawMutex, ()> = Signal::new();

/// Micro-optimized function for obtaining the delta (in seconds) since `last`.
///
/// Returns the delta; updates `last` to the current time.
#[inline(always)]
pub fn get_dt_s(last: &mut Instant) -> f32 {
    /// Accuracy loss of multiplying by this value instead of dividing
    /// by `TICK_HZ` is cosmically insignificant for our application.
    const TICKIN_MAH_SHIT: f32 = 1.0 / TICK_HZ as f32;
    // Because we've acquired `now` when we already have `last`,
    // we guarantee that `tick_delta` will be positive so long
    // as time is ticking forward, which it always is for us.
    let now = Instant::now();
    // Casting to u32 here is very important, as the u64->f32 conversion
    // requires calling an intrinsic, but u32->f32 is a dedicated instruction.
    // And u64->u32 is literally free.
    // The tick delta will always be very small, so this
    // downcast doesn't cause any loss of information.
    let tick_delta = (now.as_ticks() - last.as_ticks()) as u32;
    let dt_s = tick_delta as f32 * TICKIN_MAH_SHIT;
    *last = now;
    dt_s
}

/// Fully initializes the MCU, including all required clock domains.
///
/// This should always be the first function called in a binary,
/// and all peripherals will be ready immediately afterwards.
pub async fn init() -> AssignedResources {
    let mut config = Config::default();

    // ECS-TXO-2520-33-240-AN-TR on PH0 / OSC_IN
    const FREQUENCY_MEGAHERTZ: u32 = 24;
    config.rcc.hse = Some(Hse {
        freq: Hertz::mhz(FREQUENCY_MEGAHERTZ),
        mode: HseMode::BypassDigital,
    });

    // HSE = 24 MHz TCXO
    // 24 / 6 = 4 MHz PLL1 input
    // 4 * 125 = 500 MHz PLL1 VCO
    // PLL1_P = 500 / 2 = 250 MHz SYSCLK
    // PLL1_Q = 500 / 5 = 100 MHz SPI kernel
    config.rcc.voltage_scale = VoltageScale::Scale0;
    config.rcc.pll1 = Some(Pll {
        source: PllSource::HSE,
        prediv: PllPreDiv::DIV6,
        mul: PllMul::MUL125,
        divp: Some(PllDiv::DIV2),
        divq: Some(PllDiv::DIV5),
        divr: None,
    });
    config.rcc.sys = Sysclk::PLL1_P;

    // Passthrough scalars
    config.rcc.ahb_pre = AHBPrescaler::DIV1;
    config.rcc.apb1_pre = APBPrescaler::DIV1;
    config.rcc.apb2_pre = APBPrescaler::DIV1;
    config.rcc.apb3_pre = APBPrescaler::DIV1;

    // I2C are all on 250MHz kernel
    config.rcc.mux.i2c1sel = I2csel::PCLK1;
    config.rcc.mux.i2c2sel = I2csel::PCLK1;
    config.rcc.mux.i2c3sel = I2c34sel::PCLK3;

    // SPI are all on 100MHz kernel
    config.rcc.mux.spi1sel = Spi1sel::PLL1_Q;
    config.rcc.mux.spi2sel = Spi2sel::PLL1_Q;
    config.rcc.mux.spi3sel = Spi3sel::PLL1_Q;

    // Enable HSI48 for the USB peripheral
    config.rcc.hsi48 = Some(Hsi48Config {
        sync_from_usb: true, // Enable Clock Recovery System (CRS) for USB accuracy
    });
    // Explicitly bind usb to 48MHz kernel
    config.rcc.mux.usbsel = Usbsel::HSI48;

    let p = embassy_stm32::init(config);

    // Force whoever's initializing the MCU to wait a small duration,
    // ensuring all peripherals have enough time to fully turn on.
    Timer::after_millis(100).await;

    split_resources!(p)
}

#[inline(never)]
#[panic_handler]
fn panic(_info: &core::panic::PanicInfo) -> ! {
    defmt::error!("panicked");
    const RETRY_COUNT: usize = 69;
    for _ in 0..RETRY_COUNT {
        if let Ok(mut l) = motor::DEPLOYED.try_lock() {
            *l = false;
        }
    }
    loop {
        cortex_m::asm::nop();
    }
}
