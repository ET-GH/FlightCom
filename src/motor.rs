use crate::resources::{Irq, MotorResources};
use embassy_stm32::{
    gpio::{Level, Output, Pull, Speed},
    spi::{BitOrder, Config, MODE_3, Spi},
    time::Hertz,
};
use embassy_sync::{blocking_mutex::raw::ThreadModeRawMutex, mutex::Mutex, signal::Signal};
use embassy_time::{Duration, Ticker, Timer};

/// `false`: fully retracted
/// `true`: fully deployed (me when she's 5'1" and calls me daddy)
pub static DEPLOYED: Mutex<ThreadModeRawMutex, bool> = Mutex::new(false);
pub static READY: Signal<ThreadModeRawMutex, ()> = Signal::new();

// Mechanical configuration.
const MOTOR_FULL_STEPS_PER_REV: i32 = 200;
const MICROSTEPS_PER_FULL_STEP: i32 = 256;
const DEPLOY_ROTATIONS: i32 = 3;
const DEPLOY_DIRECTION: i32 = 1; // Change to -1 if deployment runs backward.

// Motion configuration (zoom zoom bitch).
const MAX_SPEED_REV_PER_S: u32 = 5;
const MAX_ACCEL_REV_PER_S2: u32 = 33;

// Internal clock when CLK is tied low.
const TMC_CLOCK_HZ: u128 = 12_500_000;
const MICROSTEPS_PER_REV: u128 =
    MOTOR_FULL_STEPS_PER_REV as u128 * MICROSTEPS_PER_FULL_STEP as u128;

// Datasheet conversions for ramp generator.
// This might just be wrong lol.
const VMAX: u32 =
    (((MAX_SPEED_REV_PER_S as u128 * MICROSTEPS_PER_REV) << 24) / TMC_CLOCK_HZ) as u32;
const AMAX: u32 = (((MAX_ACCEL_REV_PER_S2 as u128 * MICROSTEPS_PER_REV) << 42)
    / (TMC_CLOCK_HZ * TMC_CLOCK_HZ)) as u32;
const DMAX: u32 = AMAX;

const FULL_DEPLOY_POSITION: i32 =
    DEPLOY_DIRECTION * DEPLOY_ROTATIONS * MOTOR_FULL_STEPS_PER_REV * MICROSTEPS_PER_FULL_STEP;

// Register addresses.
const GCONF: u8 = 0x00;
const GSTAT: u8 = 0x01;
const IOIN: u8 = 0x04;
const DRV_CONF: u8 = 0x0A;
const GLOBALSCALER: u8 = 0x0B;
const IHOLD_IRUN: u8 = 0x10;
const TPOWERDOWN: u8 = 0x11;
const RAMPMODE: u8 = 0x20;
const XACTUAL: u8 = 0x21;
const VSTART: u8 = 0x23;
const A1: u8 = 0x24;
const V1: u8 = 0x25;
const AMAX_REG: u8 = 0x26;
const VMAX_REG: u8 = 0x27;
const DMAX_REG: u8 = 0x28;
const TVMAX: u8 = 0x29;
const D1: u8 = 0x2A;
const VSTOP: u8 = 0x2B;
const TZEROWAIT: u8 = 0x2C;
const XTARGET: u8 = 0x2D;
const V2: u8 = 0x2E;
const A2: u8 = 0x2F;
const D2: u8 = 0x30;
const SW_MODE: u8 = 0x34;
const CHOPCONF: u8 = 0x6C;
const DRV_STATUS: u8 = 0x6F;

const WRITE_BIT: u8 = 0x80;

// Requested current configuration.
const DRV_CONF_VALUE: u32 = 0b01; // CURRENT_RANGE = 01. 00 is for pussies. maybe worth trying 10?
const GLOBALSCALER_VALUE: u32 = 0; // MAX IT OUT bABY

const IRUNDELAY: u32 = 0;
const IHOLDDELAY: u32 = 1; // hold this l
const IRUN: u32 = 31; // max
const IHOLD: u32 = 23; // 75%
const IHOLD_IRUN_VALUE: u32 = (IRUNDELAY << 24) | (IHOLDDELAY << 16) | (IRUN << 8) | IHOLD;

// SpreadCycle:
// INTPOL=1, TPFD=4, MRES=0,
// TBL=2, HSTART=0, HEND=0 (encoded as 3), TOFF=5.
const CHOPCONF_VALUE: u32 = (1 << 28) | (4 << 20) | (2 << 15) | (3 << 7) | 5;

const GSTAT_CRITICAL_MASK: u32 = 0b1_1110;
const DRV_STATUS_CRITICAL_MASK: u32 = (1 << 28) | (1 << 27) | (1 << 25) | (1 << 13) | (1 << 12);

macro_rules! transfer {
    ($bus:expr, $cs:expr, $address:expr, $value:expr) => {{
        let data = ($value as u32).to_be_bytes();
        let write = [$address, data[0], data[1], data[2], data[3]];
        let mut read = [0u8; 5];

        $cs.set_low();
        let result = $bus.transfer(&mut read, &write).await;
        $cs.set_high();

        if let Err(e) = result {
            defmt::panic!("motor: SPI transfer failed: {}", e);
        }

        read
    }};
}

macro_rules! write_register {
    ($bus:expr, $cs:expr, $register:expr, $value:expr) => {{
        let _ = transfer!($bus, $cs, $register | WRITE_BIT, $value);
    }};
}

macro_rules! read_register {
    ($bus:expr, $cs:expr, $register:expr) => {{
        // TMC5240 reads are pipelined: request first, receive on the next datagram.
        let _ = transfer!($bus, $cs, $register, 0);
        let read = transfer!($bus, $cs, $register, 0);
        u32::from_be_bytes([read[1], read[2], read[3], read[4]])
    }};
}

#[embassy_executor::task]
pub async fn start(r: MotorResources) {
    // SPI config as specified in the datasheet.
    let mut config = Config::default();
    config.mode = MODE_3;
    config.bit_order = BitOrder::MsbFirst;
    config.frequency = Hertz::mhz(1);
    config.gpio_speed = Speed::Medium;
    config.miso_pull = Pull::None;

    let mut bus = Spi::new(
        r.peri, r.sck, r.mosi, r.miso, r.tx_dma, r.rx_dma, Irq, config,
    );
    let mut cs = Output::new(r.cs, Level::High, Speed::Medium);
    let mut drv_en_n = Output::new(r.drv_en_n, Level::High, Speed::Medium);
    let mut sleep_n = Output::new(r.sleep_n, Level::Low, Speed::Medium);

    // Force reset, then wait for the IC to wake completely.
    Timer::after_millis(1).await;
    // WAKE ME UP INSIDE (I CAN'T WAKE UP)
    sleep_n.set_high();
    Timer::after_millis(100).await;

    let ioin = read_register!(&mut bus, &mut cs, IOIN);
    let version = (ioin >> 24) as u8;
    let silicon_revision = ((ioin >> 16) & 0b111) as u8;

    // lowkey just don't even bother
    if version != 0x40 && version != 0x41 {
        defmt::warn!(
            "motor: unexpected TMC version: version={}, silicon_revision={}, IOIN={=u32:#010x}",
            version,
            silicon_revision,
            ioin
        );
    }

    defmt::info!(
        "motor: TMC connected: version={}, silicon_revision={}",
        version,
        silicon_revision
    );

    // Keep bridges disabled while configuring the controller.
    write_register!(&mut bus, &mut cs, CHOPCONF, 0);
    write_register!(&mut bus, &mut cs, GSTAT, 0x1F);

    const CONFIGURATION: &[(u8, u32)] = &[
        (GCONF, 0), // SpreadCycle.
        (DRV_CONF, DRV_CONF_VALUE),
        (GLOBALSCALER, GLOBALSCALER_VALUE),
        (IHOLD_IRUN, IHOLD_IRUN_VALUE),
        (TPOWERDOWN, 10),
        (SW_MODE, 0),
        (RAMPMODE, 3), // Hold zero velocity while configuring
        (XACTUAL, 0),  // Startup is defined as fully retracted
        (VSTART, 0),
        (A1, 0),
        (V1, 0),
        (AMAX_REG, AMAX),
        (VMAX_REG, VMAX),
        (DMAX_REG, DMAX),
        (TVMAX, 0),
        (D1, 10),
        (VSTOP, 10),
        (TZEROWAIT, 0),
        (XTARGET, 0),
        (V2, 0),
        (A2, 0),
        (D2, 10),
        (RAMPMODE, 0), // Positioning mode
    ];

    for &(register, value) in CONFIGURATION {
        write_register!(&mut bus, &mut cs, register, value);
    }

    write_register!(&mut bus, &mut cs, CHOPCONF, CHOPCONF_VALUE);

    // Verify the four settings chatgpt said might fuck our shit up.
    const VERIFY: &[(u8, u32)] = &[
        (DRV_CONF, DRV_CONF_VALUE),
        (GLOBALSCALER, GLOBALSCALER_VALUE),
        (IHOLD_IRUN, IHOLD_IRUN_VALUE),
        (CHOPCONF, CHOPCONF_VALUE),
    ];

    for &(register, expected) in VERIFY {
        let actual = read_register!(&mut bus, &mut cs, register);
        if actual != expected {
            // we are cooked
            defmt::panic!(
                "motor: register verification failed: reg={}, got={}, expected={}",
                register,
                actual,
                expected
            );
        }
    }

    // not cooked :D
    drv_en_n.set_low();
    defmt::info!("motor ready; target=retracted");

    let mut commanded_deployed = false;
    let mut fault_poll_counter: u8 = 0;

    READY.signal(());
    let mut t = Ticker::every(Duration::from_millis(5));
    loop {
        t.next().await;

        let requested_deployed = *DEPLOYED.lock().await;

        if requested_deployed != commanded_deployed {
            let target = if requested_deployed {
                FULL_DEPLOY_POSITION
            } else {
                0
            };

            write_register!(&mut bus, &mut cs, XTARGET, target as u32);
            commanded_deployed = requested_deployed;

            defmt::info!(
                "motor target changed: deployed={}, target={}",
                requested_deployed,
                target
            );
        }

        // Check critical driver faults every 100 ms.
        fault_poll_counter += 1;
        if fault_poll_counter == 20 {
            fault_poll_counter = 0;

            let gstat = read_register!(&mut bus, &mut cs, GSTAT);
            let drv_status = read_register!(&mut bus, &mut cs, DRV_STATUS);

            if gstat & GSTAT_CRITICAL_MASK != 0 || drv_status & DRV_STATUS_CRITICAL_MASK != 0 {
                drv_en_n.set_high();
                defmt::error!("motor fault: GSTAT={}, DRV_STATUS={}", gstat, drv_status);
                Timer::after_secs(1).await;
                // GO AGAIN
                drv_en_n.set_low();
            }
        }
    }
}
