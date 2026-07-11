use super::{check_connection, configure, i2c_config};
use crate::{
    CANCEL_SENSORS, get_dt_s,
    resources::{Irq, MagnetResources},
};
use embassy_stm32::{exti::ExtiInput, gpio::Pull, i2c::I2c};
use embassy_sync::{blocking_mutex::raw::ThreadModeRawMutex, signal::Signal};
use embassy_time::Instant;
use nalgebra::Vector3;

const NAME: &str = "magnet";

pub static DATA: Signal<ThreadModeRawMutex, Data> = Signal::new();
pub static READY: Signal<ThreadModeRawMutex, ()> = Signal::new();

pub struct Data {
    pub field_ut: Vector3<f32>,
    pub temperature_c: f32,

    /// Time between this and the previous sample.
    pub dt_s: f32,
}

#[embassy_executor::task]
pub async fn start(r: MagnetResources) {
    let mut bus = I2c::new(r.peri, r.scl, r.sda, r.tx_dma, r.rx_dma, Irq, i2c_config());
    let mut int = ExtiInput::new(r.int, r.int_exti, Pull::None, Irq);

    const ADDRESS: u8 = 0b0011110;
    const WHO_AM_I_REGISTER: u8 = 0x4F;
    const WHO_AM_I_EXPECTED: u8 = 0x40;

    check_connection(
        &mut bus,
        ADDRESS,
        NAME,
        WHO_AM_I_REGISTER,
        WHO_AM_I_EXPECTED,
    )
    .await;

    // Register addresses.
    const CFG_REG_A: u8 = 0x60; // temp comp + mode + ODR
    const CFG_REG_B: u8 = 0x61; // low-pass filter
    const CFG_REG_C: u8 = 0x62; // data-ready pin + block data update

    // Register configurations.
    const CFG_REG_A_CONFIG: u8 = 0b_1000_1100; // 100 hz
    const CFG_REG_B_CONFIG: u8 = 0b_0000_0011; // offset cancelation + lpf
    const CFG_REG_C_CONFIG: u8 = 0b_0001_0001; // BDU + DRDY_on_PIN

    const REG_VALUE_PAIRS: [(u8, u8); 3] = [
        (CFG_REG_B, CFG_REG_B_CONFIG),
        (CFG_REG_C, CFG_REG_C_CONFIG),
        // Enable output last.
        (CFG_REG_A, CFG_REG_A_CONFIG),
    ];

    configure(&mut bus, ADDRESS, NAME, &REG_VALUE_PAIRS).await;

    const MAG_UT_PER_LSB: f32 = 0.15;
    const TEMP_C_PER_LSB: f32 = 0.125;

    // We're going to burst read 8 consecutive bytes/registers...
    const LEN: usize = 8;
    // ...and this is the address of the first register.
    // The MSB is needed to enable I2C address auto-increment on LIS2MDL.
    const OUTX_L_REG: u8 = 0x68 | 0x80;
    let write = [OUTX_L_REG];
    let mut read = [0; LEN];

    READY.signal(());
    let mut last = Instant::now();
    loop {
        int.wait_for_high().await;

        let dt_s = get_dt_s(&mut last);

        if CANCEL_SENSORS.signaled() {
            core::hint::cold_path();
            return;
        }

        match bus.write_read(ADDRESS, &write, &mut read).await {
            Ok(_) => {
                let mx = i16::from_le_bytes([read[0], read[1]]) as f32 * MAG_UT_PER_LSB;
                let my = i16::from_le_bytes([read[2], read[3]]) as f32 * MAG_UT_PER_LSB;
                let mz = i16::from_le_bytes([read[4], read[5]]) as f32 * MAG_UT_PER_LSB;

                let temperature_c =
                    25.0 + i16::from_le_bytes([read[6], read[7]]) as f32 * TEMP_C_PER_LSB;

                let field_ut = Vector3::new(-mx, -my, mz);

                DATA.signal(Data {
                    field_ut,
                    temperature_c,
                    dt_s,
                });
            }

            Err(e) => {
                defmt::error!("{}: data acquisition failed: {}", NAME, e);
            }
        }
    }
}
