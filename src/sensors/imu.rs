use super::{check_connection, configure, i2c_config};
use crate::{
    CANCEL_SENSORS, get_dt_s,
    resources::{ImuResources, Irq},
};
use embassy_stm32::{exti::ExtiInput, gpio::Pull, i2c::I2c};
use embassy_sync::{blocking_mutex::raw::ThreadModeRawMutex, signal::Signal};
use embassy_time::Instant;
use nalgebra::Vector3;

const NAME: &str = "imu";

pub static DATA: Signal<ThreadModeRawMutex, Data> = Signal::new();
pub static READY: Signal<ThreadModeRawMutex, ()> = Signal::new();

pub struct Data {
    pub accel_g: Vector3<f32>,
    pub gyro_dps: Vector3<f32>,
    pub temperature_c: f32,

    /// Time between this and the previous sample.
    pub dt_s: f32,
}

impl Data {
    pub fn magnitude(&self) -> f32 {
        let x_2 = self.accel_g.x * self.accel_g.x;
        let y_2 = self.accel_g.y * self.accel_g.y;
        let z_2 = self.accel_g.z * self.accel_g.z;
        libm::sqrtf(x_2 + y_2 + z_2)
    }
}

#[embassy_executor::task]
pub async fn start(r: ImuResources) {
    let mut bus = I2c::new(r.peri, r.scl, r.sda, r.tx_dma, r.rx_dma, Irq, i2c_config());
    let mut int = ExtiInput::new(r.int, r.int_exti, Pull::None, Irq);

    const ADDRESS: u8 = 0b1101010;
    const WHO_AM_I_REGISTER: u8 = 0x0F;
    const WHO_AM_I_EXPECTED: u8 = 0x70;

    check_connection(
        &mut bus,
        ADDRESS,
        NAME,
        WHO_AM_I_REGISTER,
        WHO_AM_I_EXPECTED,
    )
    .await;

    // Register addresses.
    const INT1_CTRL: u8 = 0x0D;
    const CTRL1: u8 = 0x10; // accel OP_MODE + ODR
    const CTRL2: u8 = 0x11; // gyro OP_MODE + ODR
    const CTRL6: u8 = 0x15; // gyro FS
    const CTRL8: u8 = 0x17; // accel FS

    // Register configurations.
    const INT1_CTRL_DRDY_XL: u8 = 0b_0000_0001;
    const CONFIG: u8 = 0b_0001_0111; // HA-ODR + 240 hz
    const CTRL1_XL_CONFIG: u8 = CONFIG;
    const CTRL2_G_CONFIG: u8 = CONFIG;
    const CTRL6_FS_G: u8 = 0b_0000_1100; // 4000 dps
    const CTRL8_FS_XL: u8 = 0b_0000_0111; // 32 g

    const REG_VALUE_PAIRS: [(u8, u8); 5] = [
        (INT1_CTRL, INT1_CTRL_DRDY_XL),
        (CTRL6, CTRL6_FS_G),
        (CTRL8, CTRL8_FS_XL),
        // Enable gyro first, then accel.
        (CTRL2, CTRL2_G_CONFIG),
        (CTRL1, CTRL1_XL_CONFIG),
    ];
    configure(&mut bus, ADDRESS, NAME, &REG_VALUE_PAIRS).await;

    const ACCEL_G_PER_LSB: f32 = 0.000_976;
    const GYRO_DEG_S_PER_LSB: f32 = 0.140;
    const TEMP_C_PER_LSB: f32 = 1.0 / 256.0;

    // We're going to burst read 14 consecutive bytes/registers...
    const LEN: usize = 14;
    // ...and this is the address of the first register.
    const OUT_TEMP_L: u8 = 0x20;
    let write = [OUT_TEMP_L];
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
                let temperature_c =
                    25.0 + i16::from_le_bytes([read[0], read[1]]) as f32 * TEMP_C_PER_LSB;

                let gx = i16::from_le_bytes([read[2], read[3]]) as f32 * GYRO_DEG_S_PER_LSB;
                let gy = i16::from_le_bytes([read[4], read[5]]) as f32 * GYRO_DEG_S_PER_LSB;
                let gz = i16::from_le_bytes([read[6], read[7]]) as f32 * GYRO_DEG_S_PER_LSB;

                let ax = i16::from_le_bytes([read[8], read[9]]) as f32 * ACCEL_G_PER_LSB;
                let ay = i16::from_le_bytes([read[10], read[11]]) as f32 * ACCEL_G_PER_LSB;
                let az = i16::from_le_bytes([read[12], read[13]]) as f32 * ACCEL_G_PER_LSB;

                let accel_g = Vector3::new(ax, -ay, az);
                let gyro_dps = Vector3::new(gx, -gy, gz);

                DATA.signal(Data {
                    accel_g,
                    gyro_dps,
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
