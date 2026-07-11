use embassy_stm32::{
    gpio::Speed,
    i2c::{Config, I2c, Master},
    mode::Async,
    time::Hertz,
};

pub mod baro;
pub mod imu;
pub mod magnet;

/// Returns a reasonable default config for I2c device busses.
fn i2c_config() -> Config {
    const FREQUENCY_KILOHERTZ: u32 = 400;
    let mut config = Config::default();
    config.frequency = Hertz::khz(FREQUENCY_KILOHERTZ);
    // This is already the default but setting it explicitly
    // in case this changes in a future embassy release.
    config.gpio_speed = Speed::Medium;
    config
}

/// Configures the addressed device using `reg_value_pairs`.
///
/// Panics on failure.
async fn configure(
    device_bus: &mut I2c<'_, Async, Master>,
    device_address: u8,
    device_name: &str,
    reg_value_pairs: &[(u8, u8)],
) {
    for &(reg, value) in reg_value_pairs {
        if let Err(e) = device_bus.write(device_address, &[reg, value]).await {
            defmt::panic!(
                "{}: configuration failure with {}|{} (reg|value): {}",
                device_name,
                reg,
                value,
                e
            );
        }
    }
    defmt::info!("{}: configuration success", device_name);
}

/// Checks if `device_bus` has a device addressable through `device_address`,
/// using the "whoami" (or equivalent) I2c register and it's expected value.
///
/// Panics on failure.
async fn check_connection(
    device_bus: &mut I2c<'_, Async, Master>,
    device_address: u8,
    device_name: &str,
    who_am_i_register: u8,
    who_am_i_expected: u8,
) {
    let write = [who_am_i_register];
    let mut read = [0];

    if let Err(e) = device_bus
        .write_read(device_address, &write, &mut read)
        .await
    {
        defmt::panic!("{}: connection failure: {}", device_name, e);
    }

    if read[0] != who_am_i_expected {
        defmt::panic!(
            "{}: whoami failure: got {} but expected {}",
            device_name,
            read[0],
            who_am_i_expected
        );
    }

    defmt::info!("{}: connection and whoami success", device_name);
}
