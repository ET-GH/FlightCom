use super::{check_connection, configure, i2c_config};
use crate::{
    CANCEL_SENSORS, get_dt_s,
    resources::{BaroResources, Irq},
};
use embassy_stm32::{exti::ExtiInput, gpio::Pull, i2c::I2c};
use embassy_sync::{blocking_mutex::raw::ThreadModeRawMutex, signal::Signal};
use embassy_time::Instant;

const NAME: &str = "baro";

pub static DATA: Signal<ThreadModeRawMutex, Data> = Signal::new();
pub static READY: Signal<ThreadModeRawMutex, ()> = Signal::new();

pub struct Data {
    pub pressure_pa: f32,
    pub temperature_c: f32,

    /// Time between this and the previous sample.
    pub dt_s: f32,
}

#[embassy_executor::task]
pub async fn start(r: BaroResources) {
    let mut bus = I2c::new(r.peri, r.scl, r.sda, r.tx_dma, r.rx_dma, Irq, i2c_config());
    let mut int = ExtiInput::new(r.int, r.int_exti, Pull::None, Irq);

    const ADDRESS: u8 = 0b1110110;
    const WHO_AM_I_REGISTER: u8 = 0x00;
    const WHO_AM_I_EXPECTED: u8 = 0x50;

    check_connection(
        &mut bus,
        ADDRESS,
        NAME,
        WHO_AM_I_REGISTER,
        WHO_AM_I_EXPECTED,
    )
    .await;

    // Register addresses.
    const INT_STATUS: u8 = 0x11;
    const INT_CTRL: u8 = 0x19;
    const PWR_CTRL: u8 = 0x1B;
    const OSR: u8 = 0x1C;
    const ODR: u8 = 0x1D;
    const CONFIG: u8 = 0x1F;

    // Register configurations.
    const INT_CTRL_DRDY_ACTIVE_HIGH: u8 = (1 << 6) | (1 << 2) | (1 << 1);
    const OSR_HIGH_RESOLUTION: u8 = 0b_000_011; // pressure ×8, temperature ×1.
    const ODR_50HZ: u8 = 0x02;
    const CONFIG_IIR_COEF_3: u8 = 0b_010 << 1;
    const PWR_CTRL_NORMAL_PRESS_TEMP: u8 = (0b11 << 4) | (1 << 1) | 1;

    let mut calibration = {
        const CALIB_LEN: usize = 21;
        const NVM_PAR_T1_L: u8 = 0x31;
        let write = [NVM_PAR_T1_L];
        let mut read = [0; CALIB_LEN];

        if let Err(e) = bus.write_read(ADDRESS, &write, &mut read).await {
            defmt::panic!("{}: calibration failed: {}", NAME, e);
        }

        Calibration::new(&read)
    };

    const REG_VALUE_PAIRS: [(u8, u8); 5] = [
        (INT_CTRL, INT_CTRL_DRDY_ACTIVE_HIGH),
        (CONFIG, CONFIG_IIR_COEF_3),
        (OSR, OSR_HIGH_RESOLUTION),
        (ODR, ODR_50HZ),
        // Enable normal mode last.
        (PWR_CTRL, PWR_CTRL_NORMAL_PRESS_TEMP),
    ];
    configure(&mut bus, ADDRESS, NAME, &REG_VALUE_PAIRS).await;

    // We're going to burst read 6 consecutive bytes/registers...
    const LEN: usize = 6;
    // ...and this is the address of the first register.
    const DATA_0: u8 = 0x04;
    let write = [DATA_0];
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

        {
            let write = [INT_STATUS];
            let mut read = [0; 1];
            // Need to explicitly clear the BMP388 interrupt flag.
            // WHY THE FUCK DO WE NEED TO DO THIS EXPLCIAJPRJTOIUENOUTNOUSDNVCOUA
            if let Err(e) = bus.write_read(ADDRESS, &write, &mut read).await {
                defmt::error!("{}: INT_STATUS read failed: {}", NAME, e);
                continue;
            }
        }

        match bus.write_read(ADDRESS, &write, &mut read).await {
            Ok(_) => {
                // Give it to that bitch raw type shit.
                let raw_pressure =
                    u32::from(read[0]) | (u32::from(read[1]) << 8) | (u32::from(read[2]) << 16);
                let raw_temperature =
                    u32::from(read[3]) | (u32::from(read[4]) << 8) | (u32::from(read[5]) << 16);

                // TEMPERATURE MUST BE COMPENSATED FIRST YOU DUNCE!!
                let temperature_c = calibration.compensate_temperature(raw_temperature);
                let pressure_pa = calibration.compensate_pressure(raw_pressure);

                DATA.signal(Data {
                    pressure_pa,
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

struct Calibration {
    par_t1: f32,
    par_t2: f32,
    par_t3: f32,
    par_p1: f32,
    par_p2: f32,
    par_p3: f32,
    par_p4: f32,
    par_p5: f32,
    par_p6: f32,
    par_p7: f32,
    par_p8: f32,
    par_p9: f32,
    par_p10: f32,
    par_p11: f32,
    t_lin: f32,
}

impl Calibration {
    fn new(bytes: &[u8; 21]) -> Self {
        let nvm_par_t1 = u16::from_le_bytes([bytes[0], bytes[1]]) as f32;
        let nvm_par_t2 = u16::from_le_bytes([bytes[2], bytes[3]]) as f32;
        let nvm_par_t3 = i8::from_le_bytes([bytes[4]]) as f32;

        let nvm_par_p1 = i16::from_le_bytes([bytes[5], bytes[6]]) as f32;
        let nvm_par_p2 = i16::from_le_bytes([bytes[7], bytes[8]]) as f32;
        let nvm_par_p3 = i8::from_le_bytes([bytes[9]]) as f32;
        let nvm_par_p4 = i8::from_le_bytes([bytes[10]]) as f32;
        let nvm_par_p5 = u16::from_le_bytes([bytes[11], bytes[12]]) as f32;
        let nvm_par_p6 = u16::from_le_bytes([bytes[13], bytes[14]]) as f32;
        let nvm_par_p7 = i8::from_le_bytes([bytes[15]]) as f32;
        let nvm_par_p8 = i8::from_le_bytes([bytes[16]]) as f32;
        let nvm_par_p9 = i16::from_le_bytes([bytes[17], bytes[18]]) as f32;
        let nvm_par_p10 = i8::from_le_bytes([bytes[19]]) as f32;
        let nvm_par_p11 = i8::from_le_bytes([bytes[20]]) as f32;

        let t_lin = 0.0;

        Self {
            par_t1: nvm_par_t1 / 0.00390625,
            par_t2: nvm_par_t2 / 1_073_741_824.0,
            par_t3: nvm_par_t3 / 281_474_976_710_656.0,
            par_p1: (nvm_par_p1 - 16_384.0) / 1_048_576.0,
            par_p2: (nvm_par_p2 - 16_384.0) / 536_870_912.0,
            par_p3: nvm_par_p3 / 4_294_967_296.0,
            par_p4: nvm_par_p4 / 137_438_953_472.0,
            par_p5: nvm_par_p5 / 0.125,
            par_p6: nvm_par_p6 / 64.0,
            par_p7: nvm_par_p7 / 256.0,
            par_p8: nvm_par_p8 / 32_768.0,
            par_p9: nvm_par_p9 / 281_474_976_710_656.0,
            par_p10: nvm_par_p10 / 281_474_976_710_656.0,
            par_p11: nvm_par_p11 / 36_893_488_147_419_103_232.0,
            t_lin,
        }
    }

    fn compensate_temperature(&mut self, raw: u32) -> f32 {
        let raw = raw as f32;
        let partial_data1 = raw - self.par_t1;
        let partial_data2 = partial_data1 * self.par_t2;
        self.t_lin = partial_data2 + partial_data1 * partial_data1 * self.par_t3;
        self.t_lin
    }

    fn compensate_pressure(&self, raw: u32) -> f32 {
        let raw = raw as f32;
        let partial_data1 = self.par_p6 * self.t_lin;
        let partial_data2 = self.par_p7 * self.t_lin * self.t_lin;
        let partial_data3 = self.par_p8 * self.t_lin * self.t_lin * self.t_lin;
        let partial_out1 = self.par_p5 + partial_data1 + partial_data2 + partial_data3;

        let partial_data1 = self.par_p2 * self.t_lin;
        let partial_data2 = self.par_p3 * self.t_lin * self.t_lin;
        let partial_data3 = self.par_p4 * self.t_lin * self.t_lin * self.t_lin;
        let partial_out2 = raw * (self.par_p1 + partial_data1 + partial_data2 + partial_data3);

        let partial_data1 = raw * raw;
        let partial_data2 = self.par_p9 + self.par_p10 * self.t_lin;
        let partial_data3 = partial_data1 * partial_data2;
        let partial_data4 = partial_data3 + raw * raw * raw * self.par_p11;

        partial_out1 + partial_out2 + partial_data4
    }
}
