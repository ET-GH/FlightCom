//! MCU pin assignments and interrupt bindings.

use assign_resources::assign_resources;
use embassy_stm32::{
    Peri, bind_interrupts, dma, exti, i2c, interrupt::typelevel, peripherals, usb,
};

assign_resources! {
    // leds my beloved, some time in the furute you
    // shall have your moment to shine
    leds: LedResources {
        led_1: PA1,
        led_2: PA0,
        led_3: PC3,
        led_4: PC0,
        led_5: PB5,
        led_6: PB4,
    }

    lora: LoraResources {
        peri: SPI1,

        cs: PA3,
        miso: PA6,
        mosi: PA7,
        sck: PA5,

        tx_dma: GPDMA1_CH0,
        rx_dma: GPDMA1_CH1,

        reset_n: PA2,

        dio1: PC4,
        dio1_exti: EXTI4,

        busy: PC5,
        busy_exti: EXTI5,
    }

    motor: MotorResources {
        peri: SPI2,

        cs: PB0,
        miso: PC2,
        mosi: PC1,
        sck: PB13,

        tx_dma: GPDMA1_CH2,
        rx_dma: GPDMA1_CH3,

        drv_en_n: PB1,
        sleep_n: PB2,

        diag0: PC7,
        diag0_exti: EXTI7,

        diag1: PB15,
        diag1_exti: EXTI15,
    }

    flash: FlashResources {
        peri: SPI3,

        cs: PD2,
        miso: PC11,
        mosi: PC12,
        sck: PC10,

        tx_dma: GPDMA1_CH4,
        rx_dma: GPDMA1_CH5,
    }

    imu: ImuResources {
        peri: I2C1,

        scl: PB8,
        sda: PB7,

        tx_dma: GPDMA2_CH0,
        rx_dma: GPDMA2_CH1,

        int: PB6,
        int_exti: EXTI6,
    }

    baro: BaroResources {
        peri: I2C2,

        scl: PB10,
        sda: PB12,

        tx_dma: GPDMA2_CH2,
        rx_dma: GPDMA2_CH3,

        int: PB14,
        int_exti: EXTI14,
    }

    magnet: MagnetResources {
        peri: I2C3,

        scl: PA8,
        sda: PC9,

        tx_dma: GPDMA2_CH4,
        rx_dma: GPDMA2_CH5,

        int: PC8,
        int_exti: EXTI8,
    }

    usb: UsbResources {
        peri: USB,

        dp: PA12,
        dm: PA11,

        vbus_sense: PA9,
        vbus_sense_exti: EXTI9,
    }
}

// All these channels we ain't even using but it's nice to look at :)
bind_interrupts!(pub struct Irq {
    // IMU
    I2C1_EV => i2c::EventInterruptHandler<peripherals::I2C1>;
    I2C1_ER => i2c::ErrorInterruptHandler<peripherals::I2C1>;

    // Barometer
    I2C2_EV => i2c::EventInterruptHandler<peripherals::I2C2>;
    I2C2_ER => i2c::ErrorInterruptHandler<peripherals::I2C2>;

    // Magnetometer
    I2C3_EV => i2c::EventInterruptHandler<peripherals::I2C3>;
    I2C3_ER => i2c::ErrorInterruptHandler<peripherals::I2C3>;

    // Group 1: SPI devices
    GPDMA1_CHANNEL0 => dma::InterruptHandler<peripherals::GPDMA1_CH0>; // Lora Tx
    GPDMA1_CHANNEL1 => dma::InterruptHandler<peripherals::GPDMA1_CH1>; // Lora Rx
    GPDMA1_CHANNEL2 => dma::InterruptHandler<peripherals::GPDMA1_CH2>; // Motor Tx
    GPDMA1_CHANNEL3 => dma::InterruptHandler<peripherals::GPDMA1_CH3>; // Motor Rx
    GPDMA1_CHANNEL4 => dma::InterruptHandler<peripherals::GPDMA1_CH4>; // Flash Tx
    GPDMA1_CHANNEL5 => dma::InterruptHandler<peripherals::GPDMA1_CH5>; // Flash Rx

    // Group 2: I2C devices
    GPDMA2_CHANNEL0 => dma::InterruptHandler<peripherals::GPDMA2_CH0>; // IMU Tx
    GPDMA2_CHANNEL1 => dma::InterruptHandler<peripherals::GPDMA2_CH1>; // IMU Rx
    GPDMA2_CHANNEL2 => dma::InterruptHandler<peripherals::GPDMA2_CH2>; // Baro Tx
    GPDMA2_CHANNEL3 => dma::InterruptHandler<peripherals::GPDMA2_CH3>; // Baro Rx
    GPDMA2_CHANNEL4 => dma::InterruptHandler<peripherals::GPDMA2_CH4>; // Magnet Tx
    GPDMA2_CHANNEL5 => dma::InterruptHandler<peripherals::GPDMA2_CH5>; // Magnet Rx

    EXTI4 => exti::InterruptHandler<typelevel::EXTI4>; // Lora dio1
    EXTI5 => exti::InterruptHandler<typelevel::EXTI5>; // Lora busy
    EXTI6 => exti::InterruptHandler<typelevel::EXTI6>; // IMU int
    EXTI7 => exti::InterruptHandler<typelevel::EXTI7>; // Motor diag0
    EXTI8 => exti::InterruptHandler<typelevel::EXTI8>; // Magnet int
    EXTI9 => exti::InterruptHandler<typelevel::EXTI9>; // VBUS
    EXTI14 => exti::InterruptHandler<typelevel::EXTI14>; // Baro int
    EXTI15 => exti::InterruptHandler<typelevel::EXTI15>; // Motor diag1

    // Obviously this is for USB you fucking idiot
    USB_DRD_FS => usb::InterruptHandler<peripherals::USB>;
});
