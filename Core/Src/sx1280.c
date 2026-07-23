/*
 * Fixed RF profile:
 *   2445 MHz, LoRa SF8, 203.125 kHz BW, CR 4/5,
 *   explicit header, payload CRC, standard IQ, 12-symbol preamble,
 *   +13 dBm requested TX power, and high-sensitivity RX.
 *
 *   AI told me that this would get 5000 ft, idk the math myself tho :(
 */

#include "sx1280.h"
#include "sx1280_port.h"
#include <stdint.h>
#include <string.h>

/* SX1280 command opcodes */
#define SX1280_CMD_GET_STATUS            0xC0
#define SX1280_CMD_WRITE_REGISTER        0x18
#define SX1280_CMD_READ_REGISTER         0x19
#define SX1280_CMD_WRITE_BUFFER          0x1A
#define SX1280_CMD_READ_BUFFER           0x1B
#define SX1280_CMD_SET_STANDBY           0x80
#define SX1280_CMD_SET_RX                0x82
#define SX1280_CMD_SET_TX                0x83
#define SX1280_CMD_SET_PACKET_TYPE       0x8A
#define SX1280_CMD_SET_RF_FREQUENCY      0x86
#define SX1280_CMD_SET_TX_PARAMS         0x8E
#define SX1280_CMD_SET_BUFFER_BASE_ADDR  0x8F
#define SX1280_CMD_SET_MOD_PARAMS        0x8B
#define SX1280_CMD_SET_PACKET_PARAMS     0x8C
#define SX1280_CMD_SET_DIO_IRQ_PARAMS    0x8D
#define SX1280_CMD_GET_RX_BUFFER_STATUS  0x17
#define SX1280_CMD_GET_IRQ_STATUS        0x15
#define SX1280_CMD_CLEAR_IRQ_STATUS      0x97

/* Packet type */
#define SX1280_PACKET_TYPE_LORA          0x01

/* IRQ bits */
#define SX1280_IRQ_TX_DONE               0x0001
#define SX1280_IRQ_RX_DONE               0x0002
#define SX1280_IRQ_HEADER_VALID          0x0010
#define SX1280_IRQ_HEADER_ERROR          0x0020
#define SX1280_IRQ_CRC_ERROR             0x0040
#define SX1280_IRQ_RX_TX_TIMEOUT         0x4000
#define SX1280_IRQ_ALL                   0xFFFF

/* LoRa modulation values from the SX1280 data sheet. */
#define SX1280_LORA_SF5                  0x50
#define SX1280_LORA_SF6                  0x60
#define SX1280_LORA_SF7                  0x70
#define SX1280_LORA_SF8                  0x80
#define SX1280_LORA_SF9                  0x90
#define SX1280_LORA_SF10                 0xA0
#define SX1280_LORA_SF11                 0xB0
#define SX1280_LORA_SF12                 0xC0

#define SX1280_LORA_BW_1625_KHZ          0x0A
#define SX1280_LORA_BW_812_KHZ           0x18
#define SX1280_LORA_BW_406_KHZ           0x26
#define SX1280_LORA_BW_203_KHZ           0x34

#define SX1280_LORA_CR_4_5               0x01
#define SX1280_LORA_CR_4_6               0x02
#define SX1280_LORA_CR_4_7               0x03
#define SX1280_LORA_CR_4_8               0x04
#define SX1280_LORA_CR_LI_4_5            0x05
#define SX1280_LORA_CR_LI_4_6            0x06
#define SX1280_LORA_CR_LI_4_8            0x07

#define SX1280_LORA_HEADER_EXPLICIT      0x00
#define SX1280_LORA_CRC_ON               0x20
#define SX1280_LORA_IQ_NORMAL            0x40

#define SX1280_STDBY_RC                  0x00
#define SX1280_RAMP_20_US                0xE0

/*
 * Fixed flight-link timeout settings for the 5,000 ft profile.
 * SetTx uses a 4 ms base and 500 counts, giving a 2.0 s radio-side timeout.
 * The MCU waits slightly longer so the radio IRQ is always handled first.
 */
#define SX1280_TX_TIMEOUT_BASE_4_MS       0x03
#define SX1280_TX_TIMEOUT_COUNT           500U
#define SX1280_MIN_SOFTWARE_TIMEOUT_MS    2500U

/* Registers used by the LoRa modem. */
#define SX1280_REG_RX_GAIN               0x0891
#define SX1280_REG_SF_ADDITIONAL_CONFIG  0x0925
#define SX1280_REG_FREQ_ERROR_COMP       0x093C

/*
 * All RF behavior is controlled from one structure.  To change the flight
 * profile, change this initializer rather than editing several functions.
 */
typedef struct
{
    uint32_t frequency_hz;
    uint8_t spreading_factor;
    uint8_t bandwidth;
    uint8_t coding_rate;
    uint8_t preamble_param;
    uint8_t header_type;
    uint8_t crc_mode;
    uint8_t iq_mode;
    int8_t tx_power_dbm;
    uint8_t high_sensitivity_rx;
} SX1280_LoRaConfig;

/*
 * Dedicated onboard profile for a 5,000 ft line-of-sight rocket telemetry link:
 *   - SF8 gives 3 dB more sensitivity than the previous SF7 setting.
 *   - 203 kHz is the narrowest SX1280 LoRa bandwidth and maximizes sensitivity.
 *   - CR 4/5 keeps airtime reasonable for live telemetry.
 *   - +13 dBm is the maximum SetTxParams command value for the bare SX1280.
 *   - High-sensitivity RX enables the upper LNA gain steps.
 *
 */
static const SX1280_LoRaConfig sx1280_default_config =
{
    .frequency_hz       = 2445000000UL,
    .spreading_factor   = SX1280_LORA_SF8,
    .bandwidth          = SX1280_LORA_BW_203_KHZ,
    .coding_rate        = SX1280_LORA_CR_4_5,
    .preamble_param     = 0x0C, /* 12 symbols: mantissa 12, exponent 0 */
    .header_type        = SX1280_LORA_HEADER_EXPLICIT,
    .crc_mode           = SX1280_LORA_CRC_ON,
    .iq_mode            = SX1280_LORA_IQ_NORMAL,
    .tx_power_dbm       = 13,
    .high_sensitivity_rx = 1
};

/* Used by TX and RX after initialization. */
static SX1280_LoRaConfig sx1280_active_config;

/*
 * Shared synchronous SPI transaction buffers.
 *
 * These are stored in static RAM instead of the call stack. The current radio
 * driver is synchronous and does not perform SPI transactions from an ISR, so
 * one shared pair is sufficient.
 */
static uint8_t sx1280_spi_tx_buffer[
    3U + SX1280_MAX_PAYLOAD_LEN];

static uint8_t sx1280_spi_rx_buffer[
    3U + SX1280_MAX_PAYLOAD_LEN];

// set the transmission to run, this action should not be interrupted
static HAL_StatusTypeDef sx1280_transfer(const uint8_t *tx, uint8_t *rx, uint16_t len)
{
    HAL_StatusTypeDef status;

    status = SX1280_PortWaitBusyLow(100);
    if (status != HAL_OK)
    {
        return status;
    }

    SX1280_PortCsLow();
    status = SX1280_PortSpiTxRx(tx, rx, len);
    SX1280_PortCsHigh();

    if (status != HAL_OK)
    {
        return status;
    }

    return SX1280_PortWaitBusyLow(100);
}

// preset sx1280 commands are:

static HAL_StatusTypeDef sx1280_command(uint8_t opcode, const uint8_t *args, uint8_t arg_len)
{
    uint8_t tx[16] = {0};
    uint8_t rx[16] = {0};

    if (arg_len > 15)
    {
        return HAL_ERROR;
    }

    tx[0] = opcode;

    if ((args != 0) && (arg_len > 0))
    {
        memcpy(&tx[1], args, arg_len);
    }

    return sx1280_transfer(tx, rx, (uint16_t)(arg_len + 1));
}

static HAL_StatusTypeDef sx1280_write_register(uint16_t address, uint8_t value)
{
    uint8_t tx[4];
    uint8_t rx[4] = {0};

    tx[0] = SX1280_CMD_WRITE_REGISTER;
    tx[1] = (uint8_t)(address >> 8);
    tx[2] = (uint8_t)(address & 0xFF);
    tx[3] = value;

    return sx1280_transfer(tx, rx, sizeof(tx));
}

static HAL_StatusTypeDef sx1280_read_register(uint16_t address, uint8_t *value)
{
    uint8_t tx[5] = {0};
    uint8_t rx[5] = {0};
    HAL_StatusTypeDef status;

    if (value == 0)
    {
        return HAL_ERROR;
    }

    tx[0] = SX1280_CMD_READ_REGISTER;
    tx[1] = (uint8_t)(address >> 8);
    tx[2] = (uint8_t)(address & 0xFF);
    tx[3] = 0x00; /* status/dummy byte */
    tx[4] = 0x00; /* clocks out one register byte */

    status = sx1280_transfer(tx, rx, sizeof(tx));
    if (status != HAL_OK)
    {
        return status;
    }

    *value = rx[4];
    return HAL_OK;
}

// specifically interacting with the tx/rx buffer
static HAL_StatusTypeDef sx1280_write_buffer(
    uint8_t offset,
    const uint8_t *data,
    uint8_t len)
{
    const uint16_t transfer_length =
        (uint16_t)len + 2U;

    if ((data == NULL) ||
        (len == 0U) ||
        (len > SX1280_MAX_PAYLOAD_LEN))
    {
        return HAL_ERROR;
    }

    /*
     * Clear only the part used for this SPI transaction.
     */
    memset(sx1280_spi_tx_buffer,
           0,
           transfer_length);

    memset(sx1280_spi_rx_buffer,
           0,
           transfer_length);

    sx1280_spi_tx_buffer[0] =
        SX1280_CMD_WRITE_BUFFER;

    sx1280_spi_tx_buffer[1] = offset;

    memcpy(&sx1280_spi_tx_buffer[2],
           data,
           len);

    return sx1280_transfer(
        sx1280_spi_tx_buffer,
        sx1280_spi_rx_buffer,
        transfer_length);
}

static HAL_StatusTypeDef sx1280_read_buffer(
    uint8_t offset,
    uint8_t *data,
    uint8_t len)
{
    const uint16_t transfer_length =
        (uint16_t)len + 3U;

    HAL_StatusTypeDef status;

    if ((data == NULL) ||
        (len == 0U) ||
        (len > SX1280_MAX_PAYLOAD_LEN))
    {
        return HAL_ERROR;
    }

    memset(sx1280_spi_tx_buffer,
           0,
           transfer_length);

    memset(sx1280_spi_rx_buffer,
           0,
           transfer_length);

    sx1280_spi_tx_buffer[0] =
        SX1280_CMD_READ_BUFFER;

    sx1280_spi_tx_buffer[1] = offset;
    sx1280_spi_tx_buffer[2] = 0x00U;

    status = sx1280_transfer(
        sx1280_spi_tx_buffer,
        sx1280_spi_rx_buffer,
        transfer_length);

    if (status != HAL_OK)
    {
        return status;
    }

    memcpy(data,
           &sx1280_spi_rx_buffer[3],
           len);

    return HAL_OK;
}

static HAL_StatusTypeDef sx1280_set_standby(void)
{
    uint8_t args[1] = {SX1280_STDBY_RC};
    return sx1280_command(SX1280_CMD_SET_STANDBY, args, sizeof(args));
}

// we would change this command if we want a different packet type but LoRa works great rn
static HAL_StatusTypeDef sx1280_set_packet_type_lora(void)
{
    uint8_t args[1] = {SX1280_PACKET_TYPE_LORA};
    return sx1280_command(SX1280_CMD_SET_PACKET_TYPE, args, sizeof(args));
}

static HAL_StatusTypeDef sx1280_set_rf_frequency(uint32_t freq_hz)
{
    /* rfFrequency = frequency_Hz * 2^18 / 52 MHz */
    uint32_t reg = (uint32_t)((((uint64_t)freq_hz << 18) + 26000000ULL) / 52000000ULL);
    uint8_t args[3];

    args[0] = (uint8_t)(reg >> 16);
    args[1] = (uint8_t)(reg >> 8);
    args[2] = (uint8_t)(reg);

    return sx1280_command(SX1280_CMD_SET_RF_FREQUENCY, args, sizeof(args));
}

// the tx/rx buffer can be adjusted in size
static HAL_StatusTypeDef sx1280_set_buffer_base_address(void)
{
    uint8_t args[2] = {
        0x00, /* TX base address */
        0x00  /* RX base address */
    };

    return sx1280_command(SX1280_CMD_SET_BUFFER_BASE_ADDR, args, sizeof(args));
}

static HAL_StatusTypeDef sx1280_validate_config(const SX1280_LoRaConfig *config)
{
    if (config == 0)
    {
        return HAL_ERROR;
    }

    if ((config->frequency_hz < 2400000000UL) ||
        (config->frequency_hz > 2500000000UL))
    {
        return HAL_ERROR;
    }

    switch (config->spreading_factor)
    {
        case SX1280_LORA_SF5:
        case SX1280_LORA_SF6:
        case SX1280_LORA_SF7:
        case SX1280_LORA_SF8:
        case SX1280_LORA_SF9:
        case SX1280_LORA_SF10:
        case SX1280_LORA_SF11:
        case SX1280_LORA_SF12:
            break;

        default:
            return HAL_ERROR;
    }

    switch (config->bandwidth)
    {
        case SX1280_LORA_BW_1625_KHZ:
        case SX1280_LORA_BW_812_KHZ:
        case SX1280_LORA_BW_406_KHZ:
        case SX1280_LORA_BW_203_KHZ:
            break;

        default:
            return HAL_ERROR;
    }

    if ((config->coding_rate < SX1280_LORA_CR_4_5) ||
        (config->coding_rate > SX1280_LORA_CR_LI_4_8) ||
        (config->preamble_param == 0) ||
        (config->tx_power_dbm < -18) ||
        (config->tx_power_dbm > 13))
    {
        return HAL_ERROR;
    }

    return HAL_OK;
}

static HAL_StatusTypeDef sx1280_set_modulation_params(const SX1280_LoRaConfig *config)
{
    uint8_t args[3] = {
        config->spreading_factor,
        config->bandwidth,
        config->coding_rate
    };

    return sx1280_command(SX1280_CMD_SET_MOD_PARAMS, args, sizeof(args));
}

static HAL_StatusTypeDef sx1280_set_packet_params(const SX1280_LoRaConfig *config,
                                                   uint8_t payload_len)
{
    uint8_t args[7] = {
        config->preamble_param,
        config->header_type,
        payload_len,
        config->crc_mode,
        config->iq_mode,
        0x00,
        0x00
    };

    return sx1280_command(SX1280_CMD_SET_PACKET_PARAMS, args, sizeof(args));
}

static HAL_StatusTypeDef sx1280_set_tx_params(const SX1280_LoRaConfig *config)
{
    /* Data sheet formula: Pout[dBm] = -18 + power_parameter. */
    uint8_t power_parameter = (uint8_t)(config->tx_power_dbm + 18);
    uint8_t args[2] = {
        power_parameter,
        SX1280_RAMP_20_US
    };

    return sx1280_command(SX1280_CMD_SET_TX_PARAMS, args, sizeof(args));
}

// spreading factor for modulation
static HAL_StatusTypeDef sx1280_apply_sf_registers(const SX1280_LoRaConfig *config)
{
    uint8_t sf_register_value;
    HAL_StatusTypeDef status;

    switch (config->spreading_factor)
    {
        case SX1280_LORA_SF5:
        case SX1280_LORA_SF6:
            sf_register_value = 0x1E;
            break;

        case SX1280_LORA_SF7:
        case SX1280_LORA_SF8:
            sf_register_value = 0x37;
            break;

        default: /* SF9 through SF12 */
            sf_register_value = 0x32;
            break;
    }

    status = sx1280_write_register(SX1280_REG_SF_ADDITIONAL_CONFIG,
                                    sf_register_value);
    if (status != HAL_OK)
    {
        return status;
    }

    return sx1280_write_register(SX1280_REG_FREQ_ERROR_COMP, 0x01);
}

// far away, it need to be able to detect
static HAL_StatusTypeDef sx1280_set_high_sensitivity(uint8_t enabled)
{
    uint8_t rx_gain;
    HAL_StatusTypeDef status;

    status = sx1280_read_register(SX1280_REG_RX_GAIN, &rx_gain);
    if (status != HAL_OK)
    {
        return status;
    }

    /* RxGain bits 7:6 select the LNA gain regime. */
    rx_gain &= (uint8_t)~0xC0U;
    if (enabled != 0)
    {
        rx_gain |= 0xC0U;
    }

    return sx1280_write_register(SX1280_REG_RX_GAIN, rx_gain);
}

// interrups aren't used seeing as the entire system needs to work on a cylce, might as well maintaint the cycle.
static HAL_StatusTypeDef sx1280_set_dio_irq_params(void)
{
    uint16_t irq_mask =
        SX1280_IRQ_TX_DONE |
        SX1280_IRQ_RX_DONE |
        SX1280_IRQ_HEADER_ERROR |
        SX1280_IRQ_CRC_ERROR |
        SX1280_IRQ_RX_TX_TIMEOUT;

    uint8_t args[8];

    args[0] = (uint8_t)(irq_mask >> 8);
    args[1] = (uint8_t)(irq_mask);

    /* Route all enabled IRQs to DIO1. */
    args[2] = (uint8_t)(irq_mask >> 8);
    args[3] = (uint8_t)(irq_mask);

    /* DIO2 and DIO3 disabled. */
    args[4] = 0x00;
    args[5] = 0x00;
    args[6] = 0x00;
    args[7] = 0x00;

    return sx1280_command(SX1280_CMD_SET_DIO_IRQ_PARAMS, args, sizeof(args));
}

static HAL_StatusTypeDef sx1280_clear_irq(uint16_t irq_mask)
{
    uint8_t args[2];

    args[0] = (uint8_t)(irq_mask >> 8);
    args[1] = (uint8_t)(irq_mask);

    return sx1280_command(SX1280_CMD_CLEAR_IRQ_STATUS, args, sizeof(args));
}

static HAL_StatusTypeDef sx1280_get_irq_status(uint16_t *irq_status)
{
    uint8_t tx[4] = {
        SX1280_CMD_GET_IRQ_STATUS,
        0x00,
        0x00,
        0x00
    };
    uint8_t rx[4] = {0};
    HAL_StatusTypeDef status;

    if (irq_status == 0)
    {
        return HAL_ERROR;
    }

    status = sx1280_transfer(tx, rx, sizeof(tx));
    if (status != HAL_OK)
    {
        return status;
    }

    *irq_status = ((uint16_t)rx[2] << 8) | rx[3];
    return HAL_OK;
}

// is a payload here?
static HAL_StatusTypeDef sx1280_get_rx_buffer_status(uint8_t *payload_len,
                                                      uint8_t *rx_start_pointer)
{
    uint8_t tx[4] = {
        SX1280_CMD_GET_RX_BUFFER_STATUS,
        0x00,
        0x00,
        0x00
    };
    uint8_t rx[4] = {0};
    HAL_StatusTypeDef status;

    if ((payload_len == 0) || (rx_start_pointer == 0))
    {
        return HAL_ERROR;
    }

    status = sx1280_transfer(tx, rx, sizeof(tx));
    if (status != HAL_OK)
    {
        return status;
    }

    *payload_len = rx[2];
    *rx_start_pointer = rx[3];
    return HAL_OK;
}

static HAL_StatusTypeDef sx1280_apply_lora_config(const SX1280_LoRaConfig *config)
{
    HAL_StatusTypeDef status;

    status = sx1280_validate_config(config);
    if (status != HAL_OK)
    {
        return status;
    }

    status = sx1280_set_rf_frequency(config->frequency_hz);
    if (status != HAL_OK) return status;

    status = sx1280_set_buffer_base_address();
    if (status != HAL_OK) return status;

    status = sx1280_set_modulation_params(config);
    if (status != HAL_OK) return status;

    status = sx1280_apply_sf_registers(config);
    if (status != HAL_OK) return status;

    status = sx1280_set_high_sensitivity(config->high_sensitivity_rx);
    if (status != HAL_OK) return status;

    status = sx1280_set_tx_params(config);
    if (status != HAL_OK) return status;

    status = sx1280_set_dio_irq_params();
    if (status != HAL_OK) return status;

    status = sx1280_clear_irq(SX1280_IRQ_ALL);
    if (status != HAL_OK) return status;

    memcpy(&sx1280_active_config, config, sizeof(sx1280_active_config));
    return HAL_OK;
}

// listening
HAL_StatusTypeDef SX1280_StartRxContinuous(void)
{
    HAL_StatusTypeDef status;
    uint8_t args[3] = {
        0x03, /* 4 ms period base */
        0xFF,
        0xFF  /* continuous RX */
    };

    status = sx1280_set_standby();
    if (status != HAL_OK) return status;

    status = sx1280_set_packet_params(&sx1280_active_config,
                                       SX1280_MAX_PAYLOAD_LEN);
    if (status != HAL_OK) return status;

    status = sx1280_clear_irq(SX1280_IRQ_ALL);
    if (status != HAL_OK) return status;

    return sx1280_command(SX1280_CMD_SET_RX, args, sizeof(args));
}

// just get the lora talking to the MCU
HAL_StatusTypeDef SX1280_InitLoRa(void)
{
    HAL_StatusTypeDef status;

    status = SX1280_PortInit();
    if (status != HAL_OK) return status;

    SX1280_PortReset();

    status = SX1280_PortWaitBusyLow(500);
    if (status != HAL_OK) return status;

    status = sx1280_set_standby();
    if (status != HAL_OK) return status;

    status = sx1280_set_packet_type_lora();
    if (status != HAL_OK) return status;

    status = sx1280_apply_lora_config(&sx1280_default_config);
    if (status != HAL_OK) return status;

    return SX1280_StartRxContinuous();
}

// I send data now, good luck everyone
HAL_StatusTypeDef SX1280_Transmit(const uint8_t *data, uint8_t len, uint32_t timeout_ms)
{
    HAL_StatusTypeDef status;
    uint16_t irq_status;
    uint32_t start;
    uint8_t tx_args[3] = {
        SX1280_TX_TIMEOUT_BASE_4_MS,
        (uint8_t)(SX1280_TX_TIMEOUT_COUNT >> 8),
        (uint8_t)(SX1280_TX_TIMEOUT_COUNT)
    };

    if ((data == 0) || (len == 0))
    {
        return HAL_ERROR;
    }

    /*
     * The existing radio bridge passes 1000 ms.  That is unnecessarily close
     * to the worst-case packet time for the long-range profile, so enforce a
     * safer minimum inside the driver without requiring radio_bridge.c changes.
     */
    if (timeout_ms < SX1280_MIN_SOFTWARE_TIMEOUT_MS)
    {
        timeout_ms = SX1280_MIN_SOFTWARE_TIMEOUT_MS;
    }

    status = sx1280_set_standby();
    if (status != HAL_OK) return status;

    status = sx1280_set_packet_params(&sx1280_active_config, len);
    if (status != HAL_OK) return status;

    status = sx1280_clear_irq(SX1280_IRQ_ALL);
    if (status != HAL_OK) return status;

    status = sx1280_write_buffer(0x00, data, len);
    if (status != HAL_OK) return status;

    status = sx1280_command(SX1280_CMD_SET_TX, tx_args, sizeof(tx_args));
    if (status != HAL_OK) return status;

    start = HAL_GetTick();

    do
    {
        status = sx1280_get_irq_status(&irq_status);
        if (status != HAL_OK) return status;

        if ((irq_status & SX1280_IRQ_TX_DONE) != 0)
        {
            sx1280_clear_irq(SX1280_IRQ_TX_DONE);
            return SX1280_StartRxContinuous();
        }

        if ((irq_status & SX1280_IRQ_RX_TX_TIMEOUT) != 0)
        {
            sx1280_clear_irq(SX1280_IRQ_RX_TX_TIMEOUT);
            SX1280_StartRxContinuous();
            return HAL_TIMEOUT;
        }

        /* Avoid continuously occupying SPI while waiting for TxDone. */
        HAL_Delay(1);
    }
    while ((HAL_GetTick() - start) < timeout_ms);

    SX1280_StartRxContinuous();
    return HAL_TIMEOUT;
}

// acquire packet and place it into the data
HAL_StatusTypeDef SX1280_ReadPacketIfAvailable(uint8_t *data, uint8_t *len)
{
    HAL_StatusTypeDef status;
    uint16_t irq_status;
    uint8_t payload_len;
    uint8_t start_pointer;

    if ((data == 0) || (len == 0))
    {
        return HAL_ERROR;
    }

    status = sx1280_get_irq_status(&irq_status);
    if (status != HAL_OK)
    {
        return status;
    }

    if (irq_status == 0)
    {
        return HAL_BUSY;
    }

    if ((irq_status & (SX1280_IRQ_HEADER_ERROR |
                       SX1280_IRQ_CRC_ERROR |
                       SX1280_IRQ_RX_TX_TIMEOUT)) != 0)
    {
        sx1280_clear_irq(irq_status);
        SX1280_StartRxContinuous();
        return HAL_ERROR;
    }

    if ((irq_status & SX1280_IRQ_RX_DONE) == 0)
    {
        sx1280_clear_irq(irq_status);
        return HAL_BUSY;
    }

    status = sx1280_get_rx_buffer_status(&payload_len, &start_pointer);
    if (status != HAL_OK)
    {
        sx1280_clear_irq(irq_status);
        SX1280_StartRxContinuous();
        return status;
    }

    status = sx1280_read_buffer(start_pointer, data, payload_len);

    sx1280_clear_irq(irq_status);
    SX1280_StartRxContinuous();

    if (status != HAL_OK)
    {
        return status;
    }

    *len = payload_len;
    return HAL_OK;
}
