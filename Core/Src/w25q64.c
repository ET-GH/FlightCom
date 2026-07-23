/**
 * @file w25q64.c
 * @brief Blocking SPI driver and power-loss-safe Rocket Protocol flight log for
 *        the Winbond W25Q64JV 64-Mbit serial NOR flash.
 *
 * Hardware assumptions
 * --------------------
 * - The flash is connected to the SPI handle and chip-select GPIO supplied to
 *   W25Q64_Attach().
 * - Standard single-line SPI is used. The W25Q64JV accepts SPI mode 0
 *   (clock idle low, data captured on the rising edge), which matches the
 *   supplied SPI3 configuration.
 * - /WP and /HOLD are held high by the board when standard SPI is used.
 *
 * Low-level behavior
 * ------------------
 * - All public low-level calls are synchronous and use STM32 HAL blocking SPI.
 * - Writes are split at 256-byte page boundaries. This is mandatory because a
 *   Page Program command wraps within one page if more bytes are clocked.
 * - Erases are aligned to 4-KiB sectors.
 * - Write Enable Latch (WEL) and BUSY are checked for every program/erase.
 * - Range checks are written without unsigned addition overflow.
 *
 * Rocket log behavior
 * -------------------
 * - Sector 0 is intentionally reserved for future configuration/metadata.
 * - Sectors 1..2047 contain fixed 48-byte records. Records never cross a
 *   sector boundary; each 4-KiB sector holds 85 records with 16 unused bytes.
 * - Each record stores the exact 26-byte RocketTelemetryPayload in stable
 *   little-endian form, plus its original millisecond timestamp, sequence,
 *   protocol version, CRC-16/CCITT-FALSE, and a commit byte.
 * - The commit byte is programmed last. A reset or power loss before that byte
 *   is written leaves the record uncommitted and prevents partially written
 *   telemetry from being exported as valid data.
 * - No sector is erased automatically. When full, logging stops instead of
 *   destroying an earlier flight. W25Q64_LogErase() must be called explicitly
 *   while the rocket is safely on the ground.
 *
 * Integration note
 * ----------------
 * The existing project header is intentionally preserved to minimize project
 * conflicts. The original W25Q64_* API remains source-compatible. The log API
 * at the end of this file can be forward-declared in main.c, as done by the
 * supplied main_with_memory.c. Moving those declarations into w25q64.h later is
 * recommended, but is not required for this two-file integration.
 */

#include "w25q64.h"
#include "rocket_protocol.h"

#include <stdbool.h>
#include <stddef.h>
#include <string.h>

/* -------------------------------------------------------------------------- */
/* W25Q64JV instruction and status definitions                                */
/* -------------------------------------------------------------------------- */

#define W25Q64_CMD_WRITE_ENABLE          0x06U
#define W25Q64_CMD_WRITE_DISABLE         0x04U
#define W25Q64_CMD_READ_STATUS1          0x05U
#define W25Q64_CMD_READ_DATA             0x03U
#define W25Q64_CMD_PAGE_PROGRAM          0x02U
#define W25Q64_CMD_SECTOR_ERASE          0x20U
#define W25Q64_CMD_JEDEC_ID              0x9FU
#define W25Q64_CMD_POWER_DOWN            0xB9U
#define W25Q64_CMD_RELEASE_POWER_DOWN    0xABU
#define W25Q64_CMD_ENABLE_RESET          0x66U
#define W25Q64_CMD_RESET_DEVICE          0x99U

#define W25Q64_STATUS1_BUSY              0x01U
#define W25Q64_STATUS1_WEL               0x02U

#define W25Q64_DEFAULT_SPI_TIMEOUT_MS     100U
#define W25Q64_DEFAULT_PROGRAM_TIMEOUT_MS 500U
#define W25Q64_DEFAULT_ERASE_TIMEOUT_MS   5000U
#define W25Q64_POWER_UP_DELAY_MS          5U
#define W25Q64_RESET_DELAY_MS             1U
#define W25Q64_WAKE_DELAY_MS              1U

#define W25Q64_EXPECTED_MANUFACTURER_ID   0xEFU
#define W25Q64_EXPECTED_CAPACITY_ID       0x17U

/* -------------------------------------------------------------------------- */
/* Flight-log format                                                          */
/* -------------------------------------------------------------------------- */

#define W25Q64_LOG_START_ADDRESS          W25Q64_SECTOR_SIZE_BYTES
#define W25Q64_LOG_SECTOR_COUNT \
    ((W25Q64_FLASH_SIZE_BYTES - W25Q64_LOG_START_ADDRESS) / \
     W25Q64_SECTOR_SIZE_BYTES)

#define W25Q64_LOG_RECORD_SIZE            48U
#define W25Q64_LOG_RECORDS_PER_SECTOR \
    (W25Q64_SECTOR_SIZE_BYTES / W25Q64_LOG_RECORD_SIZE)
#define W25Q64_LOG_CAPACITY_RECORDS \
    (W25Q64_LOG_SECTOR_COUNT * W25Q64_LOG_RECORDS_PER_SECTOR)

/* Little-endian bytes are 'R', 'L', 'O', 'G'. */
#define W25Q64_LOG_MAGIC                  0x474F4C52UL
#define W25Q64_LOG_FORMAT_VERSION         2U
#define W25Q64_LOG_TELEMETRY_SIZE         26U
#define W25Q64_LOG_EVENT_SIZE             10U

#define W25Q64_LOG_MAGIC_OFFSET           0U
#define W25Q64_LOG_FORMAT_OFFSET          4U
#define W25Q64_LOG_PROTOCOL_OFFSET        5U
#define W25Q64_LOG_PAYLOAD_LENGTH_OFFSET  6U
#define W25Q64_LOG_PACKET_TYPE_OFFSET     7U
#define W25Q64_LOG_SEQUENCE_OFFSET        8U
#define W25Q64_LOG_TIME_OFFSET            12U
#define W25Q64_LOG_PAYLOAD_OFFSET         16U
#define W25Q64_LOG_CRC_OFFSET             42U
#define W25Q64_LOG_COMMIT_OFFSET          44U
#define W25Q64_LOG_COMMIT_VALUE           0x00U
#define W25Q64_LOG_BODY_SIZE              44U

/**
 * Single-device log state.
 *
 * The low-level driver remains handle-based and can support more than one
 * device. The high-level flight log is intentionally single-instance because
 * this flight computer has one fitted external flash and preserving the
 * existing handle structure avoids a required header change.
 */
typedef struct
{
    bool opened;
    bool append_allowed;
    uint32_t record_count;
    uint32_t next_record_index;
    uint32_t next_sequence;
    uint32_t corrupt_records;
} W25Q64_LogState;

static W25Q64_LogState g_w25q64_log;

/* -------------------------------------------------------------------------- */
/* Small endian helpers                                                       */
/* -------------------------------------------------------------------------- */

static void W25Q64_WriteU16LE(uint8_t *destination, uint16_t value)
{
    destination[0] = (uint8_t)value;
    destination[1] = (uint8_t)(value >> 8);
}

static void W25Q64_WriteI16LE(uint8_t *destination, int16_t value)
{
    W25Q64_WriteU16LE(destination, (uint16_t)value);
}

static void W25Q64_WriteU32LE(uint8_t *destination, uint32_t value)
{
    destination[0] = (uint8_t)value;
    destination[1] = (uint8_t)(value >> 8);
    destination[2] = (uint8_t)(value >> 16);
    destination[3] = (uint8_t)(value >> 24);
}

static uint16_t W25Q64_ReadU16LE(const uint8_t *source)
{
    return (uint16_t)source[0] |
           ((uint16_t)source[1] << 8);
}

static int16_t W25Q64_ReadI16LE(const uint8_t *source)
{
    return (int16_t)W25Q64_ReadU16LE(source);
}

static uint32_t W25Q64_ReadU32LE(const uint8_t *source)
{
    return (uint32_t)source[0] |
           ((uint32_t)source[1] << 8) |
           ((uint32_t)source[2] << 16) |
           ((uint32_t)source[3] << 24);
}

/* -------------------------------------------------------------------------- */
/* Low-level internal helpers                                                 */
/* -------------------------------------------------------------------------- */

static uint32_t W25Q64_TimeoutOrDefault(uint32_t configured,
                                       uint32_t fallback)
{
    return (configured == 0U) ? fallback : configured;
}

static bool W25Q64_RangeIsValid(uint32_t address, uint32_t length)
{
    if (address > W25Q64_FLASH_SIZE_BYTES)
    {
        return false;
    }

    /* Written this way to avoid overflow in address + length. */
    return length <= (W25Q64_FLASH_SIZE_BYTES - address);
}

static void W25Q64_CS_Low(W25Q64_HandleTypeDef *dev)
{
    HAL_GPIO_WritePin(dev->cs_port, dev->cs_pin, GPIO_PIN_RESET);
}

static void W25Q64_CS_High(W25Q64_HandleTypeDef *dev)
{
    HAL_GPIO_WritePin(dev->cs_port, dev->cs_pin, GPIO_PIN_SET);
}

static W25Q64_Result_t W25Q64_CheckArgs(W25Q64_HandleTypeDef *dev)
{
    if ((dev == NULL) ||
        (dev->hspi == NULL) ||
        (dev->cs_port == NULL))
    {
        return W25Q64_INVALID_ARGUMENT;
    }

    return W25Q64_OK;
}

static W25Q64_Result_t W25Q64_FromHAL(HAL_StatusTypeDef status)
{
    if (status == HAL_OK)
    {
        return W25Q64_OK;
    }

    if (status == HAL_TIMEOUT)
    {
        return W25Q64_TIMEOUT;
    }

    return W25Q64_ERROR;
}

#define W25Q64_TRANSFER_CHUNK_SIZE 32U

static W25Q64_Result_t W25Q64_Tx(
    W25Q64_HandleTypeDef *dev,
    const uint8_t *data,
    uint16_t length)
{
    uint8_t discarded_rx[W25Q64_TRANSFER_CHUNK_SIZE];

    const uint32_t timeout =
        W25Q64_TimeoutOrDefault(
            dev->spi_timeout_ms,
            W25Q64_DEFAULT_SPI_TIMEOUT_MS);

    if ((data == NULL) && (length != 0U))
    {
        return W25Q64_INVALID_ARGUMENT;
    }

    while (length != 0U)
    {
        HAL_StatusTypeDef hal_status;

        const uint16_t chunk =
            (length > W25Q64_TRANSFER_CHUNK_SIZE)
                ? W25Q64_TRANSFER_CHUNK_SIZE
                : length;

        hal_status =
            HAL_SPI_TransmitReceive(
                dev->hspi,
                (uint8_t *)data,
                discarded_rx,
                chunk,
                timeout);

        if (hal_status != HAL_OK)
        {
            return W25Q64_FromHAL(hal_status);
        }

        data += chunk;
        length -= chunk;
    }

    return W25Q64_OK;
}

static W25Q64_Result_t W25Q64_Rx(
    W25Q64_HandleTypeDef *dev,
    uint8_t *data,
    uint16_t length)
{
    uint8_t dummy_tx[W25Q64_TRANSFER_CHUNK_SIZE];

    const uint32_t timeout =
        W25Q64_TimeoutOrDefault(
            dev->spi_timeout_ms,
            W25Q64_DEFAULT_SPI_TIMEOUT_MS);

    if ((data == NULL) && (length != 0U))
    {
        return W25Q64_INVALID_ARGUMENT;
    }

    memset(dummy_tx, 0xFF, sizeof(dummy_tx));

    while (length != 0U)
    {
        HAL_StatusTypeDef hal_status;

        const uint16_t chunk =
            (length > W25Q64_TRANSFER_CHUNK_SIZE)
                ? W25Q64_TRANSFER_CHUNK_SIZE
                : length;

        /*
         * SPI reads require the master to transmit dummy bytes so that
         * clock pulses are generated for the flash response.
         */
        hal_status =
            HAL_SPI_TransmitReceive(
                dev->hspi,
                dummy_tx,
                data,
                chunk,
                timeout);

        if (hal_status != HAL_OK)
        {
            return W25Q64_FromHAL(hal_status);
        }

        data += chunk;
        length -= chunk;
    }

    return W25Q64_OK;
}

static W25Q64_Result_t W25Q64_CommandOnly(W25Q64_HandleTypeDef *dev,
                                         uint8_t command)
{
    W25Q64_Result_t result = W25Q64_CheckArgs(dev);

    if (result != W25Q64_OK)
    {
        return result;
    }

    W25Q64_CS_Low(dev);
    result = W25Q64_Tx(dev, &command, 1U);
    W25Q64_CS_High(dev);

    return result;
}

/** Send Write Enable and verify that Status Register-1 reports WEL=1. */
static W25Q64_Result_t W25Q64_WriteEnableAndVerify(
    W25Q64_HandleTypeDef *dev)
{
    uint8_t status1 = 0U;
    W25Q64_Result_t result =
        W25Q64_CommandOnly(dev, W25Q64_CMD_WRITE_ENABLE);

    if (result != W25Q64_OK)
    {
        return result;
    }

    result = W25Q64_ReadStatus1(dev, &status1);
    if (result != W25Q64_OK)
    {
        return result;
    }

    return ((status1 & W25Q64_STATUS1_WEL) != 0U)
        ? W25Q64_OK
        : W25Q64_ERROR;
}

/* -------------------------------------------------------------------------- */
/* Existing public low-level API                                              */
/* -------------------------------------------------------------------------- */

void W25Q64_Attach(W25Q64_HandleTypeDef *dev,
                   SPI_HandleTypeDef *hspi,
                   GPIO_TypeDef *cs_port,
                   uint16_t cs_pin)
{
    if (dev == NULL)
    {
        return;
    }

    dev->hspi = hspi;
    dev->cs_port = cs_port;
    dev->cs_pin = cs_pin;
    dev->spi_timeout_ms = W25Q64_DEFAULT_SPI_TIMEOUT_MS;
    dev->program_timeout_ms = W25Q64_DEFAULT_PROGRAM_TIMEOUT_MS;
    dev->erase_timeout_ms = W25Q64_DEFAULT_ERASE_TIMEOUT_MS;
}

W25Q64_Result_t W25Q64_ReadStatus1(W25Q64_HandleTypeDef *dev,
                                   uint8_t *status1)
{
    const uint8_t command = W25Q64_CMD_READ_STATUS1;
    W25Q64_Result_t result;

    if (status1 == NULL)
    {
        return W25Q64_INVALID_ARGUMENT;
    }

    result = W25Q64_CheckArgs(dev);
    if (result != W25Q64_OK)
    {
        return result;
    }

    W25Q64_CS_Low(dev);

    result = W25Q64_Tx(dev, &command, 1U);
    if (result == W25Q64_OK)
    {
        result = W25Q64_Rx(dev, status1, 1U);
    }

    W25Q64_CS_High(dev);
    return result;
}

W25Q64_Result_t W25Q64_WaitReady(W25Q64_HandleTypeDef *dev,
                                 uint32_t timeout_ms)
{
    uint8_t status1 = 0U;
    uint32_t start_ms;
    W25Q64_Result_t result = W25Q64_CheckArgs(dev);

    if (result != W25Q64_OK)
    {
        return result;
    }

    start_ms = HAL_GetTick();

    do
    {
        result = W25Q64_ReadStatus1(dev, &status1);
        if (result != W25Q64_OK)
        {
            return result;
        }

        if ((status1 & W25Q64_STATUS1_BUSY) == 0U)
        {
            return W25Q64_OK;
        }

        /* Prevent a long erase from becoming a CPU-burning tight loop. */
        HAL_Delay(1U);
    }
    while ((uint32_t)(HAL_GetTick() - start_ms) < timeout_ms);

    return W25Q64_TIMEOUT;
}

W25Q64_Result_t W25Q64_ReadJEDECID(W25Q64_HandleTypeDef *dev,
                                   uint8_t id[3])
{
    const uint8_t command = W25Q64_CMD_JEDEC_ID;
    W25Q64_Result_t result;

    if (id == NULL)
    {
        return W25Q64_INVALID_ARGUMENT;
    }

    result = W25Q64_CheckArgs(dev);
    if (result != W25Q64_OK)
    {
        return result;
    }

    W25Q64_CS_Low(dev);
    result = W25Q64_Tx(dev, &command, 1U);
    if (result == W25Q64_OK)
    {
        result = W25Q64_Rx(dev, id, 3U);
    }
    W25Q64_CS_High(dev);

    return result;
}

W25Q64_Result_t W25Q64_Init(W25Q64_HandleTypeDef *dev)
{
    uint8_t id[3] = {0U, 0U, 0U};
    W25Q64_Result_t result = W25Q64_CheckArgs(dev);

    if (result != W25Q64_OK)
    {
        return result;
    }

    /* /CS must be high during power-up before the first instruction. */
    W25Q64_CS_High(dev);
    HAL_Delay(W25Q64_POWER_UP_DELAY_MS);

    /* Wake a device that may have been left in software power-down. */
    result = W25Q64_CommandOnly(dev, W25Q64_CMD_RELEASE_POWER_DOWN);
    if (result != W25Q64_OK)
    {
        return result;
    }
    HAL_Delay(W25Q64_WAKE_DELAY_MS);

    /*
     * The datasheet warns that resetting during an active program/erase may
     * corrupt data. This also covers the case where the MCU reset while the
     * flash was finishing an operation from the previous firmware session.
     */
    result = W25Q64_WaitReady(
        dev,
        W25Q64_TimeoutOrDefault(dev->erase_timeout_ms,
                               W25Q64_DEFAULT_ERASE_TIMEOUT_MS));
    if (result != W25Q64_OK)
    {
        return result;
    }

    /* Return all volatile interface state to a known standard-SPI condition. */
    result = W25Q64_CommandOnly(dev, W25Q64_CMD_ENABLE_RESET);
    if (result != W25Q64_OK)
    {
        return result;
    }

    result = W25Q64_CommandOnly(dev, W25Q64_CMD_RESET_DEVICE);
    if (result != W25Q64_OK)
    {
        return result;
    }
    HAL_Delay(W25Q64_RESET_DELAY_MS);

    result = W25Q64_WaitReady(dev, W25Q64_DEFAULT_PROGRAM_TIMEOUT_MS);
    if (result != W25Q64_OK)
    {
        return result;
    }

    result = W25Q64_ReadJEDECID(dev, id);
    if (result != W25Q64_OK)
    {
        return result;
    }

    /*
     * Winbond manufacturer is EFh and 64-Mbit density is 17h. The middle
     * JEDEC byte varies among valid W25Q64JV package/family variants, so it is
     * deliberately not over-constrained.
     */
    if ((id[0] != W25Q64_EXPECTED_MANUFACTURER_ID) ||
        (id[2] != W25Q64_EXPECTED_CAPACITY_ID))
    {
        return W25Q64_BAD_ID;
    }

    return W25Q64_OK;
}

W25Q64_Result_t W25Q64_Read(W25Q64_HandleTypeDef *dev,
                            uint32_t address,
                            uint8_t *data,
                            uint32_t length)
{
    uint8_t command[4];
    W25Q64_Result_t result;

    if ((data == NULL) && (length != 0U))
    {
        return W25Q64_INVALID_ARGUMENT;
    }

    result = W25Q64_CheckArgs(dev);
    if (result != W25Q64_OK)
    {
        return result;
    }

    if (!W25Q64_RangeIsValid(address, length))
    {
        return W25Q64_OUT_OF_RANGE;
    }

    if (length == 0U)
    {
        return W25Q64_OK;
    }

    command[0] = W25Q64_CMD_READ_DATA;
    command[1] = (uint8_t)(address >> 16);
    command[2] = (uint8_t)(address >> 8);
    command[3] = (uint8_t)address;

    W25Q64_CS_Low(dev);
    result = W25Q64_Tx(dev, command, sizeof(command));

    while ((result == W25Q64_OK) && (length != 0U))
    {
        const uint16_t chunk =
            (length > 65535U) ? 65535U : (uint16_t)length;

        result = W25Q64_Rx(dev, data, chunk);
        data += chunk;
        length -= chunk;
    }

    W25Q64_CS_High(dev);
    return result;
}

W25Q64_Result_t W25Q64_Write(W25Q64_HandleTypeDef *dev,
                             uint32_t address,
                             const uint8_t *data,
                             uint32_t length)
{
    W25Q64_Result_t result;

    if ((data == NULL) && (length != 0U))
    {
        return W25Q64_INVALID_ARGUMENT;
    }

    result = W25Q64_CheckArgs(dev);
    if (result != W25Q64_OK)
    {
        return result;
    }

    if (!W25Q64_RangeIsValid(address, length))
    {
        return W25Q64_OUT_OF_RANGE;
    }

    while (length != 0U)
    {
        uint8_t command[4];
        const uint32_t page_offset =
            address % W25Q64_PAGE_SIZE_BYTES;
        const uint32_t room_in_page =
            W25Q64_PAGE_SIZE_BYTES - page_offset;
        const uint16_t chunk =
            (length < room_in_page)
                ? (uint16_t)length
                : (uint16_t)room_in_page;
        const uint32_t timeout =
            W25Q64_TimeoutOrDefault(dev->program_timeout_ms,
                                   W25Q64_DEFAULT_PROGRAM_TIMEOUT_MS);

        result = W25Q64_WriteEnableAndVerify(dev);
        if (result != W25Q64_OK)
        {
            return result;
        }

        command[0] = W25Q64_CMD_PAGE_PROGRAM;
        command[1] = (uint8_t)(address >> 16);
        command[2] = (uint8_t)(address >> 8);
        command[3] = (uint8_t)address;

        W25Q64_CS_Low(dev);
        result = W25Q64_Tx(dev, command, sizeof(command));
        if (result == W25Q64_OK)
        {
            result = W25Q64_Tx(dev, data, chunk);
        }
        W25Q64_CS_High(dev);

        if (result != W25Q64_OK)
        {
            return result;
        }

        result = W25Q64_WaitReady(dev, timeout);
        if (result != W25Q64_OK)
        {
            return result;
        }

        address += chunk;
        data += chunk;
        length -= chunk;
    }

    return W25Q64_OK;
}

W25Q64_Result_t W25Q64_EraseSector(W25Q64_HandleTypeDef *dev,
                                   uint32_t sector_address)
{
    uint8_t command[4];
    uint32_t aligned_address;
    uint32_t timeout;
    W25Q64_Result_t result = W25Q64_CheckArgs(dev);

    if (result != W25Q64_OK)
    {
        return result;
    }

    if (sector_address >= W25Q64_FLASH_SIZE_BYTES)
    {
        return W25Q64_OUT_OF_RANGE;
    }

    aligned_address =
        sector_address & ~(W25Q64_SECTOR_SIZE_BYTES - 1UL);

    result = W25Q64_WriteEnableAndVerify(dev);
    if (result != W25Q64_OK)
    {
        return result;
    }

    command[0] = W25Q64_CMD_SECTOR_ERASE;
    command[1] = (uint8_t)(aligned_address >> 16);
    command[2] = (uint8_t)(aligned_address >> 8);
    command[3] = (uint8_t)aligned_address;

    W25Q64_CS_Low(dev);
    result = W25Q64_Tx(dev, command, sizeof(command));
    W25Q64_CS_High(dev);

    if (result != W25Q64_OK)
    {
        return result;
    }

    timeout = W25Q64_TimeoutOrDefault(dev->erase_timeout_ms,
                                     W25Q64_DEFAULT_ERASE_TIMEOUT_MS);
    return W25Q64_WaitReady(dev, timeout);
}

W25Q64_Result_t W25Q64_EraseRange(W25Q64_HandleTypeDef *dev,
                                  uint32_t start_address,
                                  uint32_t length_bytes)
{
    uint32_t address;
    uint32_t end_address;
    W25Q64_Result_t result = W25Q64_CheckArgs(dev);

    if (result != W25Q64_OK)
    {
        return result;
    }

    if (length_bytes == 0U)
    {
        return W25Q64_OK;
    }

    if (!W25Q64_RangeIsValid(start_address, length_bytes) ||
        (start_address == W25Q64_FLASH_SIZE_BYTES))
    {
        return W25Q64_OUT_OF_RANGE;
    }

    address = start_address & ~(W25Q64_SECTOR_SIZE_BYTES - 1UL);
    end_address = start_address + length_bytes;

    while (address < end_address)
    {
        result = W25Q64_EraseSector(dev, address);
        if (result != W25Q64_OK)
        {
            return result;
        }

        address += W25Q64_SECTOR_SIZE_BYTES;
    }

    return W25Q64_OK;
}

W25Q64_Result_t W25Q64_PowerDown(W25Q64_HandleTypeDef *dev)
{
    W25Q64_Result_t result =
        W25Q64_CommandOnly(dev, W25Q64_CMD_POWER_DOWN);

    if (result == W25Q64_OK)
    {
        HAL_Delay(W25Q64_WAKE_DELAY_MS);
    }

    return result;
}

W25Q64_Result_t W25Q64_ReleasePowerDown(W25Q64_HandleTypeDef *dev)
{
    W25Q64_Result_t result =
        W25Q64_CommandOnly(dev, W25Q64_CMD_RELEASE_POWER_DOWN);

    if (result == W25Q64_OK)
    {
        HAL_Delay(W25Q64_WAKE_DELAY_MS);
    }

    return result;
}

/* -------------------------------------------------------------------------- */
/* Rocket telemetry/event serialization                                       */
/* -------------------------------------------------------------------------- */

static void W25Q64_LogEncodeTelemetry(
    uint8_t output[W25Q64_LOG_TELEMETRY_SIZE],
    const RocketTelemetryPayload *payload)
{
    W25Q64_WriteU16LE(output + 0U, payload->flags);
    output[2U] = payload->state;
    output[3U] = payload->status_code;
    W25Q64_WriteI16LE(output + 4U, payload->altitude_dm);
    W25Q64_WriteI16LE(output + 6U, payload->velocity_cms);
    W25Q64_WriteI16LE(output + 8U, payload->acceleration_cms2);
    W25Q64_WriteU16LE(output + 10U, payload->predicted_apogee_dm);
    W25Q64_WriteU16LE(output + 12U, payload->target_apogee_dm);
    W25Q64_WriteI16LE(output + 14U, payload->roll_ddeg);
    W25Q64_WriteI16LE(output + 16U, payload->pitch_ddeg);
    W25Q64_WriteI16LE(output + 18U, payload->yaw_ddeg);
    output[20U] = payload->deployment_percent;
    output[21U] = payload->sensor_health;
    W25Q64_WriteU16LE(output + 22U, payload->failed_reads);
    output[24U] = payload->message_code;
    output[25U] = payload->reserved;
}

static void W25Q64_LogDecodeTelemetry(
    const uint8_t input[W25Q64_LOG_TELEMETRY_SIZE],
    RocketTelemetryPayload *payload)
{
    payload->flags = W25Q64_ReadU16LE(input + 0U);
    payload->state = input[2U];
    payload->status_code = input[3U];
    payload->altitude_dm = W25Q64_ReadI16LE(input + 4U);
    payload->velocity_cms = W25Q64_ReadI16LE(input + 6U);
    payload->acceleration_cms2 = W25Q64_ReadI16LE(input + 8U);
    payload->predicted_apogee_dm = W25Q64_ReadU16LE(input + 10U);
    payload->target_apogee_dm = W25Q64_ReadU16LE(input + 12U);
    payload->roll_ddeg = W25Q64_ReadI16LE(input + 14U);
    payload->pitch_ddeg = W25Q64_ReadI16LE(input + 16U);
    payload->yaw_ddeg = W25Q64_ReadI16LE(input + 18U);
    payload->deployment_percent = input[20U];
    payload->sensor_health = input[21U];
    payload->failed_reads = W25Q64_ReadU16LE(input + 22U);
    payload->message_code = input[24U];
    payload->reserved = input[25U];
}

static void W25Q64_LogEncodeEvent(
    uint8_t output[W25Q64_LOG_EVENT_SIZE],
    const RocketEventPayload *payload)
{
    W25Q64_WriteU16LE(output + 0U, payload->changed_flags);
    W25Q64_WriteU16LE(output + 2U, payload->current_flags);
    output[4U] = payload->previous_state;
    output[5U] = payload->current_state;
    output[6U] = payload->status_code;
    output[7U] = payload->message_code;
    W25Q64_WriteU16LE(output + 8U, payload->detail);
}

static void W25Q64_LogDecodeEvent(
    const uint8_t input[W25Q64_LOG_EVENT_SIZE],
    RocketEventPayload *payload)
{
    payload->changed_flags = W25Q64_ReadU16LE(input + 0U);
    payload->current_flags = W25Q64_ReadU16LE(input + 2U);
    payload->previous_state = input[4U];
    payload->current_state = input[5U];
    payload->status_code = input[6U];
    payload->message_code = input[7U];
    payload->detail = W25Q64_ReadU16LE(input + 8U);
}

/* -------------------------------------------------------------------------- */
/* Rocket log internal helpers                                                */
/* -------------------------------------------------------------------------- */

static uint32_t W25Q64_LogAddressForIndex(uint32_t record_index)
{
    const uint32_t sector_index =
        record_index / W25Q64_LOG_RECORDS_PER_SECTOR;
    const uint32_t slot_index =
        record_index % W25Q64_LOG_RECORDS_PER_SECTOR;

    return W25Q64_LOG_START_ADDRESS +
           sector_index * W25Q64_SECTOR_SIZE_BYTES +
           slot_index * W25Q64_LOG_RECORD_SIZE;
}

static bool W25Q64_LogBufferIsErased(const uint8_t *data, size_t length)
{
    size_t index;

    for (index = 0U; index < length; ++index)
    {
        if (data[index] != 0xFFU)
        {
            return false;
        }
    }

    return true;
}

static uint8_t W25Q64_LogPacketType(
    const uint8_t raw[W25Q64_LOG_RECORD_SIZE])
{
    /* Version 1 records were telemetry-only and left byte 7 as zero. */
    if (raw[W25Q64_LOG_FORMAT_OFFSET] == 1U)
    {
        return ROCKET_PKT_TELEMETRY;
    }

    return raw[W25Q64_LOG_PACKET_TYPE_OFFSET];
}

static bool W25Q64_LogMetadataIsValid(
    const uint8_t raw[W25Q64_LOG_RECORD_SIZE])
{
    const uint8_t format = raw[W25Q64_LOG_FORMAT_OFFSET];
    const uint8_t packet_type = W25Q64_LogPacketType(raw);
    const uint8_t payload_length =
        raw[W25Q64_LOG_PAYLOAD_LENGTH_OFFSET];

    if (format == 1U)
    {
        return payload_length == W25Q64_LOG_TELEMETRY_SIZE;
    }

    if (format != W25Q64_LOG_FORMAT_VERSION)
    {
        return false;
    }

    if (packet_type == ROCKET_PKT_TELEMETRY)
    {
        return payload_length == W25Q64_LOG_TELEMETRY_SIZE;
    }

    if (packet_type == ROCKET_PKT_EVENT)
    {
        return payload_length == W25Q64_LOG_EVENT_SIZE;
    }

    return false;
}

static bool W25Q64_LogRawRecordIsValid(
    const uint8_t raw[W25Q64_LOG_RECORD_SIZE])
{
    const uint16_t stored_crc =
        W25Q64_ReadU16LE(raw + W25Q64_LOG_CRC_OFFSET);
    const uint16_t calculated_crc =
        RocketProtocol_Crc16(raw, W25Q64_LOG_CRC_OFFSET);

    return
        (raw[W25Q64_LOG_COMMIT_OFFSET] == W25Q64_LOG_COMMIT_VALUE) &&
        (W25Q64_ReadU32LE(raw + W25Q64_LOG_MAGIC_OFFSET) ==
         W25Q64_LOG_MAGIC) &&
        (raw[W25Q64_LOG_PROTOCOL_OFFSET] == ROCKET_PROTOCOL_VERSION) &&
        W25Q64_LogMetadataIsValid(raw) &&
        (stored_crc == calculated_crc);
}

static W25Q64_Result_t W25Q64_LogReadRaw(
    W25Q64_HandleTypeDef *dev,
    uint32_t record_index,
    uint8_t raw[W25Q64_LOG_RECORD_SIZE])
{
    if ((raw == NULL) ||
        (record_index >= W25Q64_LOG_CAPACITY_RECORDS))
    {
        return W25Q64_OUT_OF_RANGE;
    }

    return W25Q64_Read(dev,
                       W25Q64_LogAddressForIndex(record_index),
                       raw,
                       W25Q64_LOG_RECORD_SIZE);
}

/**
 * Scan one sector to find its first erased record or the first damaged record.
 * Earlier sectors are assumed full because append order is strictly sequential.
 */
static W25Q64_Result_t W25Q64_LogScanSector(
    W25Q64_HandleTypeDef *dev,
    uint32_t sector_index)
{
    uint32_t slot;
    const uint32_t base_record =
        sector_index * W25Q64_LOG_RECORDS_PER_SECTOR;

    for (slot = 0U; slot < W25Q64_LOG_RECORDS_PER_SECTOR; ++slot)
    {
        uint8_t raw[W25Q64_LOG_RECORD_SIZE];
        const uint32_t record_index = base_record + slot;
        W25Q64_Result_t result =
            W25Q64_LogReadRaw(dev, record_index, raw);

        if (result != W25Q64_OK)
        {
            g_w25q64_log.append_allowed = false;
            return result;
        }

        if (W25Q64_LogBufferIsErased(raw, sizeof(raw)))
        {
            g_w25q64_log.record_count = record_index;
            g_w25q64_log.next_record_index = record_index;
            g_w25q64_log.next_sequence = record_index;
            g_w25q64_log.append_allowed = true;
            return W25Q64_OK;
        }

        if (!W25Q64_LogRawRecordIsValid(raw))
        {
            /* Never reuse a partially programmed NOR slot. */
            g_w25q64_log.record_count = record_index;
            g_w25q64_log.next_record_index = record_index;
            g_w25q64_log.next_sequence = record_index;
            g_w25q64_log.append_allowed = false;
            ++g_w25q64_log.corrupt_records;
            return W25Q64_ERROR;
        }
    }

    g_w25q64_log.record_count =
        base_record + W25Q64_LOG_RECORDS_PER_SECTOR;
    g_w25q64_log.next_record_index = g_w25q64_log.record_count;
    g_w25q64_log.next_sequence = g_w25q64_log.record_count;
    g_w25q64_log.append_allowed =
        g_w25q64_log.next_record_index < W25Q64_LOG_CAPACITY_RECORDS;

    return W25Q64_OK;
}

/* -------------------------------------------------------------------------- */
/* Public Rocket log API                                                      */
/* -------------------------------------------------------------------------- */

W25Q64_Result_t W25Q64_LogOpen(W25Q64_HandleTypeDef *dev)
{
    uint32_t sector;
    W25Q64_Result_t result = W25Q64_CheckArgs(dev);

    memset(&g_w25q64_log, 0, sizeof(g_w25q64_log));

    if (result != W25Q64_OK)
    {
        return result;
    }

    g_w25q64_log.opened = true;
    g_w25q64_log.append_allowed = true;

    for (sector = 0U; sector < W25Q64_LOG_SECTOR_COUNT; ++sector)
    {
        uint8_t first[W25Q64_LOG_RECORD_SIZE];
        uint8_t last[W25Q64_LOG_RECORD_SIZE];
        const uint32_t first_index =
            sector * W25Q64_LOG_RECORDS_PER_SECTOR;
        const uint32_t last_index =
            first_index + W25Q64_LOG_RECORDS_PER_SECTOR - 1U;

        result = W25Q64_LogReadRaw(dev, first_index, first);
        if (result != W25Q64_OK)
        {
            g_w25q64_log.append_allowed = false;
            return result;
        }

        if (W25Q64_LogBufferIsErased(first, sizeof(first)))
        {
            g_w25q64_log.record_count = first_index;
            g_w25q64_log.next_record_index = first_index;
            g_w25q64_log.next_sequence = first_index;
            return W25Q64_OK;
        }

        if (!W25Q64_LogRawRecordIsValid(first))
        {
            g_w25q64_log.record_count = first_index;
            g_w25q64_log.next_record_index = first_index;
            g_w25q64_log.next_sequence = first_index;
            g_w25q64_log.append_allowed = false;
            ++g_w25q64_log.corrupt_records;
            return W25Q64_ERROR;
        }

        result = W25Q64_LogReadRaw(dev, last_index, last);
        if (result != W25Q64_OK)
        {
            g_w25q64_log.append_allowed = false;
            return result;
        }

        if (!W25Q64_LogRawRecordIsValid(last))
        {
            return W25Q64_LogScanSector(dev, sector);
        }
    }

    g_w25q64_log.record_count = W25Q64_LOG_CAPACITY_RECORDS;
    g_w25q64_log.next_record_index = W25Q64_LOG_CAPACITY_RECORDS;
    g_w25q64_log.next_sequence = W25Q64_LOG_CAPACITY_RECORDS;
    g_w25q64_log.append_allowed = false;
    return W25Q64_OK;
}

static W25Q64_Result_t W25Q64_LogAppendEncoded(
    W25Q64_HandleTypeDef *dev,
    uint32_t time_ms,
    uint8_t packet_type,
    const uint8_t *payload,
    uint8_t payload_length)
{
    uint8_t raw[W25Q64_LOG_RECORD_SIZE];
    uint8_t existing[W25Q64_LOG_RECORD_SIZE];
    uint8_t verify[W25Q64_LOG_BODY_SIZE];
    const uint8_t commit = W25Q64_LOG_COMMIT_VALUE;
    uint32_t address;
    W25Q64_Result_t result;

    if ((payload == NULL) || !g_w25q64_log.opened)
    {
        return W25Q64_INVALID_ARGUMENT;
    }

    if (((packet_type != ROCKET_PKT_TELEMETRY) ||
         (payload_length != W25Q64_LOG_TELEMETRY_SIZE)) &&
        ((packet_type != ROCKET_PKT_EVENT) ||
         (payload_length != W25Q64_LOG_EVENT_SIZE)))
    {
        return W25Q64_INVALID_ARGUMENT;
    }

    if (g_w25q64_log.next_record_index >= W25Q64_LOG_CAPACITY_RECORDS)
    {
        g_w25q64_log.append_allowed = false;
        return W25Q64_OUT_OF_RANGE;
    }

    if (!g_w25q64_log.append_allowed)
    {
        return W25Q64_ERROR;
    }

    address = W25Q64_LogAddressForIndex(
        g_w25q64_log.next_record_index);

    result = W25Q64_Read(dev, address, existing, sizeof(existing));
    if (result != W25Q64_OK)
    {
        return result;
    }

    if (!W25Q64_LogBufferIsErased(existing, sizeof(existing)))
    {
        g_w25q64_log.append_allowed = false;
        ++g_w25q64_log.corrupt_records;
        return W25Q64_ERROR;
    }

    memset(raw, 0xFF, sizeof(raw));
    W25Q64_WriteU32LE(raw + W25Q64_LOG_MAGIC_OFFSET,
                      W25Q64_LOG_MAGIC);
    raw[W25Q64_LOG_FORMAT_OFFSET] = W25Q64_LOG_FORMAT_VERSION;
    raw[W25Q64_LOG_PROTOCOL_OFFSET] = ROCKET_PROTOCOL_VERSION;
    raw[W25Q64_LOG_PAYLOAD_LENGTH_OFFSET] = payload_length;
    raw[W25Q64_LOG_PACKET_TYPE_OFFSET] = packet_type;
    W25Q64_WriteU32LE(raw + W25Q64_LOG_SEQUENCE_OFFSET,
                      g_w25q64_log.next_sequence);
    W25Q64_WriteU32LE(raw + W25Q64_LOG_TIME_OFFSET, time_ms);
    memcpy(raw + W25Q64_LOG_PAYLOAD_OFFSET,
           payload,
           payload_length);
    W25Q64_WriteU16LE(raw + W25Q64_LOG_CRC_OFFSET,
                      RocketProtocol_Crc16(raw,
                                           W25Q64_LOG_CRC_OFFSET));

    result = W25Q64_Write(dev, address, raw, W25Q64_LOG_BODY_SIZE);
    if (result != W25Q64_OK)
    {
        g_w25q64_log.append_allowed = false;
        return result;
    }

    result = W25Q64_Read(dev, address, verify, sizeof(verify));
    if (result != W25Q64_OK)
    {
        g_w25q64_log.append_allowed = false;
        return result;
    }

    if (memcmp(raw, verify, sizeof(verify)) != 0)
    {
        g_w25q64_log.append_allowed = false;
        return W25Q64_ERROR;
    }

    result = W25Q64_Write(dev,
                          address + W25Q64_LOG_COMMIT_OFFSET,
                          &commit,
                          1U);
    if (result != W25Q64_OK)
    {
        g_w25q64_log.append_allowed = false;
        return result;
    }

    result = W25Q64_Read(dev, address, existing, sizeof(existing));
    if ((result != W25Q64_OK) ||
        !W25Q64_LogRawRecordIsValid(existing))
    {
        g_w25q64_log.append_allowed = false;
        return (result == W25Q64_OK) ? W25Q64_ERROR : result;
    }

    ++g_w25q64_log.record_count;
    ++g_w25q64_log.next_record_index;
    ++g_w25q64_log.next_sequence;

    if (g_w25q64_log.next_record_index >= W25Q64_LOG_CAPACITY_RECORDS)
    {
        g_w25q64_log.append_allowed = false;
    }

    return W25Q64_OK;
}

W25Q64_Result_t W25Q64_LogAppend(
    W25Q64_HandleTypeDef *dev,
    uint32_t time_ms,
    const RocketTelemetryPayload *payload)
{
    uint8_t encoded[W25Q64_LOG_TELEMETRY_SIZE];

    if (payload == NULL)
    {
        return W25Q64_INVALID_ARGUMENT;
    }

    W25Q64_LogEncodeTelemetry(encoded, payload);
    return W25Q64_LogAppendEncoded(dev,
                                   time_ms,
                                   ROCKET_PKT_TELEMETRY,
                                   encoded,
                                   sizeof(encoded));
}

W25Q64_Result_t W25Q64_LogAppendEvent(
    W25Q64_HandleTypeDef *dev,
    uint32_t time_ms,
    const RocketEventPayload *payload)
{
    uint8_t encoded[W25Q64_LOG_EVENT_SIZE];

    if (payload == NULL)
    {
        return W25Q64_INVALID_ARGUMENT;
    }

    W25Q64_LogEncodeEvent(encoded, payload);
    return W25Q64_LogAppendEncoded(dev,
                                   time_ms,
                                   ROCKET_PKT_EVENT,
                                   encoded,
                                   sizeof(encoded));
}

W25Q64_Result_t W25Q64_LogReadRecord(
    W25Q64_HandleTypeDef *dev,
    uint32_t record_index,
    uint32_t *time_ms,
    uint8_t *packet_type,
    RocketTelemetryPayload *telemetry,
    RocketEventPayload *event)
{
    uint8_t raw[W25Q64_LOG_RECORD_SIZE];
    uint8_t type;
    W25Q64_Result_t result;

    if ((time_ms == NULL) ||
        (packet_type == NULL) ||
        (telemetry == NULL) ||
        (event == NULL) ||
        !g_w25q64_log.opened)
    {
        return W25Q64_INVALID_ARGUMENT;
    }

    if (record_index >= g_w25q64_log.record_count)
    {
        return W25Q64_OUT_OF_RANGE;
    }

    result = W25Q64_LogReadRaw(dev, record_index, raw);
    if (result != W25Q64_OK)
    {
        return result;
    }

    if (!W25Q64_LogRawRecordIsValid(raw))
    {
        ++g_w25q64_log.corrupt_records;
        return W25Q64_ERROR;
    }

    type = W25Q64_LogPacketType(raw);
    memset(telemetry, 0, sizeof(*telemetry));
    memset(event, 0, sizeof(*event));

    *time_ms = W25Q64_ReadU32LE(raw + W25Q64_LOG_TIME_OFFSET);
    *packet_type = type;

    if (type == ROCKET_PKT_TELEMETRY)
    {
        W25Q64_LogDecodeTelemetry(raw + W25Q64_LOG_PAYLOAD_OFFSET,
                                  telemetry);
        return W25Q64_OK;
    }

    if (type == ROCKET_PKT_EVENT)
    {
        W25Q64_LogDecodeEvent(raw + W25Q64_LOG_PAYLOAD_OFFSET,
                              event);
        return W25Q64_OK;
    }

    return W25Q64_ERROR;
}

/* Compatibility helper for telemetry-only callers. */
W25Q64_Result_t W25Q64_LogRead(
    W25Q64_HandleTypeDef *dev,
    uint32_t record_index,
    uint32_t *time_ms,
    RocketTelemetryPayload *payload)
{
    RocketEventPayload ignored_event;
    uint8_t packet_type;
    W25Q64_Result_t result =
        W25Q64_LogReadRecord(dev,
                             record_index,
                             time_ms,
                             &packet_type,
                             payload,
                             &ignored_event);

    if (result != W25Q64_OK)
    {
        return result;
    }

    return (packet_type == ROCKET_PKT_TELEMETRY)
        ? W25Q64_OK
        : W25Q64_ERROR;
}


/**
 * @brief Erase the complete flight-log area and reopen it empty.
 *
 * This is intentionally blocking and may take a long time because 2047 sectors
 * are erased. Call only on the ground while measurement/control scheduling is
 * suspended. Sector 0 remains untouched for future configuration metadata.
 */
W25Q64_Result_t W25Q64_LogErase(W25Q64_HandleTypeDef *dev)
{
    uint32_t address;
    W25Q64_Result_t result;

    for (address = W25Q64_LOG_START_ADDRESS;
         address < W25Q64_FLASH_SIZE_BYTES;
         address += W25Q64_SECTOR_SIZE_BYTES)
    {
        result = W25Q64_EraseSector(dev, address);
        if (result != W25Q64_OK)
        {
            g_w25q64_log.opened = false;
            g_w25q64_log.append_allowed = false;
            return result;
        }
    }

    return W25Q64_LogOpen(dev);
}

uint32_t W25Q64_LogGetCount(void)
{
    return g_w25q64_log.record_count;
}

uint32_t W25Q64_LogGetCapacity(void)
{
    return W25Q64_LOG_CAPACITY_RECORDS;
}

uint32_t W25Q64_LogGetCorruptCount(void)
{
    return g_w25q64_log.corrupt_records;
}

bool W25Q64_LogCanAppend(void)
{
    return g_w25q64_log.opened && g_w25q64_log.append_allowed;
}
