/**
 * @file flight_log.c
 * @brief CSV append wrapper for payload data stored in W25Q64JV flash.
 *
 * Drop into Core/Src.
 */

#include "flight_log.h"
#include <stdio.h>
#include <string.h>

static const char FLIGHT_LOG_HEADER[] =
    "time_ms,"
    "imu_ax,imu_ay,imu_az,imu_gx,imu_gy,imu_gz,"
    "baro0,baro1,"
    "mag_x,mag_y,mag_z,"
    "vertical_v,current_apogee,projected_apogee,deployment_cmd,"
    "deploy_state,error_state,message\n";

static FlightLog_Result_t FlightLog_FromFlashResult(W25Q64_Result_t result)
{
    if (result == W25Q64_OK)
    {
        return FLIGHT_LOG_OK;
    }

    if (result == W25Q64_OUT_OF_RANGE)
    {
        return FLIGHT_LOG_FULL;
    }

    return FLIGHT_LOG_ERROR;
}

static void FlightLog_SanitizeMessage(char *dst, size_t dst_size, const char *src)
{
    size_t i = 0U;

    if ((dst == NULL) || (dst_size == 0U))
    {
        return;
    }

    if (src == NULL)
    {
        dst[0] = '\0';
        return;
    }

    /*
     * Keep CSV simple by replacing commas, quotes, CR, and LF.
     * This avoids needing quoted CSV fields on the MCU.
     */
    while ((src[i] != '\0') && (i < (dst_size - 1U)))
    {
        char c = src[i];

        if ((c == ',') || (c == '"') || (c == '\r') || (c == '\n'))
        {
            dst[i] = ' ';
        }
        else
        {
            dst[i] = c;
        }

        i++;
    }

    dst[i] = '\0';
}

static FlightLog_Result_t FlightLog_AppendBytes(FlightLog_HandleTypeDef *log,
                                                const uint8_t *data,
                                                uint32_t len)
{
    W25Q64_Result_t flash_result;

    if ((log == NULL) || ((data == NULL) && (len > 0U)))
    {
        return FLIGHT_LOG_INVALID_ARGUMENT;
    }

    if (!log->active)
    {
        return FLIGHT_LOG_INACTIVE;
    }

    if ((log->write_addr + len) > log->end_addr)
    {
        log->active = false;
        return FLIGHT_LOG_FULL;
    }

    if (len == 0U)
    {
        return FLIGHT_LOG_OK;
    }

    flash_result = W25Q64_Write(log->flash, log->write_addr, data, len);
    if (flash_result != W25Q64_OK)
    {
        log->active = false;
        return FlightLog_FromFlashResult(flash_result);
    }

    log->write_addr += len;
    return FLIGHT_LOG_OK;
}

FlightLog_Result_t FlightLog_Begin(FlightLog_HandleTypeDef *log,
                                   W25Q64_HandleTypeDef *flash,
                                   uint32_t start_addr,
                                   uint32_t max_size_bytes,
                                   bool erase_before_start)
{
    W25Q64_Result_t flash_result;

    if ((log == NULL) || (flash == NULL) || (max_size_bytes == 0U))
    {
        return FLIGHT_LOG_INVALID_ARGUMENT;
    }

    if ((start_addr >= W25Q64_FLASH_SIZE_BYTES) ||
        (max_size_bytes > W25Q64_FLASH_SIZE_BYTES) ||
        ((start_addr + max_size_bytes) > W25Q64_FLASH_SIZE_BYTES))
    {
        return FLIGHT_LOG_INVALID_ARGUMENT;
    }

    log->flash = flash;
    log->start_addr = start_addr;
    log->write_addr = start_addr;
    log->end_addr = start_addr + max_size_bytes;
    log->active = true;

    if (erase_before_start)
    {
        flash_result = W25Q64_EraseRange(flash, start_addr, max_size_bytes);
        if (flash_result != W25Q64_OK)
        {
            log->active = false;
            return FlightLog_FromFlashResult(flash_result);
        }
    }

    return FlightLog_AppendBytes(log,
                                 (const uint8_t *)FLIGHT_LOG_HEADER,
                                 (uint32_t)strlen(FLIGHT_LOG_HEADER));
}

FlightLog_Result_t FlightLog_BeginDefault(FlightLog_HandleTypeDef *log,
                                          W25Q64_HandleTypeDef *flash,
                                          bool erase_before_start)
{
    return FlightLog_Begin(log,
                           flash,
                           FLIGHT_LOG_DEFAULT_START_ADDR,
                           FLIGHT_LOG_DEFAULT_SIZE_BYTES,
                           erase_before_start);
}

FlightLog_Result_t FlightLog_AppendPayload(FlightLog_HandleTypeDef *log,
                                           uint32_t time_ms,
                                           const int16_t imu_data[6],
                                           const uint32_t baro_data[2],
                                           const int16_t mag_data[3],
                                           const int16_t calc_data[4],
                                           uint8_t deploy_state,
                                           int32_t error_state,
                                           const char *message)
{
    char row[256];
    char safe_message[48];
    int n;

    if ((log == NULL) || (imu_data == NULL) || (baro_data == NULL) ||
        (mag_data == NULL) || (calc_data == NULL))
    {
        return FLIGHT_LOG_INVALID_ARGUMENT;
    }

    FlightLog_SanitizeMessage(safe_message, sizeof(safe_message), message);

    n = snprintf(row, sizeof(row),
                 "%lu,"
                 "%d,%d,%d,%d,%d,%d,"
                 "%lu,%lu,"
                 "%d,%d,%d,"
                 "%d,%d,%d,%d,"
                 "%u,%ld,%s\n",
                 (unsigned long)time_ms,

                 (int)imu_data[0],
                 (int)imu_data[1],
                 (int)imu_data[2],
                 (int)imu_data[3],
                 (int)imu_data[4],
                 (int)imu_data[5],

                 (unsigned long)baro_data[0],
                 (unsigned long)baro_data[1],

                 (int)mag_data[0],
                 (int)mag_data[1],
                 (int)mag_data[2],

                 (int)calc_data[0],
                 (int)calc_data[1],
                 (int)calc_data[2],
                 (int)calc_data[3],

                 (unsigned int)deploy_state,
                 (long)error_state,
                 safe_message);

    if ((n <= 0) || (n >= (int)sizeof(row)))
    {
        return FLIGHT_LOG_ERROR;
    }

    return FlightLog_AppendBytes(log, (const uint8_t *)row, (uint32_t)n);
}

FlightLog_Result_t FlightLog_AppendRawLine(FlightLog_HandleTypeDef *log,
                                           const char *line)
{
    uint32_t len;

    if (line == NULL)
    {
        return FLIGHT_LOG_INVALID_ARGUMENT;
    }

    len = (uint32_t)strlen(line);
    return FlightLog_AppendBytes(log, (const uint8_t *)line, len);
}

uint32_t FlightLog_GetWriteAddress(const FlightLog_HandleTypeDef *log)
{
    if (log == NULL)
    {
        return 0U;
    }

    return log->write_addr;
}

uint32_t FlightLog_GetBytesUsed(const FlightLog_HandleTypeDef *log)
{
    if (log == NULL)
    {
        return 0U;
    }

    if (log->write_addr < log->start_addr)
    {
        return 0U;
    }

    return log->write_addr - log->start_addr;
}
