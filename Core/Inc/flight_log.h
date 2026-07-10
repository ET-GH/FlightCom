/**
 * @file flight_log.h
 * @brief CSV append wrapper for payload data stored in W25Q64JV flash.
 *
 * Drop into Core/Inc.
 */

#ifndef FLIGHT_LOG_H
#define FLIGHT_LOG_H

#ifdef __cplusplus
extern "C" {
#endif

#include "w25q64.h"
#include <stdint.h>
#include <stdbool.h>

#define FLIGHT_LOG_DEFAULT_START_ADDR       0x001000UL
#define FLIGHT_LOG_DEFAULT_SIZE_BYTES       (1024UL * 1024UL)  /* 1 MiB default test log region */

/*
 * Your current payload arrays:
 *   imu_data[6]
 *   baro_data[2]
 *   mag_data[3]
 *   calc_data[4]
 */
typedef struct
{
    W25Q64_HandleTypeDef *flash;
    uint32_t start_addr;
    uint32_t write_addr;
    uint32_t end_addr;
    bool active;
} FlightLog_HandleTypeDef;

typedef enum
{
    FLIGHT_LOG_OK = 0,
    FLIGHT_LOG_ERROR = 1,
    FLIGHT_LOG_FULL = 2,
    FLIGHT_LOG_INACTIVE = 3,
    FLIGHT_LOG_INVALID_ARGUMENT = 4
} FlightLog_Result_t;

FlightLog_Result_t FlightLog_Begin(FlightLog_HandleTypeDef *log,
                                   W25Q64_HandleTypeDef *flash,
                                   uint32_t start_addr,
                                   uint32_t max_size_bytes,
                                   bool erase_before_start);

FlightLog_Result_t FlightLog_BeginDefault(FlightLog_HandleTypeDef *log,
                                          W25Q64_HandleTypeDef *flash,
                                          bool erase_before_start);

FlightLog_Result_t FlightLog_AppendPayload(FlightLog_HandleTypeDef *log,
                                           uint32_t time_ms,
                                           const int16_t imu_data[6],
                                           const uint32_t baro_data[2],
                                           const int16_t mag_data[3],
                                           const int16_t calc_data[4],
                                           uint8_t deploy_state,
                                           int32_t error_state,
                                           const char *message);

FlightLog_Result_t FlightLog_AppendRawLine(FlightLog_HandleTypeDef *log,
                                           const char *line);

uint32_t FlightLog_GetWriteAddress(const FlightLog_HandleTypeDef *log);
uint32_t FlightLog_GetBytesUsed(const FlightLog_HandleTypeDef *log);

#ifdef __cplusplus
}
#endif

#endif /* FLIGHT_LOG_H */
