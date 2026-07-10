#include "radio_bridge.h"
#include "sx1280.h"
#include <stdint.h>
#include <string.h>
#include <stdio.h>

#define RADIO_TX_TIMEOUT_MS              2000UL
#define RADIO_BRIDGE_HEARTBEAT_ENABLE     1
#define RADIO_BRIDGE_HEARTBEAT_PERIOD_MS  5000UL

#define PAYLOAD_MAGIC       0xA5
#define PAYLOAD_VERSION     0x01
#define PAYLOAD_TYPE_DATA   0x01

#define TAG_IMU             0x10
#define TAG_BARO            0x20
#define TAG_MAGNET          0x30
#define TAG_CALC            0x40
#define TAG_STATUS_MSG      0x50
#define TAG_COMMAND_MSG     0x51
#define TAG_ERROR_MSG       0x52

#define IMU_COUNT           6
#define BARO_COUNT          2
#define MAGNET_COUNT        3
#define CALC_COUNT          4
#define MAX_MESSAGE_LEN     48

static volatile uint8_t radio_dio1_seen = 0;
static uint32_t last_heartbeat_ms = 0;
static uint32_t heartbeat_counter = 0;
static uint16_t payload_sequence = 0;

static size_t RadioBridge_BoundedStringLength(const char *text, size_t maximum)
{
    size_t length = 0;

    if (text == 0)
    {
        return 0;
    }

    while ((length < maximum) && (text[length] != '\0'))
    {
        length++;
    }

    return length;
}

typedef struct
{
    uint8_t *data;
    uint16_t length;
    uint16_t capacity;
} PayloadBuilder;

static HAL_StatusTypeDef PayloadBuilder_Reserve(const PayloadBuilder *builder,
                                                 uint16_t bytes)
{
    if ((builder == 0) || (builder->data == 0))
    {
        return HAL_ERROR;
    }

    if ((builder->length + bytes) > builder->capacity)
    {
        return HAL_ERROR;
    }

    return HAL_OK;
}

static HAL_StatusTypeDef PayloadBuilder_AddU8(PayloadBuilder *builder,
                                               uint8_t value)
{
    if (PayloadBuilder_Reserve(builder, 1) != HAL_OK)
    {
        return HAL_ERROR;
    }

    builder->data[builder->length++] = value;
    return HAL_OK;
}

static HAL_StatusTypeDef PayloadBuilder_AddU16(PayloadBuilder *builder,
                                                uint16_t value)
{
    if (PayloadBuilder_Reserve(builder, 2) != HAL_OK)
    {
        return HAL_ERROR;
    }

    builder->data[builder->length++] = (uint8_t)(value & 0xFFU);
    builder->data[builder->length++] = (uint8_t)((value >> 8) & 0xFFU);
    return HAL_OK;
}

static HAL_StatusTypeDef PayloadBuilder_AddU32(PayloadBuilder *builder,
                                                uint32_t value)
{
    if (PayloadBuilder_Reserve(builder, 4) != HAL_OK)
    {
        return HAL_ERROR;
    }

    builder->data[builder->length++] = (uint8_t)(value & 0xFFUL);
    builder->data[builder->length++] = (uint8_t)((value >> 8) & 0xFFUL);
    builder->data[builder->length++] = (uint8_t)((value >> 16) & 0xFFUL);
    builder->data[builder->length++] = (uint8_t)((value >> 24) & 0xFFUL);
    return HAL_OK;
}

static HAL_StatusTypeDef PayloadBuilder_AddU16Array(PayloadBuilder *builder,
                                                     uint8_t tag,
                                                     const uint16_t *values,
                                                     uint8_t count)
{
    uint8_t i;

    if ((values == 0) || (count == 0))
    {
        return HAL_ERROR;
    }

    if (PayloadBuilder_Reserve(builder, (uint16_t)(2U + (2U * count))) != HAL_OK)
    {
        return HAL_ERROR;
    }

    if ((PayloadBuilder_AddU8(builder, tag) != HAL_OK) ||
        (PayloadBuilder_AddU8(builder, count) != HAL_OK))
    {
        return HAL_ERROR;
    }

    for (i = 0; i < count; i++)
    {
        if (PayloadBuilder_AddU16(builder, values[i]) != HAL_OK)
        {
            return HAL_ERROR;
        }
    }

    return HAL_OK;
}

static HAL_StatusTypeDef PayloadBuilder_AddU32Array(PayloadBuilder *builder,
                                                     uint8_t tag,
                                                     const uint32_t *values,
                                                     uint8_t count)
{
    uint8_t i;

    if ((values == 0) || (count == 0))
    {
        return HAL_ERROR;
    }

    if (PayloadBuilder_Reserve(builder, (uint16_t)(2U + (4U * count))) != HAL_OK)
    {
        return HAL_ERROR;
    }

    if ((PayloadBuilder_AddU8(builder, tag) != HAL_OK) ||
        (PayloadBuilder_AddU8(builder, count) != HAL_OK))
    {
        return HAL_ERROR;
    }

    for (i = 0; i < count; i++)
    {
        if (PayloadBuilder_AddU32(builder, values[i]) != HAL_OK)
        {
            return HAL_ERROR;
        }
    }

    return HAL_OK;
}

static HAL_StatusTypeDef PayloadBuilder_AddString(PayloadBuilder *builder,
                                                   uint8_t tag,
                                                   const char *message)
{
    size_t len;

    if (message == 0)
    {
        return HAL_OK;
    }

    len = RadioBridge_BoundedStringLength(message, MAX_MESSAGE_LEN);
    if (len == 0)
    {
        return HAL_OK;
    }

    if (PayloadBuilder_Reserve(builder, (uint16_t)(2U + len)) != HAL_OK)
    {
        return HAL_ERROR;
    }

    if ((PayloadBuilder_AddU8(builder, tag) != HAL_OK) ||
        (PayloadBuilder_AddU8(builder, (uint8_t)len) != HAL_OK))
    {
        return HAL_ERROR;
    }

    memcpy(&builder->data[builder->length], message, len);
    builder->length += (uint16_t)len;
    return HAL_OK;
}

HAL_StatusTypeDef PayloadPipeline(uint16_t *IMU,
                                  uint32_t *Baro,
                                  uint16_t *Magnet,
                                  uint16_t *Calc,
                                  const char *Message[3],
                                  uint8_t deployment_state)
{
    uint8_t tx_buffer[SX1280_MAX_PAYLOAD_LEN];
    uint32_t current_time_ms = HAL_GetTick();
    PayloadBuilder builder = {
        .data = tx_buffer,
        .length = 0,
        .capacity = SX1280_MAX_PAYLOAD_LEN
    };

    if ((IMU == 0) || (Baro == 0) || (Magnet == 0) || (Calc == 0))
    {
        return HAL_ERROR;
    }

    /* Application header.  The SX1280 adds the physical LoRa packet fields. */
    if ((PayloadBuilder_AddU8(&builder, PAYLOAD_MAGIC) != HAL_OK) ||
        (PayloadBuilder_AddU8(&builder, PAYLOAD_VERSION) != HAL_OK) ||
        (PayloadBuilder_AddU8(&builder, PAYLOAD_TYPE_DATA) != HAL_OK) ||
        (PayloadBuilder_AddU16(&builder, payload_sequence++) != HAL_OK) ||
        (PayloadBuilder_AddU32(&builder, current_time_ms) != HAL_OK) ||
        (PayloadBuilder_AddU8(&builder, deployment_state) != HAL_OK))
    {
        return HAL_ERROR;
    }

    if ((PayloadBuilder_AddU16Array(&builder, TAG_IMU, IMU, IMU_COUNT) != HAL_OK) ||
        (PayloadBuilder_AddU32Array(&builder, TAG_BARO, Baro, BARO_COUNT) != HAL_OK) ||
        (PayloadBuilder_AddU16Array(&builder, TAG_MAGNET, Magnet, MAGNET_COUNT) != HAL_OK) ||
        (PayloadBuilder_AddU16Array(&builder, TAG_CALC, Calc, CALC_COUNT) != HAL_OK))
    {
        return HAL_ERROR;
    }

    if (Message != 0)
    {
        if ((PayloadBuilder_AddString(&builder, TAG_STATUS_MSG, Message[0]) != HAL_OK) ||
            (PayloadBuilder_AddString(&builder, TAG_COMMAND_MSG, Message[1]) != HAL_OK) ||
            (PayloadBuilder_AddString(&builder, TAG_ERROR_MSG, Message[2]) != HAL_OK))
        {
            return HAL_ERROR;
        }
    }

    if ((builder.length == 0) || (builder.length > SX1280_MAX_PAYLOAD_LEN))
    {
        return HAL_ERROR;
    }

    return SX1280_Transmit(tx_buffer,
                           (uint8_t)builder.length,
                           RADIO_TX_TIMEOUT_MS);
}

/*
 * If HAL_GPIO_EXTI_Callback() is already defined elsewhere, merge this case
 * into that callback instead of keeping two definitions.
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == LORA_DIO1_Pin)
    {
        radio_dio1_seen = 1;
    }
}

HAL_StatusTypeDef RadioBridge_Init(void)
{
    HAL_StatusTypeDef status;
    const char boot_msg[] = "STM32_BOOT";

    status = SX1280_InitLoRa();
    if (status != HAL_OK)
    {
        return status;
    }

    status = SX1280_Transmit((const uint8_t *)boot_msg,
                             (uint8_t)strlen(boot_msg),
                             RADIO_TX_TIMEOUT_MS);
    if (status != HAL_OK)
    {
        return status;
    }

    last_heartbeat_ms = HAL_GetTick();
    return HAL_OK;
}

HAL_StatusTypeDef RadioBridge_SendText(const char *text)
{
    size_t len;

    if (text == 0)
    {
        return HAL_ERROR;
    }

    len = RadioBridge_BoundedStringLength(text, SX1280_MAX_PAYLOAD_LEN + 1U);
    if ((len == 0) || (len > SX1280_MAX_PAYLOAD_LEN))
    {
        return HAL_ERROR;
    }

    return SX1280_Transmit((const uint8_t *)text,
                           (uint8_t)len,
                           RADIO_TX_TIMEOUT_MS);
}

void RadioBridge_Task(void)
{
    uint8_t payload[SX1280_MAX_PAYLOAD_LEN + 1U];
    uint8_t payload_len = 0;

    if ((radio_dio1_seen != 0) ||
        (HAL_GPIO_ReadPin(LORA_DIO1_GPIO_Port, LORA_DIO1_Pin) == GPIO_PIN_SET))
    {
        radio_dio1_seen = 0;

        if (SX1280_ReadPacketIfAvailable(payload, &payload_len) == HAL_OK)
        {
            payload[payload_len] = '\0';

            /* Temporary text acknowledgement used during radio bring-up. */
            char reply[SX1280_MAX_PAYLOAD_LEN + 20U];
            int n = snprintf(reply, sizeof(reply), "STM32_RX:%s", (char *)payload);

            if (n > 0)
            {
                if (n > (int)SX1280_MAX_PAYLOAD_LEN)
                {
                    n = SX1280_MAX_PAYLOAD_LEN;
                }

                (void)SX1280_Transmit((const uint8_t *)reply,
                                      (uint8_t)n,
                                      RADIO_TX_TIMEOUT_MS);
            }
        }
    }

#if RADIO_BRIDGE_HEARTBEAT_ENABLE
    if ((HAL_GetTick() - last_heartbeat_ms) >= RADIO_BRIDGE_HEARTBEAT_PERIOD_MS)
    {
        char msg[48];
        int n;

        last_heartbeat_ms = HAL_GetTick();
        n = snprintf(msg, sizeof(msg), "STM32_HEARTBEAT_%lu",
                     (unsigned long)heartbeat_counter++);

        if (n > 0)
        {
            (void)SX1280_Transmit((const uint8_t *)msg,
                                  (uint8_t)n,
                                  RADIO_TX_TIMEOUT_MS);
        }
    }
#endif
}
