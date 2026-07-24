/*
 * AMBAR portable USB HIL transport implementation.
 *
 * The byte transforms and field offsets in this file intentionally match the
 * active firmware's Rocket Protocol v2 implementation.  This module performs
 * transport framing only; it never authorizes or executes a command.
 */

#include "ambar_hil_usb.h"

#include <string.h>

#define AMBAR_HIL_USB_READ_CHUNK_SIZE 64u
#define AMBAR_HIL_USB_READS_PER_POLL   4u

typedef struct
{
    uint8_t type;
    uint16_t sequence;
    uint32_t time_ms;
    uint8_t payload_length;
    uint8_t payload[AMBAR_HIL_USB_MAX_PAYLOAD_SIZE];
} AmbarHilUsbDecodedPacket;

static void write_u16(uint8_t *destination, uint16_t value)
{
    destination[0] = (uint8_t)value;
    destination[1] = (uint8_t)(value >> 8);
}

static void write_i16(uint8_t *destination, int16_t value)
{
    write_u16(destination, (uint16_t)value);
}

static void write_u32(uint8_t *destination, uint32_t value)
{
    destination[0] = (uint8_t)value;
    destination[1] = (uint8_t)(value >> 8);
    destination[2] = (uint8_t)(value >> 16);
    destination[3] = (uint8_t)(value >> 24);
}

static void write_i32(uint8_t *destination, int32_t value)
{
    write_u32(destination, (uint32_t)value);
}

static uint16_t read_u16(const uint8_t *source)
{
    return (uint16_t)source[0] | ((uint16_t)source[1] << 8);
}

static int16_t read_i16(const uint8_t *source)
{
    return (int16_t)read_u16(source);
}

static uint32_t read_u32(const uint8_t *source)
{
    return (uint32_t)source[0]
        | ((uint32_t)source[1] << 8)
        | ((uint32_t)source[2] << 16)
        | ((uint32_t)source[3] << 24);
}

static int32_t read_i32(const uint8_t *source)
{
    return (int32_t)read_u32(source);
}

static uint16_t crc16_ccitt_false(const uint8_t *data, size_t length)
{
    uint16_t crc = 0xFFFFu;
    size_t index;

    if (data == NULL && length != 0u)
    {
        return 0u;
    }

    for (index = 0u; index < length; ++index)
    {
        uint8_t bit;
        crc ^= (uint16_t)data[index] << 8;
        for (bit = 0u; bit < 8u; ++bit)
        {
            crc = (crc & 0x8000u) != 0u
                ? (uint16_t)((crc << 1) ^ 0x1021u)
                : (uint16_t)(crc << 1);
        }
    }

    return crc;
}

static size_t cobs_encode(const uint8_t *input,
                          size_t input_length,
                          uint8_t *output,
                          size_t output_capacity)
{
    size_t read_index = 0u;
    size_t write_index = 1u;
    size_t code_index = 0u;
    uint8_t code = 1u;

    if ((input == NULL && input_length != 0u)
        || output == NULL
        || output_capacity == 0u)
    {
        return 0u;
    }

    while (read_index < input_length)
    {
        if (input[read_index] == 0u)
        {
            if (code_index >= output_capacity)
            {
                return 0u;
            }
            output[code_index] = code;
            code = 1u;
            code_index = write_index;
            ++write_index;
            ++read_index;
        }
        else
        {
            if (write_index >= output_capacity)
            {
                return 0u;
            }
            output[write_index] = input[read_index];
            ++write_index;
            ++read_index;
            ++code;

            if (code == 0xFFu)
            {
                if (code_index >= output_capacity)
                {
                    return 0u;
                }
                output[code_index] = code;
                code = 1u;
                code_index = write_index;
                ++write_index;
            }
        }
    }

    if (code_index >= output_capacity)
    {
        return 0u;
    }
    output[code_index] = code;
    return write_index;
}

static size_t cobs_decode(const uint8_t *input,
                          size_t input_length,
                          uint8_t *output,
                          size_t output_capacity)
{
    size_t read_index = 0u;
    size_t write_index = 0u;

    if (input == NULL || input_length == 0u || output == NULL)
    {
        return 0u;
    }

    while (read_index < input_length)
    {
        const uint8_t code = input[read_index];
        const size_t copy_length = (size_t)code - 1u;
        ++read_index;

        if (code == 0u
            || copy_length > input_length - read_index
            || write_index > output_capacity
            || copy_length > output_capacity - write_index)
        {
            return 0u;
        }

        if (copy_length != 0u)
        {
            memcpy(output + write_index, input + read_index, copy_length);
            read_index += copy_length;
            write_index += copy_length;
        }

        if (code != 0xFFu && read_index < input_length)
        {
            if (write_index >= output_capacity)
            {
                return 0u;
            }
            output[write_index] = 0u;
            ++write_index;
        }
    }

    return write_index;
}

static size_t encode_frame(uint8_t *frame,
                           size_t frame_capacity,
                           uint8_t type,
                           uint16_t sequence,
                           uint32_t time_ms,
                           const uint8_t *payload,
                           size_t payload_length)
{
    uint8_t raw[AMBAR_HIL_USB_MAX_PACKET_SIZE];
    size_t covered_length;
    size_t encoded_length;

    if (frame == NULL
        || frame_capacity < 2u
        || payload_length > AMBAR_HIL_USB_MAX_PAYLOAD_SIZE
        || (payload == NULL && payload_length != 0u))
    {
        return 0u;
    }

    raw[0] = AMBAR_HIL_USB_MAGIC;
    raw[1] = AMBAR_HIL_USB_PROTOCOL_VERSION;
    raw[2] = type;
    write_u16(raw + 3u, sequence);
    write_u32(raw + 5u, time_ms);
    if (payload_length != 0u)
    {
        memcpy(raw + AMBAR_HIL_USB_HEADER_SIZE, payload, payload_length);
    }

    covered_length = AMBAR_HIL_USB_HEADER_SIZE + payload_length;
    write_u16(raw + covered_length, crc16_ccitt_false(raw, covered_length));
    encoded_length = cobs_encode(raw,
                                 covered_length + AMBAR_HIL_USB_CRC_SIZE,
                                 frame,
                                 frame_capacity - 1u);
    if (encoded_length == 0u || encoded_length >= frame_capacity)
    {
        return 0u;
    }

    frame[encoded_length] = AMBAR_HIL_USB_FRAME_DELIMITER;
    return encoded_length + 1u;
}

static bool decode_frame(const uint8_t *encoded,
                         size_t encoded_length,
                         AmbarHilUsbDecodedPacket *packet)
{
    uint8_t raw[AMBAR_HIL_USB_MAX_PACKET_SIZE];
    size_t raw_length;
    size_t covered_length;
    size_t payload_length;

    if (encoded == NULL
        || packet == NULL
        || encoded_length == 0u
        || encoded_length >= AMBAR_HIL_USB_MAX_FRAME_SIZE)
    {
        return false;
    }

    raw_length = cobs_decode(encoded, encoded_length, raw, sizeof(raw));
    if (raw_length < AMBAR_HIL_USB_HEADER_SIZE + AMBAR_HIL_USB_CRC_SIZE
        || raw_length > AMBAR_HIL_USB_MAX_PACKET_SIZE
        || raw[0] != AMBAR_HIL_USB_MAGIC
        || raw[1] != AMBAR_HIL_USB_PROTOCOL_VERSION)
    {
        return false;
    }

    covered_length = raw_length - AMBAR_HIL_USB_CRC_SIZE;
    if (crc16_ccitt_false(raw, covered_length)
        != read_u16(raw + covered_length))
    {
        return false;
    }

    payload_length = covered_length - AMBAR_HIL_USB_HEADER_SIZE;
    packet->type = raw[2];
    packet->sequence = read_u16(raw + 3u);
    packet->time_ms = read_u32(raw + 5u);
    packet->payload_length = (uint8_t)payload_length;
    if (payload_length != 0u)
    {
        memcpy(packet->payload,
               raw + AMBAR_HIL_USB_HEADER_SIZE,
               payload_length);
    }
    return true;
}

static bool decode_command(const uint8_t *data,
                           size_t length,
                           AmbarHilUsbCommand *command)
{
    uint8_t payload_length;

    if (data == NULL
        || command == NULL
        || length < AMBAR_HIL_USB_COMMAND_PREFIX_SIZE)
    {
        return false;
    }

    payload_length = data[1];
    if (payload_length > AMBAR_HIL_USB_COMMAND_DATA_MAX
        || length != AMBAR_HIL_USB_COMMAND_PREFIX_SIZE + payload_length)
    {
        return false;
    }

    command->command = data[0];
    command->payload_length = payload_length;
    if (payload_length != 0u)
    {
        memcpy(command->payload,
               data + AMBAR_HIL_USB_COMMAND_PREFIX_SIZE,
               payload_length);
    }
    return true;
}

static bool decode_simulation(const uint8_t *data,
                              size_t length,
                              AmbarHilUsbSimulation *simulation)
{
    if (data == NULL
        || simulation == NULL
        || length != AMBAR_HIL_USB_SIMULATION_PAYLOAD_SIZE)
    {
        return false;
    }

    simulation->flags = read_u16(data + 0u);
    simulation->altitude_mm = read_i32(data + 2u);
    simulation->acceleration_mmps2 = read_i32(data + 6u);
    simulation->velocity_mmps = read_i32(data + 10u);
    simulation->barometer_stddev_cm = read_u16(data + 14u);
    return true;
}

static bool decode_variable_config(const uint8_t *data,
                                   size_t length,
                                   AmbarHilUsbVariableHilConfig *config)
{
    size_t index;

    if (data == NULL
        || config == NULL
        || length != AMBAR_HIL_USB_VARIABLE_CONFIG_SIZE)
    {
        return false;
    }

    config->schema_version = data[0];
    config->control_mode = data[1];
    config->predictor_mode = data[2];
    config->cda_point_count = data[3];
    config->calibration_version = read_u32(data + 4u);
    config->target_apogee_dm = read_u16(data + 8u);
    config->mission_tolerance_dm = read_u16(data + 10u);
    config->control_deadband_cm = read_u16(data + 12u);
    config->full_deployment_error_dm = read_u16(data + 14u);
    config->minimum_deploy_altitude_dm = read_u16(data + 16u);
    config->minimum_flight_time_cs = read_u16(data + 18u);
    config->predictive_update_period_ms = read_u16(data + 20u);
    config->coast_mass_g = read_u16(data + 22u);
    config->maximum_deploy_u8 = data[24];
    config->hysteresis_permille = data[25];
    for (index = 0u; index < AMBAR_HIL_USB_CDA_POINT_COUNT; ++index)
    {
        config->deployment_cda_um2[index] = read_u16(data + 26u + 2u * index);
    }
    config->air_density_1e4_kgpm3 = read_u16(data + 36u);
    config->density_scale_height_m = read_u16(data + 38u);
    config->launch_site_elevation_dm = read_i16(data + 40u);
    config->actuator_delay_ms = read_u16(data + 42u);
    config->actuator_open_rate_milli_per_s = read_u16(data + 44u);
    config->actuator_close_rate_milli_per_s = read_u16(data + 46u);
    config->config_crc32 = read_u32(data + 48u);
    return true;
}

static bool decode_message(const AmbarHilUsbDecodedPacket *packet,
                           AmbarHilUsbMessage *message)
{
    if (packet == NULL || message == NULL)
    {
        return false;
    }

    memset(message, 0, sizeof(*message));
    message->packet_type = packet->type;
    message->sequence = packet->sequence;
    message->time_ms = packet->time_ms;

    switch (packet->type)
    {
    case AMBAR_HIL_USB_PACKET_COMMAND:
        message->kind = AMBAR_HIL_USB_RX_COMMAND;
        return decode_command(packet->payload,
                              packet->payload_length,
                              &message->body.command);

    case AMBAR_HIL_USB_PACKET_SIMULATION:
        message->kind = AMBAR_HIL_USB_RX_SIMULATION;
        return decode_simulation(packet->payload,
                                 packet->payload_length,
                                 &message->body.simulation);

    case AMBAR_HIL_USB_PACKET_VARIABLE_HIL_CONFIG_UPLOAD:
        message->kind = AMBAR_HIL_USB_RX_VARIABLE_HIL_CONFIG_UPLOAD;
        return decode_variable_config(packet->payload,
                                      packet->payload_length,
                                      &message->body.variable_hil_config);

    default:
        message->kind = AMBAR_HIL_USB_RX_RAW_PACKET;
        message->body.raw.length = packet->payload_length;
        if (packet->payload_length != 0u)
        {
            memcpy(message->body.raw.bytes,
                   packet->payload,
                   packet->payload_length);
        }
        return true;
    }
}

static bool message_is_estop(const AmbarHilUsbMessage *message)
{
    return message != NULL
        && message->kind == AMBAR_HIL_USB_RX_COMMAND
        && message->body.command.command == AMBAR_HIL_USB_COMMAND_ESTOP
        && message->body.command.payload_length == 0u;
}

static void clear_streams(AmbarHilUsb *context, bool account_transmit_drops)
{
    if (account_transmit_drops)
    {
        context->stats.transmit_drops += context->transmit_count;
        if (context->priority_transmit_pending)
        {
            ++context->stats.transmit_drops;
        }
    }

    context->encoded_receive_length = 0u;
    context->drop_until_delimiter = false;
    context->receive_head = 0u;
    context->receive_tail = 0u;
    context->receive_count = 0u;
    context->priority_estop_pending = false;
    context->transmit_head = 0u;
    context->transmit_tail = 0u;
    context->transmit_count = 0u;
    context->priority_transmit_pending = false;
}

static void queue_received(AmbarHilUsb *context,
                           const AmbarHilUsbMessage *message)
{
    if (message_is_estop(message))
    {
        if (context->priority_estop_pending)
        {
            ++context->stats.receive_queue_drops;
        }
        context->priority_estop = *message;
        context->priority_estop_pending = true;
        return;
    }

    if (context->receive_count >= AMBAR_HIL_USB_RX_QUEUE_DEPTH)
    {
        ++context->stats.receive_queue_drops;
        return;
    }

    context->receive_queue[context->receive_tail] = *message;
    context->receive_tail = (uint8_t)(
        (context->receive_tail + 1u) % AMBAR_HIL_USB_RX_QUEUE_DEPTH);
    ++context->receive_count;
}

static void finish_encoded_frame(AmbarHilUsb *context)
{
    AmbarHilUsbDecodedPacket packet;
    AmbarHilUsbMessage message;

    if (!decode_frame(context->encoded_receive,
                      context->encoded_receive_length,
                      &packet))
    {
        ++context->stats.receive_errors;
    }
    else
    {
        ++context->stats.received_frames;
        if (decode_message(&packet, &message))
        {
            queue_received(context, &message);
        }
        else
        {
            ++context->stats.receive_errors;
        }
    }

    context->encoded_receive_length = 0u;
}

static void consume_bytes(AmbarHilUsb *context,
                          const uint8_t *data,
                          size_t length)
{
    size_t index;

    for (index = 0u; index < length; ++index)
    {
        const uint8_t byte = data[index];

        if (byte == AMBAR_HIL_USB_FRAME_DELIMITER)
        {
            if (context->drop_until_delimiter)
            {
                context->drop_until_delimiter = false;
                context->encoded_receive_length = 0u;
            }
            else if (context->encoded_receive_length != 0u)
            {
                finish_encoded_frame(context);
            }
            continue;
        }

        if (context->drop_until_delimiter)
        {
            continue;
        }

        if (context->encoded_receive_length >= sizeof(context->encoded_receive))
        {
            ++context->stats.receive_errors;
            context->encoded_receive_length = 0u;
            context->drop_until_delimiter = true;
            continue;
        }

        context->encoded_receive[context->encoded_receive_length] = byte;
        ++context->encoded_receive_length;
    }
}

static bool queue_frame(AmbarHilUsb *context,
                        uint8_t packet_type,
                        uint16_t sequence,
                        const uint8_t *payload,
                        size_t payload_length,
                        bool priority)
{
    AmbarHilUsbTxFrame *frame;
    size_t length;

    if (context == NULL
        || !context->initialized
        || !context->connected
        || payload_length > AMBAR_HIL_USB_MAX_PAYLOAD_SIZE
        || (payload == NULL && payload_length != 0u))
    {
        if (context != NULL && context->initialized)
        {
            ++context->stats.transmit_drops;
        }
        return false;
    }

    if (priority)
    {
        if (context->priority_transmit_pending
            && context->priority_transmit.offset != 0u)
        {
            ++context->stats.transmit_drops;
            return false;
        }
        if (context->priority_transmit_pending)
        {
            ++context->stats.transmit_drops;
        }
        frame = &context->priority_transmit;
    }
    else
    {
        if (context->transmit_count >= AMBAR_HIL_USB_TX_QUEUE_DEPTH)
        {
            ++context->stats.transmit_drops;
            return false;
        }
        frame = &context->transmit_queue[context->transmit_tail];
    }

    length = encode_frame(frame->bytes,
                          sizeof(frame->bytes),
                          packet_type,
                          sequence,
                          context->io.now_ms(context->io.user),
                          payload,
                          payload_length);
    if (length == 0u || length > UINT8_MAX)
    {
        ++context->stats.transmit_drops;
        return false;
    }

    frame->length = (uint8_t)length;
    frame->offset = 0u;
    if (priority)
    {
        context->priority_transmit_pending = true;
    }
    else
    {
        context->transmit_tail = (uint8_t)(
            (context->transmit_tail + 1u) % AMBAR_HIL_USB_TX_QUEUE_DEPTH);
        ++context->transmit_count;
    }
    return true;
}

static bool queue_auto_sequence(AmbarHilUsb *context,
                                uint8_t packet_type,
                                const uint8_t *payload,
                                size_t payload_length,
                                bool priority)
{
    uint16_t sequence;

    if (context == NULL)
    {
        return false;
    }
    sequence = context->transmit_sequence;
    if (!queue_frame(context,
                     packet_type,
                     sequence,
                     payload,
                     payload_length,
                     priority))
    {
        return false;
    }
    context->transmit_sequence = (uint16_t)(sequence + 1u);
    return true;
}

static void drop_selected_frame(AmbarHilUsb *context, bool selected_priority)
{
    if (selected_priority)
    {
        context->priority_transmit_pending = false;
    }
    else
    {
        context->transmit_head = (uint8_t)(
            (context->transmit_head + 1u) % AMBAR_HIL_USB_TX_QUEUE_DEPTH);
        --context->transmit_count;
    }
}

static void service_transmit(AmbarHilUsb *context)
{
    AmbarHilUsbTxFrame *normal = NULL;
    AmbarHilUsbTxFrame *frame = NULL;
    bool selected_priority = false;
    size_t remaining;
    size_t written;

    if (context->transmit_count != 0u)
    {
        normal = &context->transmit_queue[context->transmit_head];
    }

    /* Never interleave a priority frame into a normal frame already in flight. */
    if (normal != NULL && normal->offset != 0u)
    {
        frame = normal;
    }
    else if (context->priority_transmit_pending)
    {
        frame = &context->priority_transmit;
        selected_priority = true;
    }
    else
    {
        frame = normal;
    }

    if (frame == NULL)
    {
        return;
    }

    remaining = (size_t)frame->length - frame->offset;
    written = context->io.write(context->io.user,
                                frame->bytes + frame->offset,
                                remaining);
    if (written == AMBAR_HIL_USB_IO_ERROR || written > remaining)
    {
        ++context->stats.io_errors;
        ++context->stats.transmit_drops;
        drop_selected_frame(context, selected_priority);
        return;
    }

    frame->offset = (uint8_t)(frame->offset + written);
    if (frame->offset == frame->length)
    {
        drop_selected_frame(context, selected_priority);
        ++context->stats.transmitted_frames;
    }
}

static size_t encode_telemetry(uint8_t *out,
                               size_t capacity,
                               const AmbarHilUsbTelemetry *payload)
{
    if (out == NULL
        || payload == NULL
        || capacity < AMBAR_HIL_USB_TELEMETRY_PAYLOAD_SIZE)
    {
        return 0u;
    }

    write_u16(out + 0u, payload->flags);
    out[2] = payload->state;
    out[3] = payload->status_code;
    write_i16(out + 4u, payload->altitude_dm);
    write_i16(out + 6u, payload->velocity_cms);
    write_i16(out + 8u, payload->acceleration_cms2);
    write_u16(out + 10u, payload->predicted_apogee_dm);
    write_u16(out + 12u, payload->target_apogee_dm);
    write_i16(out + 14u, payload->roll_ddeg);
    write_i16(out + 16u, payload->pitch_ddeg);
    write_i16(out + 18u, payload->yaw_ddeg);
    out[20] = payload->deployment_percent;
    out[21] = payload->sensor_health;
    write_u16(out + 22u, payload->failed_reads);
    out[24] = payload->message_code;
    out[25] = payload->reserved;
    return AMBAR_HIL_USB_TELEMETRY_PAYLOAD_SIZE;
}

static size_t encode_event(uint8_t *out,
                           size_t capacity,
                           const AmbarHilUsbEvent *payload)
{
    if (out == NULL
        || payload == NULL
        || capacity < AMBAR_HIL_USB_EVENT_PAYLOAD_SIZE)
    {
        return 0u;
    }

    write_u16(out + 0u, payload->changed_flags);
    write_u16(out + 2u, payload->current_flags);
    out[4] = payload->previous_state;
    out[5] = payload->current_state;
    out[6] = payload->status_code;
    out[7] = payload->message_code;
    write_u16(out + 8u, payload->detail);
    return AMBAR_HIL_USB_EVENT_PAYLOAD_SIZE;
}

static size_t encode_ack(uint8_t *out,
                         size_t capacity,
                         const AmbarHilUsbAck *payload)
{
    if (out == NULL
        || payload == NULL
        || capacity < AMBAR_HIL_USB_ACK_PAYLOAD_SIZE)
    {
        return 0u;
    }

    write_u16(out + 0u, payload->command_sequence);
    out[2] = payload->command;
    out[3] = payload->result;
    write_u16(out + 4u, payload->detail);
    return AMBAR_HIL_USB_ACK_PAYLOAD_SIZE;
}

static size_t encode_actuator(uint8_t *out,
                              size_t capacity,
                              const AmbarHilUsbActuatorStatus *payload)
{
    if (out == NULL
        || payload == NULL
        || capacity < AMBAR_HIL_USB_ACTUATOR_PAYLOAD_SIZE)
    {
        return 0u;
    }

    write_u32(out + 0u, payload->actuator_inhibit_flags);
    write_u32(out + 4u, payload->flight_inhibit_flags);
    write_i32(out + 8u, payload->target_steps);
    write_i32(out + 12u, payload->actual_steps);
    write_u32(out + 16u, payload->driver_status);
    out[20] = payload->machine_state;
    out[21] = payload->flags;
    write_u16(out + 22u, payload->reserved);
    return AMBAR_HIL_USB_ACTUATOR_PAYLOAD_SIZE;
}

static size_t encode_heartbeat(uint8_t *out,
                               size_t capacity,
                               const AmbarHilUsbHeartbeat *payload)
{
    if (out == NULL
        || payload == NULL
        || capacity < AMBAR_HIL_USB_HEARTBEAT_PAYLOAD_SIZE)
    {
        return 0u;
    }

    write_u32(out + 0u, payload->feature_flags);
    write_u16(out + 4u, payload->receive_errors);
    write_u16(out + 6u, payload->transmit_drops);
    return AMBAR_HIL_USB_HEARTBEAT_PAYLOAD_SIZE;
}

static size_t encode_variable_state(uint8_t *out,
                                    size_t capacity,
                                    const AmbarHilUsbVariableHilState *payload)
{
    if (out == NULL
        || payload == NULL
        || capacity < AMBAR_HIL_USB_VARIABLE_STATE_SIZE)
    {
        return 0u;
    }

    write_u16(out + 0u, payload->simulation_sequence);
    write_u16(out + 2u, payload->controller_fraction_u16);
    write_u16(out + 4u, payload->actuator_target_fraction_u16);
    write_u16(out + 6u, payload->xactual_fraction_u16);
    write_i32(out + 8u, payload->target_steps);
    write_i32(out + 12u, payload->actual_steps);
    write_u32(out + 16u, payload->flight_inhibit_flags);
    write_u32(out + 20u, payload->actuator_inhibit_flags);
    write_u32(out + 24u, payload->driver_status);
    write_u32(out + 28u, payload->config_crc32);
    write_i32(out + 32u, payload->closed_predicted_apogee_dm);
    write_i32(out + 36u, payload->full_predicted_apogee_dm);
    out[40] = payload->phase;
    out[41] = payload->machine_state;
    out[42] = payload->state_flags;
    out[43] = payload->feedback_source;
    return AMBAR_HIL_USB_VARIABLE_STATE_SIZE;
}

static size_t encode_variable_config(uint8_t *out,
                                     size_t capacity,
                                     const AmbarHilUsbVariableHilConfig *payload)
{
    size_t index;

    if (out == NULL
        || payload == NULL
        || capacity < AMBAR_HIL_USB_VARIABLE_CONFIG_SIZE)
    {
        return 0u;
    }

    out[0] = payload->schema_version;
    out[1] = payload->control_mode;
    out[2] = payload->predictor_mode;
    out[3] = payload->cda_point_count;
    write_u32(out + 4u, payload->calibration_version);
    write_u16(out + 8u, payload->target_apogee_dm);
    write_u16(out + 10u, payload->mission_tolerance_dm);
    write_u16(out + 12u, payload->control_deadband_cm);
    write_u16(out + 14u, payload->full_deployment_error_dm);
    write_u16(out + 16u, payload->minimum_deploy_altitude_dm);
    write_u16(out + 18u, payload->minimum_flight_time_cs);
    write_u16(out + 20u, payload->predictive_update_period_ms);
    write_u16(out + 22u, payload->coast_mass_g);
    out[24] = payload->maximum_deploy_u8;
    out[25] = payload->hysteresis_permille;
    for (index = 0u; index < AMBAR_HIL_USB_CDA_POINT_COUNT; ++index)
    {
        write_u16(out + 26u + 2u * index, payload->deployment_cda_um2[index]);
    }
    write_u16(out + 36u, payload->air_density_1e4_kgpm3);
    write_u16(out + 38u, payload->density_scale_height_m);
    write_i16(out + 40u, payload->launch_site_elevation_dm);
    write_u16(out + 42u, payload->actuator_delay_ms);
    write_u16(out + 44u, payload->actuator_open_rate_milli_per_s);
    write_u16(out + 46u, payload->actuator_close_rate_milli_per_s);
    write_u32(out + 48u, payload->config_crc32);
    return AMBAR_HIL_USB_VARIABLE_CONFIG_SIZE;
}

static size_t encode_log_status(
    uint8_t *out,
    size_t capacity,
    const AmbarHilUsbLogStatus *payload)
{
    if ((out == NULL) ||
        (payload == NULL) ||
        (capacity < AMBAR_HIL_USB_LOG_STATUS_PAYLOAD_SIZE))
    {
        return 0u;
    }

    write_u16(out + 0u, payload->command_sequence);
    out[2] = payload->state;
    out[3] = payload->error_code;
    write_u32(out + 4u, payload->total_records);
    write_u32(out + 8u, payload->records_sent);
    write_u32(out + 12u, payload->corrupt_records);

    return AMBAR_HIL_USB_LOG_STATUS_PAYLOAD_SIZE;
}

bool AmbarHilUsb_Init(AmbarHilUsb *context, const AmbarHilUsbIo *io)
{
    if (context == NULL
        || io == NULL
        || io->read == NULL
        || io->write == NULL
        || io->now_ms == NULL
        || io->is_connected == NULL)
    {
        return false;
    }

    memset(context, 0, sizeof(*context));
    context->io = *io;
    context->initialized = true;
    return true;
}

void AmbarHilUsb_Reset(AmbarHilUsb *context)
{
    if (context == NULL || !context->initialized)
    {
        return;
    }

    clear_streams(context, true);
    context->transmit_sequence = 0u;
}

void AmbarHilUsb_Poll(AmbarHilUsb *context)
{
    bool connected;
    uint8_t read_buffer[AMBAR_HIL_USB_READ_CHUNK_SIZE];
    uint8_t read_pass;

    if (context == NULL || !context->initialized)
    {
        return;
    }

    connected = context->io.is_connected(context->io.user);
    if (!context->connection_observed)
    {
        context->connection_observed = true;
        context->connected = connected;
        if (connected)
        {
            ++context->stats.connection_count;
        }
    }
    else if (connected != context->connected)
    {
        if (connected)
        {
            ++context->stats.connection_count;
        }
        else
        {
            ++context->stats.disconnection_count;
            clear_streams(context, true);
        }
        context->connected = connected;
    }
    context->stats.connected = connected;

    if (!connected)
    {
        return;
    }

    for (read_pass = 0u; read_pass < AMBAR_HIL_USB_READS_PER_POLL; ++read_pass)
    {
        const size_t received = context->io.read(context->io.user,
                                                 read_buffer,
                                                 sizeof(read_buffer));
        if (received == AMBAR_HIL_USB_IO_ERROR || received > sizeof(read_buffer))
        {
            ++context->stats.io_errors;
            ++context->stats.receive_errors;
            context->encoded_receive_length = 0u;
            context->drop_until_delimiter = true;
            break;
        }
        if (received == 0u)
        {
            break;
        }

        consume_bytes(context, read_buffer, received);
        if (received < sizeof(read_buffer))
        {
            break;
        }
    }

    service_transmit(context);
}

bool AmbarHilUsb_TakeMessage(AmbarHilUsb *context,
                             AmbarHilUsbMessage *message)
{
    if (context == NULL
        || message == NULL
        || !context->initialized)
    {
        return false;
    }

    if (context->priority_estop_pending)
    {
        *message = context->priority_estop;
        context->priority_estop_pending = false;
        return true;
    }

    if (context->receive_count == 0u)
    {
        return false;
    }

    *message = context->receive_queue[context->receive_head];
    context->receive_head = (uint8_t)(
        (context->receive_head + 1u) % AMBAR_HIL_USB_RX_QUEUE_DEPTH);
    --context->receive_count;
    return true;
}

bool AmbarHilUsb_IsConnected(const AmbarHilUsb *context)
{
    return context != NULL && context->initialized && context->connected;
}

AmbarHilUsbStats AmbarHilUsb_GetStats(const AmbarHilUsb *context)
{
    AmbarHilUsbStats stats;
    memset(&stats, 0, sizeof(stats));

    if (context != NULL && context->initialized)
    {
        stats = context->stats;
        stats.connected = context->connected;
        stats.receive_queue_count = (uint8_t)(
            context->receive_count + (context->priority_estop_pending ? 1u : 0u));
        stats.transmit_queue_count = (uint8_t)(
            context->transmit_count + (context->priority_transmit_pending ? 1u : 0u));
    }
    return stats;
}

bool AmbarHilUsb_SendPacket(AmbarHilUsb *context,
                            uint8_t packet_type,
                            const uint8_t *payload,
                            size_t payload_length)
{
    return queue_auto_sequence(context,
                               packet_type,
                               payload,
                               payload_length,
                               false);
}

bool AmbarHilUsb_SendCorrelatedPacket(AmbarHilUsb *context,
                                      uint8_t packet_type,
                                      uint16_t sequence,
                                      const uint8_t *payload,
                                      size_t payload_length)
{
    return queue_frame(context,
                       packet_type,
                       sequence,
                       payload,
                       payload_length,
                       false);
}

bool AmbarHilUsb_SendAck(AmbarHilUsb *context, const AmbarHilUsbAck *payload)
{
    uint8_t encoded[AMBAR_HIL_USB_ACK_PAYLOAD_SIZE];
    const size_t length = encode_ack(encoded, sizeof(encoded), payload);

    if (length == 0u)
    {
        return false;
    }
    return queue_auto_sequence(context,
                               AMBAR_HIL_USB_PACKET_ACK,
                               encoded,
                               length,
                               payload->command == AMBAR_HIL_USB_COMMAND_ESTOP);
}

bool AmbarHilUsb_SendTelemetry(AmbarHilUsb *context,
                               const AmbarHilUsbTelemetry *payload)
{
    uint8_t encoded[AMBAR_HIL_USB_TELEMETRY_PAYLOAD_SIZE];
    const size_t length = encode_telemetry(encoded, sizeof(encoded), payload);
    return length != 0u
        && queue_auto_sequence(context,
                               AMBAR_HIL_USB_PACKET_TELEMETRY,
                               encoded,
                               length,
                               false);
}

bool AmbarHilUsb_SendEvent(AmbarHilUsb *context,
                           const AmbarHilUsbEvent *payload)
{
    uint8_t encoded[AMBAR_HIL_USB_EVENT_PAYLOAD_SIZE];
    const size_t length = encode_event(encoded, sizeof(encoded), payload);
    return length != 0u
        && queue_auto_sequence(context,
                               AMBAR_HIL_USB_PACKET_EVENT,
                               encoded,
                               length,
                               false);
}

bool AmbarHilUsb_SendActuatorStatus(AmbarHilUsb *context,
                                    const AmbarHilUsbActuatorStatus *payload)
{
    uint8_t encoded[AMBAR_HIL_USB_ACTUATOR_PAYLOAD_SIZE];
    const size_t length = encode_actuator(encoded, sizeof(encoded), payload);
    return length != 0u
        && queue_auto_sequence(context,
                               AMBAR_HIL_USB_PACKET_ACTUATOR_STATUS,
                               encoded,
                               length,
                               false);
}

bool AmbarHilUsb_SendHeartbeat(AmbarHilUsb *context,
                               const AmbarHilUsbHeartbeat *payload)
{
    uint8_t encoded[AMBAR_HIL_USB_HEARTBEAT_PAYLOAD_SIZE];
    const size_t length = encode_heartbeat(encoded, sizeof(encoded), payload);
    return length != 0u
        && queue_auto_sequence(context,
                               AMBAR_HIL_USB_PACKET_HEARTBEAT,
                               encoded,
                               length,
                               false);
}

bool AmbarHilUsb_SendLogStatus(
    AmbarHilUsb *context,
    const AmbarHilUsbLogStatus *payload)
{
    uint8_t encoded[AMBAR_HIL_USB_LOG_STATUS_PAYLOAD_SIZE];

    const size_t length =
        encode_log_status(encoded, sizeof(encoded), payload);

    return (length != 0u) &&
           queue_auto_sequence(
               context,
               AMBAR_HIL_USB_PACKET_LOG_STATUS,
               encoded,
               length,
               false);
}

bool AmbarHilUsb_SendVariableHilState(
    AmbarHilUsb *context,
    uint16_t correlated_sequence,
    const AmbarHilUsbVariableHilState *payload)
{
    uint8_t encoded[AMBAR_HIL_USB_VARIABLE_STATE_SIZE];
    size_t length;

    if (payload == NULL || payload->simulation_sequence != correlated_sequence)
    {
        return false;
    }
    length = encode_variable_state(encoded, sizeof(encoded), payload);
    return length != 0u
        && queue_frame(context,
                       AMBAR_HIL_USB_PACKET_VARIABLE_HIL_STATE,
                       correlated_sequence,
                       encoded,
                       length,
                       false);
}

bool AmbarHilUsb_SendVariableHilConfig(
    AmbarHilUsb *context,
    uint16_t correlated_sequence,
    const AmbarHilUsbVariableHilConfig *payload)
{
    uint8_t encoded[AMBAR_HIL_USB_VARIABLE_CONFIG_SIZE];
    const size_t length = encode_variable_config(encoded, sizeof(encoded), payload);
    return length != 0u
        && queue_frame(context,
                       AMBAR_HIL_USB_PACKET_VARIABLE_HIL_CONFIG,
                       correlated_sequence,
                       encoded,
                       length,
                       false);
}

uint8_t AmbarHilUsb_PackSensorHealth(uint8_t imu,
                                     uint8_t barometer,
                                     uint8_t magnetometer,
                                     uint8_t actuator)
{
    return (uint8_t)(((imu & 0x03u) << 0)
        | ((barometer & 0x03u) << 2)
        | ((magnetometer & 0x03u) << 4)
        | ((actuator & 0x03u) << 6));
}
