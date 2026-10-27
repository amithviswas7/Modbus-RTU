

#include "modbus_rtu.h"

/*
 * Calculates the expected NORMAL Modbus RTU response length.
 *
 * Return:
 *   0  = success
 *  -1  = invalid arguments or incomplete request
 *  -2  = invalid request frame
 *  -3  = unsupported function code
 *
 * Note: Exception responses are handled separately by the
 * UART transaction function; an exception response is 5 bytes.
 */
int modbus_expected_response_length(
    const uint8_t *request,
    size_t request_len,
    size_t *expected_len,
    uint16_t *quantity_out)
{
    uint8_t function;
    uint16_t quantity;
    size_t data_bytes;

    if (request == NULL ||
        expected_len == NULL ||
        quantity_out == NULL)
    {
        return -1;
    }

    *expected_len = 0U;
    *quantity_out = 0U;

    if (request_len < 8U)
    {
        return -1;
    }

    if (modbus_validate_request(request, request_len) != 0)
    {
        return -2;
    }

    function = request[1];

    quantity = (uint16_t)(
        ((uint16_t)request[4] << 8) |
        (uint16_t)request[5]);

    switch (function)
    {
        case 0x01U: /* Read Coils */
        case 0x02U: /* Read Discrete Inputs */
            if (quantity == 0U || quantity > 2000U)
            {
                return -2;
            }

            data_bytes = ((size_t)quantity + 7U) / 8U;
            *expected_len = 5U + data_bytes;
            break;

        case 0x03U: /* Read Holding Registers */
        case 0x04U: /* Read Input Registers */
            if (quantity == 0U || quantity > 125U)
            {
                return -2;
            }

            *expected_len = 5U + (2U * (size_t)quantity);
            break;

        case 0x05U: /* Write Single Coil */
        case 0x06U: /* Write Single Register */
            quantity = 1U;
            *expected_len = 8U;
            break;

        case 0x0FU: /* Write Multiple Coils */
            if (quantity == 0U || quantity > 1968U)
            {
                return -2;
            }

            *expected_len = 8U;
            break;

        case 0x10U: /* Write Multiple Registers */
            if (quantity == 0U || quantity > 123U)
            {
                return -2;
            }

            *expected_len = 8U;
            break;

        default:
            return -3;
    }

    *quantity_out = quantity;

    return 0;
}
// It calculates crc bytes and returns crc values
int modbus_crc16(const uint8_t *data,
                 size_t length,
                 uint16_t *crc_out)
{
    uint16_t crc = 0xFFFFU;
    size_t i;
    uint8_t bit;

    if (data == NULL || crc_out == NULL || length == 0U)
    {
        return -1;
    }

    for (i = 0U; i < length; i++)
    {
        crc ^= (uint16_t)data[i];

        for (bit = 0U; bit < 8U; bit++)
        {
            if ((crc & 0x0001U) != 0U)
            {
                crc = (uint16_t)((crc >> 1U) ^ 0xA001U);
            }
            else
            {
                crc >>= 1U;
            }
        }
    }

    *crc_out = crc;

    return 0;
}

// It adds crc bytes at last of the frame

int modbus_append_crc(uint8_t *frame,
                      size_t length,
                      size_t capacity)
{
    uint16_t crc;

    if (frame == NULL || length == 0U)
    {
        return -1;
    }

    /* Reserve two bytes for the CRC. */
    if (length > capacity || (capacity - length) < 2U)
    {
        return -1;
    }

    if (modbus_crc16(frame, length, &crc) != 0)
    {
        return -1;
    }

    /* Modbus RTU sends the low CRC byte first. */
    frame[length] = (uint8_t)(crc & 0x00FFU);
    frame[length + 1U] = (uint8_t)((crc >> 8U) & 0x00FFU);

    return (int)(length + 2U);
}



// It will validate the received frame after comparing with crc
int modbus_validate_crc(const uint8_t *frame,
                        size_t frame_len)
{
    uint16_t calculated_crc;
    uint16_t received_crc;

    if (frame == NULL || frame_len < 4U)
    {
        return -1;
    }

    /*
     * Calculate CRC over all frame bytes except
     * the final two CRC bytes.
     */
    if (modbus_crc16(frame, frame_len - 2U,
                     &calculated_crc) != 0)
    {
        return -1;
    }

    /*
     * Modbus RTU stores CRC low byte first,
     * followed by CRC high byte.
     */
    received_crc = (uint16_t)frame[frame_len - 2U];
    received_crc |= (uint16_t)frame[frame_len - 1U] << 8U;

    if (calculated_crc != received_crc)
    {
        return -2;
    }

    return 0;
}



/* ---------- Internal helpers ---------- */

static int modbus_valid_read_slave(uint8_t slave)
{
    return (slave >= MODBUS_MIN_SLAVE_ADDRESS &&
            slave <= MODBUS_MAX_SLAVE_ADDRESS);
}

static int modbus_valid_write_slave(uint8_t slave)
{
    /* Address 0 is the broadcast address. */
    return (slave <= MODBUS_MAX_SLAVE_ADDRESS);
}

static int modbus_valid_range(uint16_t start, uint16_t quantity)
{
    /*
     * The address space contains 65536 addresses:
     * 0x0000 through 0xFFFF.
     */
    return (quantity != 0U &&
            (uint32_t)quantity <=
                (65536UL - (uint32_t)start));
}

static void modbus_put_u16_be(uint8_t *p, uint16_t value)
{
    p[0] = (uint8_t)(value >> 8U);
    p[1] = (uint8_t)(value & 0x00FFU);
}

static int modbus_finish_request(uint8_t *frame,
                                 size_t length,
                                 size_t capacity)
{
    return modbus_append_crc(frame, length, capacity);
}

static int modbus_build_read_request(
    uint8_t slave,
    uint8_t function,
    uint16_t start,
    uint16_t quantity,
    uint16_t maximum,
    uint8_t *frame,
    size_t capacity)
{
    if (frame == NULL || !modbus_valid_read_slave(slave))
    {
        return -1;
    }

    if (quantity == 0U || quantity > maximum ||
        !modbus_valid_range(start, quantity))
    {
        return -1;
    }

    if (capacity < 8U)
    {
        return -2;
    }

    frame[0] = slave;
    frame[1] = function;
    modbus_put_u16_be(&frame[2], start);
    modbus_put_u16_be(&frame[4], quantity);

    return modbus_finish_request(frame, 6U, capacity);
}

static int modbus_build_single_write(
    uint8_t slave,
    uint8_t function,
    uint16_t address,
    uint16_t value,
    uint8_t *frame,
    size_t capacity)
{
    if (frame == NULL || !modbus_valid_write_slave(slave))
    {
        return -1;
    }

    if (capacity < 8U)
    {
        return -2;
    }

    frame[0] = slave;
    frame[1] = function;
    modbus_put_u16_be(&frame[2], address);
    modbus_put_u16_be(&frame[4], value);

    return modbus_finish_request(frame, 6U, capacity);
}

/* ---------- Function 01: Read Coils ---------- */

int modbus_build_read_coils(
    uint8_t slave, uint16_t start, uint16_t quantity,
    uint8_t *frame, size_t capacity)
{
    return modbus_build_read_request(
        slave, MODBUS_FC_READ_COILS, start, quantity,
        MODBUS_MAX_READ_BITS, frame, capacity);
}

/* ---------- Function 02: Read Discrete Inputs ---------- */

int modbus_build_read_discrete_inputs(
    uint8_t slave, uint16_t start, uint16_t quantity,
    uint8_t *frame, size_t capacity)
{
    return modbus_build_read_request(
        slave, MODBUS_FC_READ_DISCRETE_INPUTS, start, quantity,
        MODBUS_MAX_READ_BITS, frame, capacity);
}

/* ---------- Function 03: Read Holding Registers ---------- */

int modbus_build_read_holding_registers(
    uint8_t slave, uint16_t start, uint16_t quantity,
    uint8_t *frame, size_t capacity)
{
    return modbus_build_read_request(
        slave, MODBUS_FC_READ_HOLDING_REGS, start, quantity,
        MODBUS_MAX_READ_REGISTERS, frame, capacity);
}

/* ---------- Function 04: Read Input Registers ---------- */

int modbus_build_read_input_registers(
    uint8_t slave, uint16_t start, uint16_t quantity,
    uint8_t *frame, size_t capacity)
{
    return modbus_build_read_request(
        slave, MODBUS_FC_READ_INPUT_REGS, start, quantity,
        MODBUS_MAX_READ_REGISTERS, frame, capacity);
}

/* ---------- Function 05: Write Single Coil ---------- */

int modbus_build_write_single_coil(
    uint8_t slave, uint16_t address, uint8_t on,
    uint8_t *frame, size_t capacity)
{
    uint16_t wire_value;

    if (on > 1U)
    {
        return -1;
    }

    /*
     * Standard encoding:
     * ON  = FF 00
     * OFF = 00 00
     */
    wire_value = (on != 0U) ? 0xFF00U : 0x0000U;

    return modbus_build_single_write(
        slave, MODBUS_FC_WRITE_SINGLE_COIL,
        address, wire_value, frame, capacity);
}

/* ---------- Function 06: Write Single Register ---------- */

int modbus_build_write_single_register(
    uint8_t slave, uint16_t address, uint16_t value,
    uint8_t *frame, size_t capacity)
{
    return modbus_build_single_write(
        slave, MODBUS_FC_WRITE_SINGLE_REG,
        address, value, frame, capacity);
}

/* ---------- Function 15 (0x0F): Write Multiple Coils ---------- */

/*
 * packed_coils uses LSB-first packing:
 * bit 0 of packed_coils[0] is the first coil.
 *
 * Example: coils ON, OFF, ON, ON
 * packed byte = 00001101b = 0x0D.
 */
int modbus_build_write_multiple_coils(
    uint8_t slave, uint16_t start, uint16_t quantity,
    const uint8_t *packed_coils, size_t packed_length,
    uint8_t *frame, size_t capacity)
{
    size_t byte_count;
    size_t i;
    size_t expected_length;
    uint8_t unused_bits;
    int result;

    if (frame == NULL || packed_coils == NULL ||
        !modbus_valid_write_slave(slave))
    {
        return -1;
    }

    if (quantity == 0U ||
        quantity > MODBUS_MAX_WRITE_BITS ||
        !modbus_valid_range(start, quantity))
    {
        return -1;
    }

    byte_count = ((size_t)quantity + 7U) / 8U;
    expected_length = 9U + byte_count;

    if (packed_length != byte_count)
    {
        return -1;
    }

    if (capacity < expected_length)
    {
        return -2;
    }

    /*
     * Unused high bits in the last data byte must be zero.
     */
    unused_bits = (uint8_t)(quantity % 8U);

    if (unused_bits != 0U)
    {
        uint8_t mask =
            (uint8_t)(0xFFU << unused_bits);

        if ((packed_coils[byte_count - 1U] & mask) != 0U)
        {
            return -1;
        }
    }

    frame[0] = slave;
    frame[1] = MODBUS_FC_WRITE_MULTIPLE_COILS;
    modbus_put_u16_be(&frame[2], start);
    modbus_put_u16_be(&frame[4], quantity);
    frame[6] = (uint8_t)byte_count;

    for (i = 0U; i < byte_count; i++)
    {
        frame[7U + i] = packed_coils[i];
    }

    result = modbus_finish_request(
        frame, 7U + byte_count, capacity);

    return result;
}

/* ---------- Function 16 (0x10): Write Multiple Registers ---------- */

int modbus_build_write_multiple_registers(
    uint8_t slave, uint16_t start, uint16_t quantity,
    const uint16_t *values,
    uint8_t *frame, size_t capacity)
{
    size_t byte_count;
    size_t i;
    size_t pdu_length;

    if (frame == NULL || values == NULL ||
        !modbus_valid_write_slave(slave))
    {
        return -1;
    }

    if (quantity == 0U ||
        quantity > MODBUS_MAX_WRITE_REGISTERS ||
        !modbus_valid_range(start, quantity))
    {
        return -1;
    }

    byte_count = (size_t)quantity * 2U;
    pdu_length = 7U + byte_count;

    if (capacity < pdu_length + 2U)
    {
        return -2;
    }

    frame[0] = slave;
    frame[1] = MODBUS_FC_WRITE_MULTIPLE_REGS;
    modbus_put_u16_be(&frame[2], start);
    modbus_put_u16_be(&frame[4], quantity);
    frame[6] = (uint8_t)byte_count;

    for (i = 0U; i < quantity; i++)
    {
        modbus_put_u16_be(
            &frame[7U + (i * 2U)], values[i]);
    }

    return modbus_finish_request(
        frame, pdu_length, capacity);
}



int modbus_parse_response(
    const uint8_t *request,
    size_t request_len,
    const uint8_t *response,
    size_t response_len,
    uint16_t *register_values,
    size_t register_capacity,
    uint8_t *bit_values,
    size_t bit_capacity,
    uint8_t *exception_code)
{
    uint8_t slave;
    uint8_t function;
    uint8_t response_function;
    uint16_t quantity;
    size_t expected_bytes;
    size_t expected_length;
    size_t i;
    int rc;

    /*
     * Return codes:
     *   0  = valid normal response
     *   1  = valid Modbus exception response
     *  -1  = invalid request or unsupported function
     *  -2  = response CRC/length validation failure
     *  -3  = slave address or function mismatch
     *  -4  = malformed response
     *  -5  = write echo mismatch
     *  -6  = insufficient output buffer capacity
     *  -7  = broadcast request; no response is expected
     */

    if (exception_code != NULL)
    {
        *exception_code = 0U;
    }

    if (request == NULL || response == NULL)
    {
        return -1;
    }

    /*
     * Validate the complete request before trusting its fields.
     */
    rc = modbus_validate_request(request, request_len);

    if (rc != 0)
    {
        return -1;
    }

    /*
     * Broadcast requests must not be processed as transactions
     * expecting a server response.
     */
    if (request[0] == MODBUS_BROADCAST_ADDRESS)
    {
        return -7;
    }

    if (response_len < 5U || response_len > 256U)
    {
        return -2;
    }

    /*
     * Validate response CRC before interpreting response fields.
     */
    rc = modbus_validate_crc(response, response_len);

    if (rc != 0)
    {
        return -2;
    }

    slave = request[0];
    function = request[1];

    if (response[0] != slave)
    {
        return -3;
    }

    response_function = response[1];

    /*
     * Exception response:
     * response function = request function | 0x80.
     *
     * Standard exception responses contain:
     * slave address, exception function, exception code, CRC.
     */
    if (response_function == (uint8_t)(function | 0x80U))
    {
        if (response_len != 5U)
        {
            return -4;
        }

        if (exception_code != NULL)
        {
            *exception_code = response[2];
        }

        /*
         * Preserve the exception code supplied by the server.
         * Do not silently reinterpret it as a normal response.
         */
        return 1;
    }

    if (response_function != function)
    {
        return -3;
    }

    quantity = (uint16_t)(((uint16_t)request[4] << 8U) |
                          (uint16_t)request[5]);

    switch (function)
    {
        case MODBUS_FC_READ_COILS:
        case MODBUS_FC_READ_DISCRETE_INPUTS:
        {
            expected_bytes = ((size_t)quantity + 7U) / 8U;
            expected_length = 5U + expected_bytes;

            if (response_len != expected_length)
            {
                return -4;
            }

            if ((size_t)response[2] != expected_bytes)
            {
                return -4;
            }

            if (bit_values == NULL ||
                bit_capacity < (size_t)quantity)
            {
                return -6;
            }

            /*
             * Validate zero-padding in the final byte before
             * writing any decoded bits to the output buffer.
             */
            if ((quantity % 8U) != 0U)
            {
                uint8_t used_bits;
                uint8_t unused_mask;
                uint8_t final_data_byte;

                used_bits = (uint8_t)(quantity % 8U);
                unused_mask = (uint8_t)(0xFFU << used_bits);

                final_data_byte =
                    response[3U + expected_bytes - 1U];

                if ((final_data_byte & unused_mask) != 0U)
                {
                    return -4;
                }
            }

            for (i = 0U; i < (size_t)quantity; i++)
            {
                bit_values[i] =
                    (uint8_t)((response[3U + (i / 8U)] >>
                               (i % 8U)) & 0x01U);
            }

            return 0;
        }




        case MODBUS_FC_READ_HOLDING_REGS:
        case MODBUS_FC_READ_INPUT_REGS:
        {
            expected_bytes = (size_t)quantity * 2U;
            expected_length = 5U + expected_bytes;

            if (response_len != expected_length)
            {
                return -4;
            }

            if ((size_t)response[2] != expected_bytes)
            {
                return -4;
            }

            if (register_values == NULL ||
                register_capacity < (size_t)quantity)
            {
                return -6;
            }

            /*
             * Modbus registers are transmitted high byte first.
             */
            for (i = 0U; i < (size_t)quantity; i++)
            {
                register_values[i] =
                    (uint16_t)(
                        ((uint16_t)response[3U + (2U * i)] << 8U) |
                        (uint16_t)response[4U + (2U * i)]);
            }

            return 0;
        }

        case MODBUS_FC_WRITE_SINGLE_COIL:
        case MODBUS_FC_WRITE_SINGLE_REG:
        {
            if (response_len != 8U)
            {
                return -4;
            }

            /*
             * The normal response echoes the request's address
             * and written value.
             */
            for (i = 2U; i <= 5U; i++)
            {
                if (response[i] != request[i])
                {
                    return -5;
                }
            }

            return 0;
        }

        case MODBUS_FC_WRITE_MULTIPLE_COILS:
        case MODBUS_FC_WRITE_MULTIPLE_REGS:
        {
            if (response_len != 8U)
            {
                return -4;
            }

            /*
             * The normal response echoes the starting address
             * and quantity written.
             */
            for (i = 2U; i <= 5U; i++)
            {
                if (response[i] != request[i])
                {
                    return -5;
                }
            }

            return 0;
        }

        default:
        {
            return -1;
        }
    }
}



int modbus_validate_request(const uint8_t *frame,
                            size_t frame_len)
{
    uint8_t slave;
    uint8_t function;
    uint8_t byte_count;
    uint16_t start;
    uint16_t quantity;
    uint16_t value;
    size_t expected_byte_count;
    size_t expected_length;

    /*
     * Return codes:
     *   0  = valid request
     *  -1  = invalid pointer or frame length
     *  -2  = invalid CRC
     *  -3  = invalid slave address
     *  -4  = invalid function or frame structure
     *  -5  = invalid quantity, address range, or data
     */

    if (frame == NULL || frame_len < 8U || frame_len > 256U)
    {
        return -1;
    }

    if (modbus_validate_crc(frame, frame_len) != 0)
    {
        return -2;
    }

    slave = frame[0];
    function = frame[1];

    if (slave > MODBUS_MAX_SLAVE_ADDRESS)
    {
        return -3;
    }

    start = (uint16_t)(((uint16_t)frame[2] << 8U) |
                       (uint16_t)frame[3]);

    quantity = (uint16_t)(((uint16_t)frame[4] << 8U) |
                          (uint16_t)frame[5]);

    switch (function)
    {
        case MODBUS_FC_READ_COILS:
        case MODBUS_FC_READ_DISCRETE_INPUTS:
        {
            if (slave == MODBUS_BROADCAST_ADDRESS ||
                frame_len != 8U)
            {
                return -4;
            }

            if (quantity == 0U ||
                quantity > MODBUS_MAX_READ_BITS ||
                !modbus_valid_range(start, quantity))
            {
                return -5;
            }

            break;
        }

        case MODBUS_FC_READ_HOLDING_REGS:
        case MODBUS_FC_READ_INPUT_REGS:
        {
            if (slave == MODBUS_BROADCAST_ADDRESS ||
                frame_len != 8U)
            {
                return -4;
            }

            if (quantity == 0U ||
                quantity > MODBUS_MAX_READ_REGISTERS ||
                !modbus_valid_range(start, quantity))
            {
                return -5;
            }

            break;
        }

        case MODBUS_FC_WRITE_SINGLE_COIL:
        {
            if (frame_len != 8U)
            {
                return -4;
            }

            value = quantity;

            if (value != 0x0000U && value != 0xFF00U)
            {
                return -5;
            }

            break;
        }

        case MODBUS_FC_WRITE_SINGLE_REG:
        {
            if (frame_len != 8U)
            {
                return -4;
            }

            break;
        }

        case MODBUS_FC_WRITE_MULTIPLE_COILS:
        {
            if (frame_len < 10U)
            {
                return -4;
            }

            if (quantity == 0U ||
                quantity > MODBUS_MAX_WRITE_BITS ||
                !modbus_valid_range(start, quantity))
            {
                return -5;
            }

            byte_count = frame[6];

            expected_byte_count =
                ((size_t)quantity + 7U) / 8U;

            expected_length = 9U + expected_byte_count;

            if ((size_t)byte_count != expected_byte_count ||
                frame_len != expected_length)
            {
                return -4;
            }

            if ((quantity % 8U) != 0U)
            {
                uint8_t used_bits;
                uint8_t unused_mask;
                uint8_t final_data_byte;

                used_bits = (uint8_t)(quantity % 8U);
                unused_mask = (uint8_t)(0xFFU << used_bits);

                final_data_byte =
                    frame[7U + expected_byte_count - 1U];

                if ((final_data_byte & unused_mask) != 0U)
                {
                    return -5;
                }
            }

            break;
        }

        case MODBUS_FC_WRITE_MULTIPLE_REGS:
        {
            if (frame_len < 11U)
            {
                return -4;
            }

            if (quantity == 0U ||
                quantity > MODBUS_MAX_WRITE_REGISTERS ||
                !modbus_valid_range(start, quantity))
            {
                return -5;
            }

            byte_count = frame[6];

            expected_byte_count = (size_t)quantity * 2U;
            expected_length = 9U + expected_byte_count;

            if ((size_t)byte_count != expected_byte_count ||
                frame_len != expected_length)
            {
                return -4;
            }

            break;
        }

        default:
        {
            return -4;
        }
    }

    return 0;
}

