

#ifndef MODBUS_RTU_H
#define MODBUS_RTU_H

#include <stdint.h>
#include <stddef.h>

/* Standard Modbus limits */
#define MODBUS_MIN_SLAVE_ADDRESS       1U
#define MODBUS_MAX_SLAVE_ADDRESS       247U
#define MODBUS_BROADCAST_ADDRESS        0U

#define MODBUS_MAX_READ_BITS            2000U
#define MODBUS_MAX_WRITE_BITS           1968U
#define MODBUS_MAX_READ_REGISTERS       125U
#define MODBUS_MAX_WRITE_REGISTERS      123U

/* Function codes */
#define MODBUS_FC_READ_COILS            0x01U
#define MODBUS_FC_READ_DISCRETE_INPUTS  0x02U
#define MODBUS_FC_READ_HOLDING_REGS     0x03U
#define MODBUS_FC_READ_INPUT_REGS       0x04U
#define MODBUS_FC_WRITE_SINGLE_COIL     0x05U
#define MODBUS_FC_WRITE_SINGLE_REG      0x06U
#define MODBUS_FC_WRITE_MULTIPLE_COILS  0x0FU
#define MODBUS_FC_WRITE_MULTIPLE_REGS   0x10U
/*
 * Calculate CRC-16/MODBUS.
 *
 * Parameters:
 *   data     - Pointer to input bytes
 *   length   - Number of bytes to process
 *   crc_out  - Pointer to receive the calculated CRC
 *
 * Returns:
 *    0  = Success
 *   -1  = Invalid argument
 */
int modbus_crc16(const uint8_t *data,size_t length, uint16_t *crc_out);


/*
 * Append CRC-16/MODBUS to a frame.
 *
 * frame     : Buffer containing the frame and room for 2 CRC bytes.
 * length    : Number of frame bytes before the CRC.
 * capacity  : Total size of the frame buffer.
 *
 * Returns:
 *   >= 0 : Total frame length including CRC
 *   -1   : Invalid argument or insufficient capacity
 */
int modbus_append_crc(uint8_t *frame,
                      size_t length,
                      size_t capacity);


/*
 * Validate CRC-16/MODBUS in a complete RTU frame.
 *
 * frame      : Complete received frame, including its CRC.
 * frame_len  : Total number of bytes received.
 *
 * Returns:
 *    0  = CRC valid
 *   -1  = Invalid argument or frame too short
 *   -2  = CRC mismatch
 */
int modbus_validate_crc(const uint8_t *frame, size_t frame_len);

int modbus_build_read_coils(uint8_t slave, uint16_t start, uint16_t quantity, uint8_t *frame, size_t capacity);

int modbus_build_read_discrete_inputs( uint8_t slave, uint16_t start, uint16_t quantity,uint8_t *frame, size_t capacity);

int modbus_build_read_holding_registers( uint8_t slave, uint16_t start, uint16_t quantity, uint8_t *frame, size_t capacity);

int modbus_build_read_input_registers( uint8_t slave, uint16_t start, uint16_t quantity, uint8_t *frame, size_t capacity);

int modbus_build_write_single_coil( uint8_t slave, uint16_t address, uint8_t on, uint8_t *frame, size_t capacity);

int modbus_build_write_single_register( uint8_t slave, uint16_t address, uint16_t value, uint8_t *frame, size_t capacity);

int modbus_build_write_multiple_coils( uint8_t slave, uint16_t start, uint16_t quantity,const uint8_t *packed_coils, size_t packed_length,uint8_t *frame, size_t capacity);

int modbus_build_write_multiple_registers( uint8_t slave, uint16_t start, uint16_t quantity, const uint16_t *values, uint8_t *frame, size_t capacity);

int modbus_expected_response_length(
    const uint8_t *request,
    size_t request_len,
    size_t *expected_len,
    uint16_t *quantity_out);
/*
 * Validate and parse a response against its original request.
 *
 * request/request_len:
 *   Complete request frame, including CRC.
 *
 * response/response_len:
 *   Complete received response frame, including CRC.
 *
 * register_values:
 *   Output array for FC 03 and FC 04; one uint16_t per register.
 *
 * bit_values:
 *   Output array for FC 01 and FC 02; one uint8_t per bit,
 *   where 0 = OFF and 1 = ON.
 *
 * exception_code:
 *   Receives the Modbus exception code if an exception is returned.
 *
 * Returns:
 *    0  = Valid response
 *    1  = Valid Modbus exception response
 *   -1  = Invalid arguments or request
 *   -2  = Invalid response CRC
 *   -3  = Unexpected slave/function
 *   -4  = Invalid response length or byte count
 *   -5  = Write response echo mismatch
 *   -6  = Insufficient output capacity
 */
int modbus_parse_response(
    const uint8_t *request,
    size_t request_len,
    const uint8_t *response,
    size_t response_len,
    uint16_t *register_values,
    size_t register_capacity,
    uint8_t *bit_values,
    size_t bit_capacity,
    uint8_t *exception_code);


int modbus_validate_request(const uint8_t *frame,
                            size_t frame_len);
int modbus_rtu_run_tests(void);
#endif /* MODBUS_RTU_H */

