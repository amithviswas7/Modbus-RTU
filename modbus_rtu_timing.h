#ifndef MODBUS_RTU_TIMING_H
#define MODBUS_RTU_TIMING_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Modbus RTU serial-line timing recommendations from Guide V1.02. */
#define MODBUS_RTU_TIMING_BAUD_FIXED_THRESHOLD 19200UL
#define MODBUS_RTU_T1_5_FIXED_US                750UL
#define MODBUS_RTU_T3_5_FIXED_US               1750UL
#define MODBUS_RTU_STANDARD_DATA_BITS              8U

typedef enum
{
    MODBUS_UART_PARITY_NONE = 0,
    MODBUS_UART_PARITY_EVEN = 1,
    MODBUS_UART_PARITY_ODD  = 2
} modbus_uart_parity_t;

typedef struct
{
    uint32_t baud_rate;
    uint8_t data_bits;
    modbus_uart_parity_t parity;
    uint8_t stop_bits;
} modbus_uart_config_t;

typedef struct
{
    uint32_t t1_5_us;
    uint32_t t3_5_us;
} modbus_rtu_timing_t;

typedef enum
{
    MODBUS_RTU_GAP_WITHIN_FRAME = 0,
    MODBUS_RTU_GAP_FRAME_INVALID = 1,
    MODBUS_RTU_GAP_FRAME_ENDED = 2
} modbus_rtu_gap_result_t;

/*
 * Calculate RTU inter-character (t1.5) and inter-frame (t3.5) thresholds.
 *
 * Strict Modbus RTU character format is 11 bits:
 *   - 8 data bits, parity enabled, 1 stop bit (8E1 or 8O1); OR
 *   - 8 data bits, no parity, 2 stop bits (8N2).
 *
 * At baud <= 19200, thresholds are rounded UP to whole microseconds:
 *   t1.5 = ceil(1.5 * 11 * 1,000,000 / baud)
 *   t3.5 = ceil(3.5 * 11 * 1,000,000 / baud)
 * At baud > 19200, the guide recommends fixed thresholds of 750 us and
 * 1750 us respectively.
 *
 * Returns:
 *   0  success
 *  -1  null pointer or zero baud rate
 *  -2  UART format is not a supported 11-bit Modbus RTU character format
 */
int modbus_rtu_calculate_timing(const modbus_uart_config_t *uart,
                                modbus_rtu_timing_t *timing);

/*
 * Classify the measured silence between two received characters, or between
 * the last received character and the current time.
 *
 * <= t1.5       : gap is acceptable within a frame
 * > t1.5 < t3.5 : current frame is incomplete/invalid
 * >= t3.5       : frame boundary / end-of-frame detected
 *
 * The caller must still maintain receive state: after an invalid internal
 * gap, discard bytes until a full t3.5 silence re-establishes the idle state.
 */
modbus_rtu_gap_result_t modbus_rtu_classify_gap(
    const modbus_rtu_timing_t *timing,
    uint32_t gap_us);

#ifdef __cplusplus
}
#endif

#endif /* MODBUS_RTU_TIMING_H */
