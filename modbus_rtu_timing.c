#include "modbus_rtu_timing.h"

#define MODBUS_RTU_BITS_PER_CHARACTER 11ULL
#define MICROSECONDS_PER_SECOND       1000000ULL
#define T1_5_HALF_MULTIPLIER          3ULL
#define T1_5_DIVISOR                  2ULL
#define T3_5_HALF_MULTIPLIER          7ULL
#define T3_5_DIVISOR                  2ULL

/* Ceiling division without floating-point arithmetic. */
static uint32_t modbus_ceil_div_u64(uint64_t numerator, uint64_t denominator)
{
    return (uint32_t)((numerator + denominator - 1ULL) / denominator);
}

static int modbus_uart_format_is_rtu_11_bit(const modbus_uart_config_t *uart)
{
    if (uart->data_bits != MODBUS_RTU_STANDARD_DATA_BITS)
    {
        return 0;
    }

    if ((uart->parity == MODBUS_UART_PARITY_EVEN) ||
        (uart->parity == MODBUS_UART_PARITY_ODD))
    {
        return (uart->stop_bits == 1U);
    }

    if (uart->parity == MODBUS_UART_PARITY_NONE)
    {
        return (uart->stop_bits == 2U);
    }

    return 0;
}

int modbus_rtu_calculate_timing(const modbus_uart_config_t *uart,
                                modbus_rtu_timing_t *timing)
{
    uint64_t numerator_t1_5;
    uint64_t numerator_t3_5;

    if ((uart == 0) || (timing == 0) || (uart->baud_rate == 0UL))
    {
        return -1;
    }

    if (!modbus_uart_format_is_rtu_11_bit(uart))
    {
        return -2;
    }

    if (uart->baud_rate > MODBUS_RTU_TIMING_BAUD_FIXED_THRESHOLD)
    {
        timing->t1_5_us = MODBUS_RTU_T1_5_FIXED_US;
        timing->t3_5_us = MODBUS_RTU_T3_5_FIXED_US;
        return 0;
    }

    /*
     * Use 64-bit integer arithmetic and ceiling division. Rounding upward
     * avoids declaring a gap complete earlier than the calculated threshold.
     */
    numerator_t1_5 = T1_5_HALF_MULTIPLIER *
                     MODBUS_RTU_BITS_PER_CHARACTER *
                     MICROSECONDS_PER_SECOND;
    numerator_t3_5 = T3_5_HALF_MULTIPLIER *
                     MODBUS_RTU_BITS_PER_CHARACTER *
                     MICROSECONDS_PER_SECOND;

    timing->t1_5_us = modbus_ceil_div_u64(
        numerator_t1_5, T1_5_DIVISOR * (uint64_t)uart->baud_rate);
    timing->t3_5_us = modbus_ceil_div_u64(
        numerator_t3_5, T3_5_DIVISOR * (uint64_t)uart->baud_rate);

    return 0;
}

modbus_rtu_gap_result_t modbus_rtu_classify_gap(
    const modbus_rtu_timing_t *timing,
    uint32_t gap_us)
{
    if (timing == 0)
    {
        /* Invalid timing configuration: fail closed, never accept the gap. */
        return MODBUS_RTU_GAP_FRAME_INVALID;
    }

    /* Check t3.5 first because it is the frame boundary condition. */
    if (gap_us >= timing->t3_5_us)
    {
        return MODBUS_RTU_GAP_FRAME_ENDED;
    }

    if (gap_us > timing->t1_5_us)
    {
        return MODBUS_RTU_GAP_FRAME_INVALID;
    }

    return MODBUS_RTU_GAP_WITHIN_FRAME;
}
