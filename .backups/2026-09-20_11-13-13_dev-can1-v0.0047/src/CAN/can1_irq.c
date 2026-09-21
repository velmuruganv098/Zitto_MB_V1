/*
 * can1_irq.c - Zitto_MB_V1 / S32K144
 *
 * Safety-net FlexCAN1 interrupt handlers.
 *
 * CAN1 runtime service is intentionally polling-based so CAN reception never
 * depends on a slow ISR/debug path. These handlers therefore do not forward
 * frames, do not call UART/RTT, and do not change CAN bit timing.
 *
 * They exist because the startup vector table aliases unregistered CAN
 * interrupts to DefaultISR. An accidental/stale interrupt must return safely.
 */

#include "can1.h"
#include "S32K144.h"
#include <stdint.h>

volatile uint32_t g_can1_or_irq_count    = 0U;
volatile uint32_t g_can1_error_irq_count = 0U;
volatile uint32_t g_can1_mb_irq_count    = 0U;

void CAN1_ORed_IRQHandler(void)
{
    uint32_t esr = CAN1->ESR1;
    uint32_t ifl = CAN1->IFLAG1;

    g_can1_or_irq_count++;

    /* Error/status condition bits clear on read. Interrupt sources are W1C. */
    (void)esr;
    CAN1->ESR1 = CAN1_ESR_BOFFINT_BIT | CAN1_ESR_ERRINT_BIT;
    CAN1->IFLAG1 = ifl;
}

void CAN1_Error_IRQHandler(void)
{
    uint32_t esr = CAN1->ESR1;

    g_can1_error_irq_count++;

    (void)esr;
    CAN1->ESR1 = CAN1_ESR_BOFFINT_BIT | CAN1_ESR_ERRINT_BIT;
}

void CAN1_ORed_0_15_MB_IRQHandler(void)
{
    uint32_t ifl = CAN1->IFLAG1;

    g_can1_mb_irq_count++;

    CAN1->IFLAG1 = ifl;
}

uint32_t Can1_GetIrqCount(void)
{
    return g_can1_or_irq_count;
}

uint32_t Can1_GetErrorIrqCount(void)
{
    return g_can1_error_irq_count;
}

uint32_t Can1_GetMbIrqCount(void)
{
    return g_can1_mb_irq_count;
}
