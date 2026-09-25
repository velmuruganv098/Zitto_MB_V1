/*
 * can2_irq.c - Zitto_MB_V1 / S32K144
 *
 * Safety-net FlexCAN2 bus-off / error interrupt handlers (never unmasked).
 * The RX FIFO interrupt (CAN2_ORed_0_15_MB_IRQHandler) lives in can2.c.
 */
#include "can2.h"
#include "S32K144.h"

void CAN2_ORed_IRQHandler(void)
{
    CAN2->ESR1 = (1UL << 1U) | (1UL << 2U);
}

void CAN2_Error_IRQHandler(void)
{
    CAN2->ESR1 = (1UL << 1U) | (1UL << 2U);
}
