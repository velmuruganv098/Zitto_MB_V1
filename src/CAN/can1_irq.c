/*
 * can1_irq.c - Zitto_MB_V1 / S32K144
 *
 * Safety-net FlexCAN1 bus-off / error interrupt handlers. These sources are
 * never unmasked (CTRL1 BOFFMSK/ERRMSK = 0); the handlers only exist so a
 * stray interrupt returns safely instead of landing in DefaultISR.
 * The RX FIFO interrupt (CAN1_ORed_0_15_MB_IRQHandler) lives in can1.c.
 */
#include "can1.h"
#include "S32K144.h"

void CAN1_ORed_IRQHandler(void)
{
    CAN1->ESR1 = CAN1_ESR_BOFFINT_BIT | CAN1_ESR_ERRINT_BIT;
}

void CAN1_Error_IRQHandler(void)
{
    CAN1->ESR1 = CAN1_ESR_BOFFINT_BIT | CAN1_ESR_ERRINT_BIT;
}
