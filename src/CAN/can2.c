/*
 * can2.c  -  Zitto_MB_V1 / S32K144
 *
 * CAN2 = FlexCAN2, PTC16 = RX (ALT3), PTB13 = TX (ALT4), no SHDN pin.
 *
 * V0.0073: receive / auto-baud logic lives in the shared flexcan_drv.c
 * (RX FIFO + IRQ ring buffer, listen-only baud scan, 10 s baud hold).
 * This file only describes the CAN2 hardware and keeps the public Can2_* API.
 * The pre-V0.0073 implementation is kept as can2_legacy_v0063.c.txt.
 */
#include "can2.h"
#include "flexcan_drv.h"
#include "debug_rtt.h"
#include "S32K144.h"
#include "UART/uart_pkt.h"
#include <stddef.h>

static Fcan_t            g_can2;
static Can2_RxCallback_t g_can2_cb = NULL;

static void Can2_PinsInit(void)
{
    PCC->PCCn[PCC_PORTB_INDEX] |= PCC_PCCn_CGC_MASK;
    PCC->PCCn[PCC_PORTC_INDEX] |= PCC_PCCn_CGC_MASK;
    PORTC->PCR[16U] = PORT_PCR_MUX(3U);                 /* CAN2_RX */
    PORTB->PCR[13U] = PORT_PCR_MUX(4U);                 /* CAN2_TX */
}

static const FcanHw_t k_can2_hw =
{
    CAN2, "CAN2", PCC_FlexCAN2_INDEX, (uint8_t)CAN2_ORed_0_15_MB_IRQn, Can2_PinsInit
};

void CAN2_ORed_0_15_MB_IRQHandler(void)
{
    Fcan_Isr(&g_can2);
}

static void Can2_Dispatch(const FcanFrame_t *f, uint32_t baud_kbps)
{
    Can2_Frame_t fr;
    uint8_t i;
    (void)baud_kbps;
    if(g_can2_cb == NULL) { return; }
    fr.id = f->id;
    fr.dlc = f->dlc;
    fr.extended = f->ide;
    fr.rtr = f->rtr;
    for(i = 0U; i < 8U; i++) { fr.data[i] = f->data[i]; }
    g_can2_cb(&fr);
}

void Can2_Init(void)
{
    RTT_LOG("[CAN2] init FlexCAN2 PTC16/PTB13 (RX FIFO+IRQ, listen-only scan, %us hold)\r\n",
            (unsigned)(FCAN_HOLD_MS / 1000U));
    (void)Fcan_Init(&g_can2, &k_can2_hw);
}

void Can2_Task(void)
{
    Fcan_Task(&g_can2, Uart_GetMs(), Can2_Dispatch);
}

void Can2_SetRxCallback(Can2_RxCallback_t callback)
{
    g_can2_cb = callback;
}

void Can2_StartDetection(void)
{
    Fcan_StartDetection(&g_can2);
}

Can2_State_t Can2_GetState(void)
{
    switch(g_can2.state)
    {
        case FCAN_ST_RUNNING: return CAN2_STATE_RUNNING;
        case FCAN_ST_LISTEN:
        case FCAN_ST_CONFIRM: return CAN2_STATE_DETECTING;
        case FCAN_ST_ERROR:   return CAN2_STATE_ERROR;
        default:              return CAN2_STATE_OFF;
    }
}

uint32_t Can2_GetBaudrate(void)
{
    return g_can2.baud_kbps;
}

void Can2_GetStatus(Can2_Status_t *status)
{
    if(status == NULL) { return; }
    status->ready              = (g_can2.state == FCAN_ST_RUNNING) ? 1U : 0U;
    status->detected           = status->ready;
    status->detected_baud_kbps = g_can2.baud_kbps;
    status->rx_count           = g_can2.rx_total;
    status->error_count        = g_can2.err_events;
    status->bus_off_count      = g_can2.busoff_events;
    status->no_frame_count     = Fcan_Dropped(&g_can2);
    status->state              = Can2_GetState();
}

void Can2_Shutdown(void)   { }   /* no SHDN pin on CAN2's transceiver */
void Can2_WakeNormal(void) { }

uint32_t Can2_GetIrqCount(void)      { return g_can2.isr_count; }
uint32_t Can2_GetErrorIrqCount(void) { return Fcan_Dropped(&g_can2); }   /* = frames dropped */
uint32_t Can2_GetMbIrqCount(void)    { return g_can2.isr_count; }
uint32_t Can2_GetRxFps(void)         { return g_can2.rx_fps; }
uint8_t  Can2_GetTec(void)           { return g_can2.tec; }
uint8_t  Can2_GetRec(void)           { return g_can2.rec; }
