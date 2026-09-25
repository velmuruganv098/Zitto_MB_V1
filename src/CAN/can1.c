/*
 * can1.c  -  Zitto_MB_V1 / S32K144
 *
 * CAN1 = FlexCAN1, PTA12 = RX (ALT3), PTA13 = TX (ALT3), TCAN334 SHDN = PTB2.
 *
 * V0.0073: the receive / auto-baud logic now lives in the shared
 * flexcan_drv.c (RX FIFO + IRQ ring buffer, listen-only baud scan, 10 s baud
 * hold).  This file only describes the CAN1 hardware and keeps the public
 * Can1_* API used by main.c unchanged.
 * The pre-V0.0073 implementation is kept as can1_legacy_v0063.c.txt.
 */
#include "can1.h"
#include "flexcan_drv.h"
#include "debug_rtt.h"
#include "S32K144.h"
#include "UART/uart_pkt.h"
#include <stddef.h>

static Fcan_t            g_can1;
static Can1_RxCallback_t g_can1_cb = NULL;

static void Can1_PinsInit(void)
{
    PCC->PCCn[PCC_PORTA_INDEX] |= PCC_PCCn_CGC_MASK;
    PCC->PCCn[PCC_PORTB_INDEX] |= PCC_PCCn_CGC_MASK;
    PORTB->PCR[CAN1_SHDN_PTB_PIN] = PORT_PCR_MUX(1U);   /* SHDN: LOW = normal */
    PTB->PDDR |= (1UL << CAN1_SHDN_PTB_PIN);
    PTB->PCOR  = (1UL << CAN1_SHDN_PTB_PIN);
    PORTA->PCR[12U] = PORT_PCR_MUX(3U);                 /* CAN1_RX */
    PORTA->PCR[13U] = PORT_PCR_MUX(3U);                 /* CAN1_TX */
}

static const FcanHw_t k_can1_hw =
{
    CAN1, "CAN1", PCC_FlexCAN1_INDEX, (uint8_t)CAN1_ORed_0_15_MB_IRQn, Can1_PinsInit
};

void CAN1_ORed_0_15_MB_IRQHandler(void)
{
    Fcan_Isr(&g_can1);
}

static void Can1_Dispatch(const FcanFrame_t *f, uint32_t baud_kbps)
{
    if(g_can1_cb != NULL)
    {
        g_can1_cb(f->id, f->ide, f->rtr, f->dlc, f->data, baud_kbps);
    }
}

void Can1_Init(void)
{
    RTT_LOG("[CAN1] init FlexCAN1 PTA12/PTA13 SHDN=PTB%u (RX FIFO+IRQ, listen-only scan, %us hold)\r\n",
            (unsigned)CAN1_SHDN_PTB_PIN, (unsigned)(FCAN_HOLD_MS / 1000U));
    (void)Fcan_Init(&g_can1, &k_can1_hw);
}

void Can1_Task(void)
{
    Fcan_Task(&g_can1, Uart_GetMs(), Can1_Dispatch);
}

void Can1_SetRxCallback(Can1_RxCallback_t callback)
{
    g_can1_cb = callback;
}

void Can1_StartDetection(void)
{
    Fcan_StartDetection(&g_can1);
}

Can1_State_t Can1_GetState(void)
{
    if(g_can1.state == FCAN_ST_RUNNING) { return CAN1_STATE_READY; }
    if((g_can1.state == FCAN_ST_ERROR) || (g_can1.state == FCAN_ST_OFF)) { return CAN1_STATE_ERROR; }
    return CAN1_STATE_DETECTING;
}

uint32_t Can1_GetBaudrate(void)
{
    return g_can1.baud_kbps;
}

uint8_t Can1_IsReady(void)
{
    return (g_can1.state == FCAN_ST_RUNNING) ? 1U : 0U;
}

void Can1_GetStatus(Can1_Status_t *out)
{
    if(out == NULL) { return; }
    out->ready              = Can1_IsReady();
    out->hw_ready           = g_can1.hw_ok;
    out->detecting          = ((g_can1.state == FCAN_ST_LISTEN) || (g_can1.state == FCAN_ST_CONFIRM)) ? 1U : 0U;
    out->rx_active          = (g_can1.rx_fps != 0U) ? 1U : 0U;
    out->bus_idle           = (uint8_t)((CAN1->ESR1 >> 7U) & 1U);
    out->bus_off            = (g_can1.fault >= 2U) ? 1U : 0U;
    out->error_passive      = (g_can1.fault == 1U) ? 1U : 0U;
    out->shdn_state         = (uint8_t)((PTB->PDOR >> CAN1_SHDN_PTB_PIN) & 1U);
    out->detected_baud_kbps = g_can1.baud_kbps;
    out->rx_count           = g_can1.rx_total;
    out->frames_rcvd        = g_can1.rx_total;
    out->tx_err_cnt         = g_can1.tec;
    out->rx_err_cnt         = g_can1.rec;
    out->error_count        = g_can1.err_events;
}

void Can1_Shutdown(void)
{
    PTB->PSOR = (1UL << CAN1_SHDN_PTB_PIN);
    RTT_LOG("[CAN1] transceiver shutdown\r\n");
}

void Can1_WakeNormal(void)
{
    PTB->PCOR = (1UL << CAN1_SHDN_PTB_PIN);
    RTT_LOG("[CAN1] transceiver normal\r\n");
}

/* Diagnostics used by main.c's 1 s status line / MSG_CAN_STATUS. */
uint32_t Can1_GetIrqCount(void)      { return g_can1.isr_count; }
uint32_t Can1_GetErrorIrqCount(void) { return Fcan_Dropped(&g_can1); }   /* = frames dropped */
uint32_t Can1_GetMbIrqCount(void)    { return g_can1.isr_count; }
uint32_t Can1_GetRxFps(void)         { return g_can1.rx_fps; }
