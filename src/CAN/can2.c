#include "can2.h"

#include "S32K144.h"

#include "../DEBUG/debug_rtt.h"


/* ========================================================================== */
/* USER HARDWARE CONFIGURATION                                                */
/* ========================================================================== */


/*
 * CAN2 = FlexCAN0
 *
 * IMPORTANT:
 *
 * Verify these pins against your schematic.
 *
 * Change only these definitions if necessary.
 */


/*
 * Example FlexCAN0 pin mapping.
 *
 * Replace with your actual CAN2 pins.
 */

#define CAN2_RX_PORT                PORTB
#define CAN2_TX_PORT                PORTB

#define CAN2_RX_PIN                 0U
#define CAN2_TX_PIN                 1U


/*
 * FlexCAN alternate function.
 *
 * Verify against S32K144 pin mux.
 */

#define CAN2_PIN_MUX                3U


/*
 * Transceiver shutdown pin.
 *
 * CHANGE according to schematic.
 *
 * If CAN2 transceiver does not have shutdown control,
 * this can be adapted.
 */

#define CAN2_SHDN_PORT              PTB
#define CAN2_SHDN_PCR_PORT          PORTB
#define CAN2_SHDN_PIN               5U


/* ========================================================================== */
/* FLEXCAN CONFIGURATION                                                      */
/* ========================================================================== */


#define CAN2_MB_RX                  4U

#define CAN2_RX_MB_FLAG             \
    (1UL << CAN2_MB_RX)


#define CAN2_CODE_RX_EMPTY          0x04U

#define CAN2_CODE_RX_FULL           0x02U


#define CAN2_CS_RX_EMPTY            \
    ((uint32_t)CAN2_CODE_RX_EMPTY << 24U)


#define CAN2_CS_CODE_MASK           \
    (0x0FUL << 24U)


#define CAN2_CS_DLC_MASK            \
    (0x0FUL << 16U)


#define CAN2_CS_RTR_MASK            \
    (1UL << 20U)


#define CAN2_CS_IDE_MASK            \
    (1UL << 21U)


/*
 * FlexCAN uses 16 message buffers.
 */

#define CAN2_MCR_MAXMB              0x0FU


/* ========================================================================== */
/* DETECTION CONFIGURATION                                                    */
/* ========================================================================== */


#define CAN2_AUTO_BAUD_COUNT        4U


/*
 * Candidate order.
 */

static const uint32_t g_can2_baud_kbps[
    CAN2_AUTO_BAUD_COUNT
] =
{
    500U,

    250U,

    125U,

    1000U
};


/*
 * Detection timing.
 *
 * Can2_Task() is expected every 10ms.
 *
 * 20 ticks = approximately 200ms.
 */

#define CAN2_AUTO_BAUD_TICKS        20U


/*
 * Consecutive error-free frames required at a candidate baud
 * before it is trusted and locked in. A single frame is not
 * enough: candidates in this table are exact 2x multiples of
 * each other (1000/500/250/125), and a receiver listening at
 * half the real bus rate can occasionally reconstruct what
 * looks like one short, CRC-valid frame out of real traffic.
 * See CAN2_ERR_FLAGS_MASK below.
 */

#define CAN2_CONFIRM_FRAMES         3U


/*
 * Real CAN protocol errors (not counter-overflow warnings) that
 * prove the current candidate baud does NOT match the bus. LOM
 * never transmits, so ACKERR is not meaningful here.
 */

#define CAN2_ERR_FLAGS_MASK \
    (CAN_ESR1_STFERR_MASK | CAN_ESR1_FRMERR_MASK | \
     CAN_ESR1_CRCERR_MASK | CAN_ESR1_BIT0ERR_MASK | \
     CAN_ESR1_BIT1ERR_MASK)


/*
 * Number of consecutive detection cycles
 * before we simply keep cycling.
 *
 * No blocking.
 */

#define CAN2_NO_FRAME_LIMIT         200U


/*
 * Register operation timeout.
 */

#define CAN2_HW_TIMEOUT             200000U


/* ========================================================================== */
/* CAN BIT TIMING                                                             */
/* ========================================================================== */


/*
 * These CTRL1 values must match your CAN clock configuration.
 *
 * They are kept in one table so they can easily be adjusted.
 *
 * FIX (V0.0063): the previous entries here assumed a TQ/PRESDIV
 * combination that did not match ANY clock this module is actually
 * fed with (8MHz SOSC or 40MHz bus clock) - none of the four labeled
 * rates were the real bit rate produced. On top of that,
 * Can2_HardwareInit() cleared CLKSRC (selecting the 8MHz oscillator)
 * while its own comment said "keep same approach as CAN1", which
 * selects the 40MHz bus clock (CLKSRC=1). Both are fixed together:
 * CLKSRC is now set to the bus clock in Can2_HardwareInit(), and the
 * values below are the same, independently-verified 40MHz/16TQ
 * (SP=81.25%) and 40MHz/8TQ (SP=75%) timing used by CAN1.
 */


static const uint32_t g_can2_ctrl1_normal[
    CAN2_AUTO_BAUD_COUNT
] =
{
    /*
     * 500 kbps  (40MHz / 5 / 16TQ)
     */
    CAN_CTRL1_PROPSEG(7U) |
    CAN_CTRL1_PSEG1(3U) |
    CAN_CTRL1_PSEG2(2U) |
    CAN_CTRL1_RJW(1U) |
    CAN_CTRL1_PRESDIV(4U),

    /*
     * 250 kbps  (40MHz / 10 / 16TQ)
     */
    CAN_CTRL1_PROPSEG(7U) |
    CAN_CTRL1_PSEG1(3U) |
    CAN_CTRL1_PSEG2(2U) |
    CAN_CTRL1_RJW(1U) |
    CAN_CTRL1_PRESDIV(9U),

    /*
     * 125 kbps  (40MHz / 20 / 16TQ)
     */
    CAN_CTRL1_PROPSEG(7U) |
    CAN_CTRL1_PSEG1(3U) |
    CAN_CTRL1_PSEG2(2U) |
    CAN_CTRL1_RJW(1U) |
    CAN_CTRL1_PRESDIV(19U),

    /*
     * 1000 kbps  (40MHz / 5 / 8TQ)
     */
    CAN_CTRL1_PROPSEG(2U) |
    CAN_CTRL1_PSEG1(1U) |
    CAN_CTRL1_PSEG2(1U) |
    CAN_CTRL1_RJW(1U) |
    CAN_CTRL1_PRESDIV(4U)
};


/*
 * Listen-only versions.
 */

static uint32_t Can2_GetListenOnlyCtrl1(
    uint8_t index
)
{
    return
        g_can2_ctrl1_normal[index] |
        CAN_CTRL1_LOM_MASK;
}


/* ========================================================================== */
/* DRIVER VARIABLES                                                           */
/* ========================================================================== */


static Can2_Status_t
    g_can2_status;


static Can2_RxCallback_t
    g_can2_rx_callback;


static uint8_t
    g_can2_baud_index;


static uint32_t
    g_can2_detect_tick;


static uint32_t
    g_can2_no_frame_counter;


static uint8_t
    g_can2_confirm_count;


/* ========================================================================== */
/* DELAY                                                                       */
/* ========================================================================== */


static void Can2_DelayMs(
    uint32_t ms
)
{
    volatile uint32_t i;

    volatile uint32_t count;

    while(ms != 0U)
    {
        count = 8000U;

        for(i = 0U;
            i < count;
            i++)
        {
            __asm volatile(
                "nop"
            );
        }

        ms--;
    }
}


/* ========================================================================== */
/* TRANSCEIVER CONTROL                                                        */
/* ========================================================================== */


static void Can2_ShutdownPinInit(void)
{
    PCC->PCCn[
        PCC_PORTB_INDEX
    ] |= PCC_PCCn_CGC_MASK;


    CAN2_SHDN_PCR_PORT->PCR[
        CAN2_SHDN_PIN
    ] =
        PORT_PCR_MUX(1U);


    CAN2_SHDN_PORT->PDDR |=
        (1UL << CAN2_SHDN_PIN);


    /*
     * Default transceiver enabled.
     */

    CAN2_SHDN_PORT->PCOR =
        (1UL << CAN2_SHDN_PIN);
}


/*
 * Normal mode.
 */

void Can2_WakeNormal(void)
{
    CAN2_SHDN_PORT->PCOR =
        (1UL << CAN2_SHDN_PIN);


    Can2_DelayMs(1U);


    RTT_LOG(
        "[CAN2] Transceiver normal\r\n"
    );
}


/*
 * Shutdown.
 */

void Can2_Shutdown(void)
{
    CAN2_SHDN_PORT->PSOR =
        (1UL << CAN2_SHDN_PIN);


    g_can2_status.ready =
        0U;


    g_can2_status.state =
        CAN2_STATE_OFF;


    RTT_LOG(
        "[CAN2] Transceiver shutdown\r\n"
    );
}


/* ========================================================================== */
/* FREEZE MODE                                                                */
/* ========================================================================== */


static uint8_t Can2_EnterFreeze(void)
{
    volatile uint32_t timeout =
        CAN2_HW_TIMEOUT;


    CAN0->MCR |=
        CAN_MCR_FRZ_MASK |
        CAN_MCR_HALT_MASK;


    while(
        ((CAN0->MCR &
          CAN_MCR_FRZACK_MASK) == 0U)
        &&
        (timeout-- != 0U)
    )
    {
    }


    if(timeout == 0U)
    {
        RTT_LOG(
            "[CAN2_ERR] Freeze timeout MCR=0x%08lX\r\n",
            (unsigned long)CAN0->MCR
        );

        return 0U;
    }


    return 1U;
}


static uint8_t Can2_ExitFreeze(void)
{
    volatile uint32_t timeout =
        CAN2_HW_TIMEOUT;


    CAN0->MCR &=
        ~CAN_MCR_HALT_MASK;


    while(
        ((CAN0->MCR &
          CAN_MCR_FRZACK_MASK) != 0U)
        &&
        (timeout-- != 0U)
    )
    {
    }


    if(timeout == 0U)
    {
        RTT_LOG(
            "[CAN2_ERR] Exit freeze timeout MCR=0x%08lX\r\n",
            (unsigned long)CAN0->MCR
        );

        return 0U;
    }


    return 1U;
}


/* ========================================================================== */
/* MAILBOX                                                                    */
/* ========================================================================== */


static void Can2_SetRxMailbox(void)
{
    uint32_t base;


    base =
        CAN2_MB_RX * 4U;


    CAN0->RAMn[
        base + 0U
    ] =
        0U;


    CAN0->RAMn[
        base + 1U
    ] =
        0U;


    CAN0->RAMn[
        base + 2U
    ] =
        0U;


    CAN0->RAMn[
        base + 3U
    ] =
        0U;


    CAN0->RAMn[
        base + 0U
    ] =
        CAN2_CS_RX_EMPTY;
}


/* ========================================================================== */
/* BIT TIMING                                                                 */
/* ========================================================================== */


static uint8_t Can2_SetBaud(
    uint8_t index,
    uint8_t listen_only
)
{
    uint32_t ctrl1;


    if(index >=
       CAN2_AUTO_BAUD_COUNT)
    {
        return 0U;
    }


    if(Can2_EnterFreeze() == 0U)
    {
        return 0U;
    }


    if(listen_only != 0U)
    {
        ctrl1 =
            Can2_GetListenOnlyCtrl1(
                index
            );
    }
    else
    {
        ctrl1 =
            g_can2_ctrl1_normal[
                index
            ];
    }


    CAN0->CTRL1 =
        ctrl1;


    /*
     * Re-arm mailbox.
     */

    Can2_SetRxMailbox();


    /*
     * Clear RX mailbox flag.
     */

    CAN0->IFLAG1 =
        CAN2_RX_MB_FLAG;


    if(Can2_ExitFreeze() == 0U)
    {
        return 0U;
    }


    return 1U;
}


/* ========================================================================== */
/* HARDWARE INITIALIZATION                                                    */
/* ========================================================================== */


static uint8_t Can2_HardwareInit(void)
{
    volatile uint32_t timeout;


    RTT_LOG(
        "[CAN2_HW] Start\r\n"
    );


    /*
     * Enable PORTB clock.
     */

    PCC->PCCn[
        PCC_PORTB_INDEX
    ] |=
        PCC_PCCn_CGC_MASK;


    /*
     * CAN RX pin.
     */

    CAN2_RX_PORT->PCR[
        CAN2_RX_PIN
    ] =
        PORT_PCR_MUX(
            CAN2_PIN_MUX
        );


    /*
     * CAN TX pin.
     */

    CAN2_TX_PORT->PCR[
        CAN2_TX_PIN
    ] =
        PORT_PCR_MUX(
            CAN2_PIN_MUX
        );


    /*
     * Enable FlexCAN0 clock.
     */

    PCC->PCCn[
        PCC_FlexCAN0_INDEX
    ] |=
        PCC_PCCn_CGC_MASK;


    /*
     * Disable module.
     */

    CAN0->MCR =
        CAN_MCR_MDIS_MASK;


    /*
     * Select peripheral (bus) clock, CLKSRC=1 - 40MHz.
     *
     * Same approach as CAN1. FIX (V0.0063): this used to clear
     * CLKSRC instead of setting it, leaving FlexCAN0 clocked from
     * the 8MHz SOSC while the bit-timing table assumed 40MHz - none
     * of the configured baud rates were actually correct.
     */

    CAN0->CTRL1 |=
        CAN_CTRL1_CLKSRC_MASK;


    /*
     * Enable module.
     */

    CAN0->MCR &=
        ~CAN_MCR_MDIS_MASK;


    timeout =
        CAN2_HW_TIMEOUT;


    while(
        ((CAN0->MCR &
          CAN_MCR_LPMACK_MASK) != 0U)
        &&
        (timeout-- != 0U)
    )
    {
    }


    if(timeout == 0U)
    {
        RTT_LOG(
            "[CAN2_ERR] LPMACK timeout MCR=0x%08lX\r\n",
            (unsigned long)CAN0->MCR
        );

        return 0U;
    }


    /*
     * Software reset.
     */

    CAN0->MCR |=
        CAN_MCR_SOFTRST_MASK;


    timeout =
        CAN2_HW_TIMEOUT;


    while(
        ((CAN0->MCR &
          CAN_MCR_SOFTRST_MASK) != 0U)
        &&
        (timeout-- != 0U)
    )
    {
    }


    if(timeout == 0U)
    {
        RTT_LOG(
            "[CAN2_ERR] Soft reset timeout\r\n"
        );

        return 0U;
    }


    /*
     * Configure FlexCAN.
     *
     * IRMQ = individual masking
     * FRZ/HALT = configuration allowed
     */

    CAN0->MCR =
        CAN_MCR_FRZ_MASK |
        CAN_MCR_HALT_MASK |
        CAN_MCR_IRMQ_MASK |
        CAN2_MCR_MAXMB;


    /*
     * Disable all interrupts.
     *
     * Driver is polling-based.
     */

    CAN0->IMASK1 =
        0U;

    /*
     * Clear pending flags.
     */

    CAN0->IFLAG1 =
        0xFFFFFFFFUL;


    RTT_LOG(
        "[CAN2_HW] MCR=0x%08lX CTRL1=0x%08lX\r\n",
        (unsigned long)CAN0->MCR,
        (unsigned long)CAN0->CTRL1
    );


    return 1U;
}


/* ========================================================================== */
/* RX FRAME                                                                   */
/* ========================================================================== */


static uint8_t Can2_ReadFrame(
    Can2_Frame_t *frame
)
{
    uint32_t base;

    uint32_t cs;

    uint32_t id_word;

    uint32_t data0;

    uint32_t data1;

    uint8_t dlc;


    if(
        (CAN0->IFLAG1 &
         CAN2_RX_MB_FLAG) == 0U
    )
    {
        return 0U;
    }


    base =
        CAN2_MB_RX * 4U;


    cs =
        CAN0->RAMn[
            base + 0U
        ];


    id_word =
        CAN0->RAMn[
            base + 1U
        ];


    data0 =
        CAN0->RAMn[
            base + 2U
        ];


    data1 =
        CAN0->RAMn[
            base + 3U
        ];


    dlc =
        (uint8_t)(
            (cs &
             CAN2_CS_DLC_MASK)
            >>
            16U
        );


    if(
        (cs &
         CAN2_CS_IDE_MASK)
        != 0U
    )
    {
        frame->extended =
            1U;

        frame->id =
            id_word &
            0x1FFFFFFFUL;
    }
    else
    {
        frame->extended =
            0U;

        frame->id =
            (id_word >> 18U) &
            0x7FFU;
    }


    frame->rtr =
        (
            (cs &
             CAN2_CS_RTR_MASK)
            != 0U
        )
        ? 1U
        : 0U;


    frame->dlc =
        dlc;


    frame->data[0] =
        (uint8_t)(
            data0 >> 24U
        );

    frame->data[1] =
        (uint8_t)(
            data0 >> 16U
        );

    frame->data[2] =
        (uint8_t)(
            data0 >> 8U
        );

    frame->data[3] =
        (uint8_t)(
            data0
        );


    frame->data[4] =
        (uint8_t)(
            data1 >> 24U
        );

    frame->data[5] =
        (uint8_t)(
            data1 >> 16U
        );

    frame->data[6] =
        (uint8_t)(
            data1 >> 8U
        );

    frame->data[7] =
        (uint8_t)(
            data1
        );


    /*
     * Unlock mailbox.
     */

    (void)CAN0->TIMER;


    /*
     * Clear flag.
     */

    CAN0->IFLAG1 =
        CAN2_RX_MB_FLAG;


    /*
     * Re-arm RX mailbox.
     */

    Can2_SetRxMailbox();


    return 1U;
}


/* ========================================================================== */
/* ERROR MONITOR                                                              */
/* ========================================================================== */


static uint8_t Can2_CheckBusOff(void)
{
    uint32_t esr;


    esr =
        CAN0->ESR1;


    if(
        (esr &
         CAN_ESR1_BOFFINT_MASK)
        != 0U
    )
    {
        CAN0->ESR1 =
            CAN_ESR1_BOFFINT_MASK;


        g_can2_status.bus_off_count++;


        g_can2_status.error_count++;


        RTT_LOG(
            "[CAN2_ERR] BUS OFF\r\n"
        );


        return 1U;
    }


    return 0U;
}


/* ========================================================================== */
/* BAUD DETECTION                                                             */
/* ========================================================================== */


void Can2_StartDetection(void)
{
    g_can2_status.detected =
        0U;


    g_can2_status.detected_baud_kbps =
        0U;


    g_can2_status.state =
        CAN2_STATE_DETECTING;


    g_can2_baud_index =
        0U;


    g_can2_detect_tick =
        0U;


    g_can2_no_frame_counter =
        0U;


    g_can2_confirm_count =
        0U;


    RTT_LOG(
        "[CAN2] Start auto baud\r\n"
    );


    if(
        Can2_SetBaud(
            g_can2_baud_index,
            1U
        )
        == 0U
    )
    {
        g_can2_status.state =
            CAN2_STATE_ERROR;


        RTT_LOG(
            "[CAN2_ERR] Cannot set first baud\r\n"
        );

        return;
    }


    RTT_LOG(
        "[CAN2] Detecting %lu kbps LOM\r\n",
        (unsigned long)
        g_can2_baud_kbps[
            g_can2_baud_index
        ]
    );
}


static void Can2_NextBaud(void)
{
    g_can2_baud_index++;


    if(
        g_can2_baud_index >=
        CAN2_AUTO_BAUD_COUNT
    )
    {
        g_can2_baud_index =
            0U;
    }


    g_can2_confirm_count =
        0U;


    if(
        Can2_SetBaud(
            g_can2_baud_index,
            1U
        )
        == 0U
    )
    {
        g_can2_status.error_count++;


        return;
    }


    RTT_LOG(
        "[CAN2] Detecting %lu kbps LOM\r\n",
        (unsigned long)
        g_can2_baud_kbps[
            g_can2_baud_index
        ]
    );
}


static void Can2_LockBaud(void)
{
    uint8_t index;


    index =
        g_can2_baud_index;


    RTT_LOG(
        "[CAN2] Frame detected at %lu kbps\r\n",
        (unsigned long)
        g_can2_baud_kbps[
            index
        ]
    );


    /*
     * Exit Listen Only Mode.
     */

    if(
        Can2_SetBaud(
            index,
            0U
        )
        == 0U
    )
    {
        g_can2_status.state =
            CAN2_STATE_ERROR;


        return;
    }


    g_can2_status.detected =
        1U;


    g_can2_status.detected_baud_kbps =
        g_can2_baud_kbps[
            index
        ];


    g_can2_status.ready =
        1U;


    g_can2_status.state =
        CAN2_STATE_RUNNING;


    g_can2_confirm_count =
        0U;


    RTT_LOG(
        "[CAN2] BAUD LOCKED %lu kbps (confirmed over %u clean frames)\r\n",
        (unsigned long)
        g_can2_status.detected_baud_kbps,
        (unsigned)CAN2_CONFIRM_FRAMES
    );
}


/* ========================================================================== */
/* PUBLIC INIT                                                                */
/* ========================================================================== */


void Can2_Init(void)
{
    RTT_LOG(
        "\r\n[CAN2] INIT START\r\n"
    );


    g_can2_status =
        (Can2_Status_t){0};


    g_can2_rx_callback =
        0;


    g_can2_status.state =
        CAN2_STATE_OFF;


    RTT_LOG(
        "[CAN2] STEP 1 SHDN init\r\n"
    );


    Can2_ShutdownPinInit();


    RTT_LOG(
        "[CAN2] STEP 2 HW init\r\n"
    );


    if(
        Can2_HardwareInit()
        == 0U
    )
    {
        RTT_LOG(
            "[CAN2_ERR] HW INIT FAIL\r\n"
        );


        g_can2_status.state =
            CAN2_STATE_ERROR;


        return;
    }


    RTT_LOG(
        "[CAN2] STEP 3 Wake transceiver\r\n"
    );


    Can2_WakeNormal();


    RTT_LOG(
        "[CAN2] STEP 4 Start detection\r\n"
    );


    Can2_StartDetection();


    RTT_LOG(
        "[CAN2] INIT DONE\r\n"
    );
}


/* ========================================================================== */
/* RX CALLBACK                                                                */
/* ========================================================================== */


void Can2_SetRxCallback(
    Can2_RxCallback_t callback
)
{
    g_can2_rx_callback =
        callback;


    RTT_LOG(
        "[CAN2] RX callback set\r\n"
    );
}


/* ========================================================================== */
/* STATUS                                                                     */
/* ========================================================================== */


void Can2_GetStatus(
    Can2_Status_t *status
)
{
    if(status == 0)
    {
        return;
    }


    *status =
        g_can2_status;
}


/* ========================================================================== */
/* TASK                                                                       */
/* ========================================================================== */


void Can2_Task(void)
{
    Can2_Frame_t frame;


    if(
        g_can2_status.state ==
        CAN2_STATE_OFF
    )
    {
        return;
    }


    if(
        g_can2_status.state ==
        CAN2_STATE_ERROR
    )
    {
        /*
         * Do not block.
         *
         * Recovery can be requested
         * externally.
         */

        return;
    }


    /*
     * Monitor bus-off.
     */

    if(
        Can2_CheckBusOff()
        != 0U
    )
    {
        Can2_StartDetection();

        return;
    }


    /* ---------------------------------------------------------------------- */
    /* DETECTION                                                              */
    /* ---------------------------------------------------------------------- */

    if(
        g_can2_status.state ==
        CAN2_STATE_DETECTING
    )
    {
        uint32_t esr1;
        uint8_t  had_error;


        /*
         * A protocol error AFTER we already have at least one clean
         * frame at this candidate is real evidence the candidate is
         * wrong (or an aliasing lock falling apart) - candidates
         * 1000/500/250/125 are exact 2x multiples of each other, so a
         * receiver listening at half the real bus rate can
         * occasionally build what looks like one short, CRC-valid
         * frame out of real traffic.
         *
         * A protocol error BEFORE any clean frame is normal boundary
         * noise: switching bit-timing while the external transmitter
         * may already be mid-frame produces transient BIT/FRM/STF
         * errors that say nothing about whether this candidate's baud
         * is correct. Ignoring those and relying on the existing
         * silence timeout to reject a truly wrong candidate is what
         * lets a candidate actually get a fair chance to receive a
         * frame in the first place.
         */

        esr1 =
            CAN0->ESR1;

        had_error =
            ((esr1 & CAN2_ERR_FLAGS_MASK) != 0U) ? 1U : 0U;

        if(had_error)
        {
            CAN0->ESR1 =
                CAN2_ERR_FLAGS_MASK;
        }

        if(had_error && (g_can2_confirm_count > 0U))
        {
            CAN0->IFLAG1 =
                CAN2_RX_MB_FLAG;

            RTT_LOG(
                "[CAN2] Bit error at %lu kbps after %u clean frame(s) (ESR1=0x%08lX)"
                " - wrong baud, next candidate\r\n",
                (unsigned long)
                g_can2_baud_kbps[
                    g_can2_baud_index
                ],
                (unsigned)g_can2_confirm_count,
                (unsigned long)esr1
            );

            Can2_NextBaud();

            return;
        }


        /*
         * Frame detected at current baud.
         */

        if(
            Can2_ReadFrame(
                &frame
            )
            != 0U
        )
        {
            g_can2_status.rx_count++;

            if(had_error)
            {
                /* Boundary noise raced with this frame before we have
                 * any confirmation yet - don't count it, but don't
                 * penalize the candidate either. */
                RTT_LOG(
                    "[CAN2] Candidate %lu kbps: frame raced with boundary error"
                    " (ESR1=0x%08lX) - ignored, not yet confirming\r\n",
                    (unsigned long)
                    g_can2_baud_kbps[
                        g_can2_baud_index
                    ],
                    (unsigned long)esr1
                );

                return;
            }

            g_can2_confirm_count++;

            g_can2_detect_tick =
                0U;   /* traffic present - extend the dwell */

            RTT_LOG(
                "[CAN2] Candidate %lu kbps: clean frame %u/%u\r\n",
                (unsigned long)
                g_can2_baud_kbps[
                    g_can2_baud_index
                ],
                (unsigned)g_can2_confirm_count,
                (unsigned)CAN2_CONFIRM_FRAMES
            );

            if(
                g_can2_confirm_count >=
                CAN2_CONFIRM_FRAMES
            )
            {
                Can2_LockBaud();
            }

            return;
        }


        /*
         * No frame this cycle.
         */

        g_can2_no_frame_counter++;


        g_can2_detect_tick++;


        /*
         * Time to try next baud.
         */

        if(
            g_can2_detect_tick >=
            CAN2_AUTO_BAUD_TICKS
        )
        {
            g_can2_detect_tick =
                0U;


            Can2_NextBaud();
        }


        /*
         * Keep detecting forever.
         *
         * Do NOT stop the firmware.
         */

        if(
            g_can2_no_frame_counter >=
            CAN2_NO_FRAME_LIMIT
        )
        {
            g_can2_no_frame_counter =
                0U;


            RTT_LOG(
                "[CAN2] Still waiting for CAN traffic\r\n"
            );
        }


        return;
    }


    /* ---------------------------------------------------------------------- */
    /* RUNNING                                                                */
    /* ---------------------------------------------------------------------- */

    if(
        g_can2_status.state ==
        CAN2_STATE_RUNNING
    )
    {
        if(
            Can2_ReadFrame(
                &frame
            )
            != 0U
        )
        {
            g_can2_status.rx_count++;


            if(
                g_can2_rx_callback
                != 0
            )
            {
                g_can2_rx_callback(
                    &frame
                );
            }
        }


        return;
    }
}
