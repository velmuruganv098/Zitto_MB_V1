#ifndef SEGGER_RTT_CONF_H
#define SEGGER_RTT_CONF_H

/*
 * SEGGER_RTT_Conf.h
 * Project-specific RTT configuration for Zitto_MB_V1 / S32K144
 */

/* Number of up-channels (target to host) */
#define SEGGER_RTT_MAX_NUM_UP_BUFFERS       1

/* Number of down-channels (host to target) */
#define SEGGER_RTT_MAX_NUM_DOWN_BUFFERS     1

/* Size of the buffer for Terminal 0 output.
 *
 * FIX: was 1024. CAN1's + CAN2's hardware bring-up alone emits ~30
 * RTT_LOG calls back-to-back at boot (no delay between them) -
 * comfortably over 1024 bytes. Combined with NO_BLOCK_SKIP below
 * (writes are silently dropped, not queued, when the buffer is full),
 * this was intermittently losing the entire CAN1/CAN2 hardware-init
 * diagnostic section from RTT captures before the host-side viewer
 * could drain the buffer - not a firmware logic bug, just insufficient
 * headroom for the boot-time logging burst.
 */
#define BUFFER_SIZE_UP                      4096

/* Size of the buffer for Terminal 0 input */
#define BUFFER_SIZE_DOWN                    16

/* NO_BLOCK_SKIP (not BLOCK_IF_FIFO_FULL, despite what an earlier
 * comment here claimed): the firmware must never stall waiting for an
 * RTT viewer that may not be attached - matches this project's
 * non-blocking design requirement. Losing occasional log output when
 * legitimately bursty is an acceptable tradeoff; hanging the whole
 * firmware over unread debug text is not. The buffer size above is
 * the real fix for the reported dropped-output symptom.
 */
#define SEGGER_RTT_MODE_DEFAULT             SEGGER_RTT_MODE_NO_BLOCK_SKIP

#endif /* SEGGER_RTT_CONF_H */
