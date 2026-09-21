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

/* Size of the buffer for Terminal 0 output */
#define BUFFER_SIZE_UP                      1024

/* Size of the buffer for Terminal 0 input */
#define BUFFER_SIZE_DOWN                    16

/* Use mode BLOCK_IF_FIFO_FULL for output */
#define SEGGER_RTT_MODE_DEFAULT             SEGGER_RTT_MODE_NO_BLOCK_SKIP

#endif /* SEGGER_RTT_CONF_H */
