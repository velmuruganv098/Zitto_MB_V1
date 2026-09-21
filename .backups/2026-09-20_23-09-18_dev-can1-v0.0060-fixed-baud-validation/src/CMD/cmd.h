#ifndef CMD_H
#define CMD_H

#include <stdint.h>
#include "../UART/uart_pkt.h"

/* ============================================================

* ESP32 -> S32K144 COMMAND TYPES
* ============================================================ */
#define CMD_OTA_INFO        0x8CU

/* ============================================================

* S32K144 -> ESP32 RESPONSE TYPES
* ============================================================ */
  #define CMD_TYPE_ACK        0x70U
  #define CMD_TYPE_NACK       0x71U
  #define CMD_TYPE_OTA_INFO   0x72U
  #define CMD_TYPE_FLASH_DATA 0x73U

/* ============================================================

* COMMAND RESULT
* ============================================================ */
  typedef enum
  {
  CMD_OK=0U,
  CMD_ERR_PARAM=1U,
  CMD_ERR_MODULE=2U,
  CMD_ERR_STATE=3U,
  CMD_ERR_OTA=4U,
  CMD_ERR_UNSUPPORTED=5U,
  CMD_ERR_FLASH=6U,
  CMD_ERR_EMPTY=7U
  } CmdResult_t;

/* ============================================================

* API
* ============================================================ */
  void Cmd_Init(void);
  void Cmd_RxPacket(const UartPkt_t *pkt);

#endif
