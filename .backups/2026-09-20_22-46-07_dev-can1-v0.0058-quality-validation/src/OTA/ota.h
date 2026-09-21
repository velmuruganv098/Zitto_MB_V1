#ifndef OTA_H
#define OTA_H

#include <stdint.h>

/* ============================================================

* OTA STATUS
* ============================================================ */
  typedef enum
  {
  OTA_IDLE=0U,
  OTA_RECEIVING,
  OTA_VERIFYING,
  OTA_READY,
  OTA_ERROR,
  OTA_ABORTED
  } OtaState_t;

/* ============================================================

* OTA ERROR
* ============================================================ */
  typedef enum
  {
  OTA_ERR_NONE=0U,
  OTA_ERR_PARAM,
  OTA_ERR_SIZE,
  OTA_ERR_STATE,
  OTA_ERR_FLASH,
  OTA_ERR_CRC,
  OTA_ERR_INCOMPLETE
  } OtaError_t;

/* ============================================================

* OTA INFORMATION
* ============================================================ */
  typedef struct
  {
  uint32_t image_size;
  uint32_t image_crc;
  uint32_t bytes_received;
  uint32_t bytes_written;

  uint16_t current_page;
  uint8_t state;
  uint8_t error;

  uint8_t active;
  uint8_t pending;

} OtaInfo_t;

/* ============================================================

* OTA INITIALIZATION
* ============================================================ */
  void OTA_Init(void);

/* ============================================================

* OTA DOWNLOAD
*
* image_size = complete firmware size in bytes
* image_crc  = expected CRC32 of complete firmware
* ============================================================ */
  uint8_t OTA_Start(uint32_t image_size,uint32_t image_crc);

/* Write firmware chunk */
uint8_t OTA_WriteChunk(const uint8_t *data,uint16_t len);

/* Verify complete firmware */
uint8_t OTA_Finish(void);

/* Abort current OTA */
void OTA_Abort(void);

/* ============================================================

* OTA STATUS
* ============================================================ */
  OtaState_t OTA_GetState(void);
  OtaError_t OTA_GetError(void);
  void OTA_GetInfo(OtaInfo_t *out);

uint8_t OTA_IsActive(void);
uint8_t OTA_IsPending(void);

/* ============================================================

* BOOT CONTROL
* ============================================================ */
  uint8_t OTA_ClearPending(void);

/* ============================================================

* CRC32
* ============================================================ */
  uint32_t OTA_Crc32(const uint8_t *data,uint32_t len);

/* ============================================================

* BACKGROUND TASK
* ============================================================ */
  void OTA_Task(void);

#endif
