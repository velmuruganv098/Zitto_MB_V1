#ifndef APP_STATUS_H
#define APP_STATUS_H

#include <stdint.h>

typedef struct
{
    uint8_t imu;
    uint8_t csa;
    uint8_t can1;
    uint8_t can2;
    uint8_t flm;
    uint8_t ota;

    uint8_t ota_active;
    uint8_t reserved;

    uint32_t uptime_ms;
    uint32_t heartbeat_count;

} AppStatus_t;


void App_StatusInit(void);

void App_StatusUpdate(uint32_t ms);

void App_StatusGet(AppStatus_t *out);

void App_StatusSetOtaActive(uint8_t active);

void App_StatusHeartbeat(void);

uint32_t App_GetUptimeMs(void);

uint32_t App_GetHeartbeatCount(void);

#endif /* APP_STATUS_H */
