#ifndef APP_MAIN_H
#define APP_MAIN_H

#include <stdint.h>

#include "../UART/uart_pkt_types.h"


void App_Init(void);

void App_Task(void);

void App_GetStatus(StatusPkt_t *status);


#endif /* APP_MAIN_H */
