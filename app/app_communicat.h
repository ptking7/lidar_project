#ifndef __APP_COMMUNICAT_H
#define __APP_COMMUNICAT_H

#include "stdint.h"
#include "app_detect.h"
#include "usart.h"

#define APP_COMM_FRAME_HEAD0           0xAAu
#define APP_COMM_FRAME_HEAD1           0x55u
#define APP_COMM_PAYLOAD_LEN           8u
#define APP_COMM_TX_PERIOD_MS          50u    /* ? 20 Hz */
#define APP_COMM_LINK_TIMEOUT_MS       500u
#define APP_COMM_RX_DMA_BUF_SIZE       256u

typedef struct {
    int16_t  vx_mm_s;
    uint8_t  heartbeat;
    uint8_t  link_ok;
    uint32_t last_rx_tick;
} AppCommVehicleState_t;

void app_comm_init(void);
void app_comm_process(void);
const AppCommVehicleState_t *app_comm_get_vehicle_state(void);

void app_comm_uart_tx_cplt(UART_HandleTypeDef *huart);

#endif /* __APP_COMMUNICAT_H */
