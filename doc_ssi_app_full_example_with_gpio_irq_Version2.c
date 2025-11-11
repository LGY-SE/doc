/*
 * SSI硬件握手协议 + FreeRTOS + SPI DMA + FIFO + CS/SRQ握手同步
 * 主控/从端 可主动发起&响应通信 + GPIO中断（IRQ）流控
 *
 * 主端和从端都通过SRQ/CS的GPIO中断callback通知任务发起/响应通信流程
 */

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "stm32f4xx_hal.h"
#include <string.h>

#define SSI_HEADER_SIZE   4
#define SSI_MAX_PAYLOAD   256
#define FIFO_SIZE         8

#define MASTER_CS_GPIO_Port GPIOA
#define MASTER_CS_Pin GPIO_PIN_4
#define SLAVE_SRQ_GPIO_Port GPIOB
#define SLAVE_SRQ_Pin GPIO_PIN_8

typedef struct {
    uint8_t frame_type;
    uint8_t len;
    uint16_t crc16;
} __attribute__((packed)) ssi_hdr_t;

typedef struct {
    uint8_t type;
    uint8_t len;
    uint8_t payload[SSI_MAX_PAYLOAD];
} ssi_frame_t;

typedef struct {
    ssi_frame_t buf[FIFO_SIZE];
    int head, tail, count;
    SemaphoreHandle_t sem, mutex;
} ssi_fifo_t;

void fifo_init(ssi_fifo_t *f) {
    f->head = f->tail = f->count = 0;
    f->sem = xSemaphoreCreateCounting(FIFO_SIZE, 0);
    f->mutex = xSemaphoreCreateMutex();
}
int fifo_push(ssi_fifo_t *f, const ssi_frame_t *frame) {
    xSemaphoreTake(f->mutex, portMAX_DELAY);
    if (f->count == FIFO_SIZE) { xSemaphoreGive(f->mutex); return -1; }
    f->buf[f->tail] = *frame;
    f->tail = (f->tail + 1) % FIFO_SIZE; f->count++;
    xSemaphoreGive(f->mutex); xSemaphoreGive(f->sem); return 0;
}
int fifo_pop(ssi_fifo_t *f, ssi_frame_t *frame, TickType_t timeout) {
    if (xSemaphoreTake(f->sem, timeout) != pdTRUE) return -1;
    xSemaphoreTake(f->mutex, portMAX_DELAY);
    *frame = f->buf[f->head];
    f->head = (f->head + 1) % FIFO_SIZE; f->count--;
    xSemaphoreGive(f->mutex); return 0;
}

uint16_t crc16(const uint8_t *data, uint16_t len) {
    uint16_t crc = 0xFFFF;
    for (uint16_t i = 0; i < len; i++) {
        crc ^= ((uint16_t)data[i]) << 8;
        for (uint8_t j = 0; j < 8; j++)
            crc = (crc & 0x8000) ? ((crc << 1) ^ 0x1021) : (crc << 1);
    }
    return crc;
}
void pack_hdr(uint8_t *buf, uint8_t type, uint8_t len) {
    ssi_hdr_t *hdr = (ssi_hdr_t*)buf;
    hdr->frame_type = type; hdr->len = len;
    hdr->crc16 = crc16(buf, SSI_HEADER_SIZE - 2);
}
void pack_payload(uint8_t *buf, const uint8_t *payload, uint8_t plen) {
    memcpy(buf, payload, plen);
    uint16_t crc = crc16(buf, plen);
    buf[plen] = (uint8_t)crc; buf[plen+1] = (uint8_t)(crc>>8);
}

/* 信号量与资源 */
SemaphoreHandle_t spiDMASem, commMutex;
SemaphoreHandle_t csSem, irqSem;
extern SPI_HandleTypeDef hspi1;

/*
 * GPIO中断回调通知信号量
 * 可用CubeMX在SRQ/CS引脚配置EXTI和NVIC，调用如下
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    BaseType_t hpw = pdFALSE;
    if (GPIO_Pin == SLAVE_SRQ_Pin) {
        xSemaphoreGiveFromISR(irqSem, &hpw);  // 若slave也用SRQ IRQ
    }
    if (GPIO_Pin == MASTER_CS_Pin) {
        xSemaphoreGiveFromISR(csSem, &hpw); // Slave捕捉CS IRQ
    }
    portYIELD_FROM_ISR(hpw);
}
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi) {
    BaseType_t hpw = pdFALSE; xSemaphoreGiveFromISR(spiDMASem, &hpw); portYIELD_FROM_ISR(hpw);
}

/* ===== 主控主动握手/通信 ===== */
int master_active_handshake(ssi_fifo_t *tx_fifo, ssi_fifo_t *rx_fifo, uint32_t timeout_ms)
{
    if (xSemaphoreTake(commMutex, pdMS_TO_TICKS(timeout_ms)) != pdTRUE) return -10;
    ssi_frame_t txframe, rxframe;

    uint8_t tx_hdr[SSI_HEADER_SIZE], rx_hdr[SSI_HEADER_SIZE];

    if (fifo_pop(tx_fifo, &txframe, 0) != 0) 
    { xSemaphoreGive(commMutex); return -11;}
    else 
    {pack_hdr(tx_hdr, txframe.type, txframe.len);}

    // 拉低CS
    HAL_GPIO_WritePin(MASTER_CS_GPIO_Port, MASTER_CS_Pin, GPIO_PIN_RESET);

    // 等待SRQ高并中断通知
    if (xSemaphoreTake(irqSem, pdMS_TO_TICKS(timeout_ms)) != pdTRUE) {
        HAL_GPIO_WritePin(MASTER_CS_GPIO_Port, MASTER_CS_Pin, GPIO_PIN_SET); xSemaphoreGive(commMutex); return -11;
    }

    // SPI两段通信
    xSemaphoreTake(spiDMASem, 0);
    HAL_SPI_TransmitReceive_DMA(&hspi1, tx_hdr, rx_hdr, SSI_HEADER_SIZE);
    if (xSemaphoreTake(spiDMASem, pdMS_TO_TICKS(timeout_ms)) != pdTRUE) {
        HAL_GPIO_WritePin(MASTER_CS_GPIO_Port, MASTER_CS_Pin, GPIO_PIN_SET); xSemaphoreGive(commMutex); return -2;
    }

    ssi_hdr_t *peer_hdr = (ssi_hdr_t*)rx_hdr;
    if (crc16(rx_hdr, SSI_HEADER_SIZE) != 0) {
        HAL_GPIO_WritePin(MASTER_CS_GPIO_Port, MASTER_CS_Pin, GPIO_PIN_SET); xSemaphoreGive(commMutex); return -3;
    }
    uint8_t peer_len = peer_hdr->len; rxframe.type = peer_hdr->frame_type; rxframe.len = peer_len;
    uint8_t tx_pld[SSI_MAX_PAYLOAD+2], rx_pld[SSI_MAX_PAYLOAD+2];
    pack_payload(tx_pld, txframe.payload, txframe.len);

    // 等待SRQ高并中断通知
    if (xSemaphoreTake(irqSem, pdMS_TO_TICKS(timeout_ms)) != pdTRUE) {
        HAL_GPIO_WritePin(MASTER_CS_GPIO_Port, MASTER_CS_Pin, GPIO_PIN_SET); xSemaphoreGive(commMutex); return -11;
    }

    xSemaphoreTake(spiDMASem, 0);
    HAL_SPI_TransmitReceive_DMA(&hspi1, tx_pld, rx_pld, MIN(MAX(txframe.len,rxframe.len),SSI_MAX_PAYLOAD)+2);
    if (xSemaphoreTake(spiDMASem, pdMS_TO_TICKS(timeout_ms)) != pdTRUE) { HAL_GPIO_WritePin(MASTER_CS_GPIO_Port, MASTER_CS_Pin, GPIO_PIN_SET); xSemaphoreGive(commMutex); return -4; }
    uint16_t peer_crc = ((uint16_t)rx_pld[peer_len+1] << 8) | rx_pld[peer_len];
    if (crc16(rx_pld, peer_len) != peer_crc) { HAL_GPIO_WritePin(MASTER_CS_GPIO_Port, MASTER_CS_Pin, GPIO_PIN_SET); xSemaphoreGive(commMutex); return -5; }
    memcpy(rxframe.payload, rx_pld, peer_len); 
    fifo_push(rx_fifo, &rxframe);

    HAL_GPIO_WritePin(MASTER_CS_GPIO_Port, MASTER_CS_Pin, GPIO_PIN_SET);
    xSemaphoreGive(commMutex);
    return 0;
}

/* ===== 主控响应Slave主动通信 ===== */
int master_respond_to_slave_irq(ssi_fifo_t *tx_fifo, ssi_fifo_t *rx_fifo, uint32_t timeout_ms)
{
    if (xSemaphoreTake(commMutex, pdMS_TO_TICKS(timeout_ms)) != pdTRUE) return -10;
    ssi_frame_t txframe, rxframe;
    uint8_t tx_hdr[SSI_HEADER_SIZE], rx_hdr[SSI_HEADER_SIZE];
    if (fifo_pop(tx_fifo, &txframe, 0) != 0) 
    { memset(tx_hdr,0xFF,SSI_HEADER_SIZE); }
    else 
    {pack_hdr(tx_hdr, txframe.type, txframe.len);}

    HAL_GPIO_WritePin(MASTER_CS_GPIO_Port, MASTER_CS_Pin, GPIO_PIN_RESET);

    // SPI两段通信...
    xSemaphoreTake(spiDMASem, 0);
    HAL_SPI_TransmitReceive_DMA(&hspi1, tx_hdr, rx_hdr, SSI_HEADER_SIZE);
    if (xSemaphoreTake(spiDMASem, pdMS_TO_TICKS(timeout_ms)) != pdTRUE) { HAL_GPIO_WritePin(MASTER_CS_GPIO_Port, MASTER_CS_Pin, GPIO_PIN_SET); xSemaphoreGive(commMutex); return -2; }
    

    ssi_hdr_t *peer_hdr = (ssi_hdr_t*)rx_hdr;
    uint8_t peer_len = peer_hdr->len; rxframe.type = peer_hdr->frame_type; rxframe.len = peer_len;
    uint8_t tx_pld[SSI_MAX_PAYLOAD+2], rx_pld[SSI_MAX_PAYLOAD+2];
    pack_payload(tx_pld, txframe.payload, txframe.len);

    // 等待SRQ高并中断通知
    if (xSemaphoreTake(irqSem, pdMS_TO_TICKS(timeout_ms)) != pdTRUE) {
        HAL_GPIO_WritePin(MASTER_CS_GPIO_Port, MASTER_CS_Pin, GPIO_PIN_SET); xSemaphoreGive(commMutex); return -11;
    }

    xSemaphoreTake(spiDMASem, 0);
    HAL_SPI_TransmitReceive_DMA(&hspi1, tx_pld, rx_pld, MIN(MAX(txframe.len,rxframe.len),SSI_MAX_PAYLOAD)+2);
    if (xSemaphoreTake(spiDMASem, pdMS_TO_TICKS(timeout_ms)) != pdTRUE) { HAL_GPIO_WritePin(MASTER_CS_GPIO_Port, MASTER_CS_Pin, GPIO_PIN_SET); xSemaphoreGive(commMutex); return -4; }
    uint16_t peer_crc = ((uint16_t)rx_pld[peer_len+1] << 8) | rx_pld[peer_len];
    memcpy(rxframe.payload, rx_pld, peer_len); if (crc16(rx_pld, peer_len) != peer_crc) { HAL_GPIO_WritePin(MASTER_CS_GPIO_Port, MASTER_CS_Pin, GPIO_PIN_SET); xSemaphoreGive(commMutex); return -5;}
    fifo_push(rx_fifo, &rxframe);
    HAL_GPIO_WritePin(MASTER_CS_GPIO_Port, MASTER_CS_Pin, GPIO_PIN_SET);
    xSemaphoreGive(commMutex);
    return 0;
}

/* ===== 从端响应主控主动通信 ===== */
int slave_response_to_master_irq(ssi_fifo_t *tx_fifo, ssi_fifo_t *rx_fifo, uint32_t timeout_ms)
{
    if (xSemaphoreTake(commMutex, pdMS_TO_TICKS(timeout_ms)) != pdTRUE) return -10;
    ssi_frame_t txframe, rxframe;
    uint8_t tx_hdr[SSI_HEADER_SIZE], rx_hdr[SSI_HEADER_SIZE];
    if (fifo_pop(tx_fifo, &txframe, 0) != 0) 
    {memset(tx_hdr,0xFF,SSI_HEADER_SIZE);}
    else
    {pack_hdr(tx_hdr, txframe.type, txframe.len);}
      
    xSemaphoreTake(spiDMASem, 0);
    HAL_SPI_TransmitReceive_DMA(&hspi1, tx_hdr, rx_hdr, SSI_HEADER_SIZE);
    HAL_GPIO_WritePin(SLAVE_SRQ_GPIO_Port, SLAVE_SRQ_Pin, GPIO_PIN_RESET);
    if (xSemaphoreTake(spiDMASem, pdMS_TO_TICKS(timeout_ms)) != pdTRUE) { HAL_GPIO_WritePin(SLAVE_SRQ_GPIO_Port, SLAVE_SRQ_Pin, GPIO_PIN_SET); xSemaphoreGive(commMutex); return -2; }
    HAL_GPIO_WritePin(SLAVE_SRQ_GPIO_Port, SLAVE_SRQ_Pin, GPIO_PIN_SET);

    ssi_hdr_t *peer_hdr = (ssi_hdr_t*)rx_hdr;
    uint8_t peer_len = peer_hdr->len; rxframe.type = peer_hdr->frame_type; rxframe.len = peer_len;
    uint8_t tx_pld[SSI_MAX_PAYLOAD+2], rx_pld[SSI_MAX_PAYLOAD+2];
    pack_payload(tx_pld, txframe.payload, txframe.len);
    xSemaphoreTake(spiDMASem, 0);
    HAL_SPI_TransmitReceive_DMA(&hspi1, tx_pld, rx_pld, MIN(MAX(txframe.len,rxframe.len),SSI_MAX_PAYLOAD)+2);
    HAL_GPIO_WritePin(SLAVE_SRQ_GPIO_Port, SLAVE_SRQ_Pin, GPIO_PIN_RESET);
    if (xSemaphoreTake(spiDMASem, pdMS_TO_TICKS(timeout_ms)) != pdTRUE) 
    { HAL_GPIO_WritePin(SLAVE_SRQ_GPIO_Port, SLAVE_SRQ_Pin, GPIO_PIN_SET); xSemaphoreGive(commMutex); return -4; }

    uint16_t peer_crc = ((uint16_t)rx_pld[peer_len+1] << 8) | rx_pld[peer_len];
    memcpy(rxframe.payload, rx_pld, peer_len); if (crc16(rx_pld, peer_len) != peer_crc) { HAL_GPIO_WritePin(SLAVE_SRQ_GPIO_Port, SLAVE_SRQ_Pin, GPIO_PIN_SET); xSemaphoreGive(commMutex); return -5;}
    fifo_push(rx_fifo, &rxframe);

    HAL_GPIO_WritePin(SLAVE_SRQ_GPIO_Port, SLAVE_SRQ_Pin, GPIO_PIN_SET);
    xSemaphoreGive(commMutex);
    return 0;
}

/* ===== 从端主动发起通信 (SRQ高，GPIO IRQ通知主控，然后主控拉低CS) ===== */
int slave_initiate_handshake_irq(ssi_fifo_t *tx_fifo, ssi_fifo_t *rx_fifo, uint32_t timeout_ms)
{
    if (xSemaphoreTake(commMutex, pdMS_TO_TICKS(timeout_ms)) != pdTRUE) return -10;
    ssi_frame_t txframe, rxframe;
    uint8_t tx_hdr[SSI_HEADER_SIZE], rx_hdr[SSI_HEADER_SIZE];

    if (fifo_pop(tx_fifo, &txframe, 0) != 0) 
    {xSemaphoreGive(commMutex); return -11;}
    else 
    {pack_hdr(tx_hdr, txframe.type, txframe.len)};

    HAL_GPIO_WritePin(SLAVE_SRQ_GPIO_Port, SLAVE_SRQ_Pin, GPIO_PIN_RESET);

    // master检测SRQ后拉低CS（由主控IRQ任务完成）

    // 等待CS低IRQ通知
    if (xSemaphoreTake(irqSem, pdMS_TO_TICKS(timeout_ms)) != pdTRUE) { HAL_GPIO_WritePin(SLAVE_SRQ_GPIO_Port, SLAVE_SRQ_Pin, GPIO_PIN_SET); xSemaphoreGive(commMutex); return -2;}
    
    xSemaphoreTake(spiDMASem, 0);
    HAL_SPI_TransmitReceive_DMA(&hspi1, tx_hdr, rx_hdr, SSI_HEADER_SIZE);
    if (xSemaphoreTake(spiDMASem, pdMS_TO_TICKS(timeout_ms)) != pdTRUE) { HAL_GPIO_WritePin(SLAVE_SRQ_GPIO_Port, SLAVE_SRQ_Pin, GPIO_PIN_SET); xSemaphoreGive(commMutex); return -3; }
    HAL_GPIO_WritePin(SLAVE_SRQ_GPIO_Port, SLAVE_SRQ_Pin, GPIO_PIN_SET);

    ssi_hdr_t *peer_hdr = (ssi_hdr_t*)rx_hdr;
    uint8_t peer_len = peer_hdr->len; rxframe.type = peer_hdr->frame_type; rxframe.len = peer_len;
    uint8_t tx_pld[SSI_MAX_PAYLOAD+2], rx_pld[SSI_MAX_PAYLOAD+2];
    pack_payload(tx_pld, txframe.payload, txframe.len);
    xSemaphoreTake(spiDMASem, 0);
    HAL_SPI_TransmitReceive_DMA(&hspi1, tx_pld, rx_pld, MIN(MAX(txframe.len,rxframe.len),SSI_MAX_PAYLOAD)+2);
    HAL_GPIO_WritePin(SLAVE_SRQ_GPIO_Port, SLAVE_SRQ_Pin, GPIO_PIN_RESET);
    if (xSemaphoreTake(spiDMASem, pdMS_TO_TICKS(timeout_ms)) != pdTRUE) { HAL_GPIO_WritePin(SLAVE_SRQ_GPIO_Port, SLAVE_SRQ_Pin, GPIO_PIN_RESET); xSemaphoreGive(commMutex); return -4;}
    uint16_t peer_crc = ((uint16_t)rx_pld[peer_len+1] << 8) | rx_pld[peer_len];
    memcpy(rxframe.payload, rx_pld, peer_len); if (crc16(rx_pld, peer_len) != peer_crc) { HAL_GPIO_WritePin(SLAVE_SRQ_GPIO_Port, SLAVE_SRQ_Pin, GPIO_PIN_RESET); xSemaphoreGive(commMutex); return -5;}
    fifo_push(rx_fifo, &rxframe);
    HAL_GPIO_WritePin(SLAVE_SRQ_GPIO_Port, SLAVE_SRQ_Pin, GPIO_PIN_SET);
    xSemaphoreGive(commMutex);
    return 0;
}

/* ===== 任务示例 ===== */
ssi_fifo_t master_tx_fifo, master_rx_fifo;
ssi_fifo_t slave_tx_fifo, slave_rx_fifo;

void MasterTask(void *param) {
    ssi_frame_t f;
    for (int i = 0; i < FIFO_SIZE; ++i) {
        f.type=0xA1; f.len=24+i*8; for(int j=0;j<f.len;j++)f.payload[j]=j;
        fifo_push(&master_tx_fifo, &f);
    }
    while (1) {
        // 等待SRQ高的IRQ通知
        if (xSemaphoreTake(irqSem, 0) == pdTRUE) {
            master_respond_to_slave_irq(&master_tx_fifo, &master_rx_fifo, 500);  // 响应slave主动IRQ请求
        } else {
            master_active_handshake(&master_tx_fifo, &master_rx_fifo, 500);      // 主控周期主动采集
        }
        
        vTaskDelay(pdMS_TO_TICKS(18));
    }
}
void SlaveTask(void *param) {
    ssi_frame_t f;
    for (int i = 0; i < FIFO_SIZE; ++i) {
        f.type=0xB2; f.len=12+i*3; for(int j=0;j<f.len;j++)f.payload[j]=0x40+j;
        fifo_push(&slave_tx_fifo, &f);
    }
    while (1) {
        // 等待CS低的IRQ通知
        if (xSemaphoreTake(csSem, 0) == pdTRUE) {
            slave_response_to_master_irq(&slave_tx_fifo, &slave_rx_fifo, 500);        // 响应主控CS IRQ主动通信
        } else {
            slave_initiate_handshake_irq(&slave_tx_fifo, &slave_rx_fifo, 500);      // 主动IRQ通知主控
        }
        
        vTaskDelay(pdMS_TO_TICKS(18));
    }
}

/* ===== RTOS初始化与启动 ===== */
void ssi_app_init(void) {
    spiDMASem = xSemaphoreCreateBinary();
    commMutex = xSemaphoreCreateMutex();
    csSem = xSemaphoreCreateBinary();
    irqSem = xSemaphoreCreateBinary();
    fifo_init(&master_tx_fifo); fifo_init(&master_rx_fifo);
    fifo_init(&slave_tx_fifo); fifo_init(&slave_rx_fifo);
    xTaskCreate(MasterTask, "MasterTask", 1024, NULL, 3, NULL);
    xTaskCreate(SlaveTask, "SlaveTask", 1024, NULL, 2, NULL);
    vTaskStartScheduler();
}

/*
关键要点：
- 主控/从端均通过GPIO中断通知任务发起/响应通信，避免轮询浪费CPU
- 沟通时序严格：主控CS低->Slave准备SRQ高->IRQ唤醒通信
- 支持从端主动SRQ高主控响应（双向握手协议）
- commMutex保证互斥不冲突
- FIFO支持批量帧并发，无丢包无忙等
- 适配STM32 HAL，移植时只需替换GPIO/SPI初始化与中断配置
*/
