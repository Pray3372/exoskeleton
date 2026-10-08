/*
 * can_logger.c  —  AK60-6 電流紀錄器（精簡版），說明見 can_logger.h
 *
 * 流程：CAN RX 中斷把 0x29 幀的原始整數放進小佇列
 *       → 主迴圈 CANLOG_Process() 轉成一行 CSV
 *       → 以 USART2 中斷式發送（不阻塞主迴圈）。
 */

#include "log_config.h"
#if MOTOR_LOG_MODE == 2

#include "can_logger.h"
#include <stdio.h>
#include <string.h>

#define FUNC_STATUS   0x29          /* 手冊 2.3.1 伺服模式即時狀態回傳幀 */
#define QUEUE_LEN     32            /* 2 的次方 */
#define TXBUF_LEN     2048          /* 2 的次方 */

typedef struct {
    uint32_t t_ms;
    int16_t  pos_int;               /* ×0.1° */
    int16_t  spd_int;               /* ×10 ERPM */
    int16_t  cur_int;               /* ×0.01 A */
    int8_t   temp;
    uint8_t  err;
} Sample;

static CAN_HandleTypeDef  *s_hcan;
static UART_HandleTypeDef *s_huart;

static Sample            s_q[QUEUE_LEN];
static volatile uint32_t s_q_head, s_q_tail;

static uint8_t           s_tx[TXBUF_LEN];
static volatile uint32_t s_tx_head, s_tx_tail;
static volatile uint16_t s_tx_chunk;
static volatile uint8_t  s_tx_busy;

CANLOG_Status canlog_status;

/* ---------------------------------------------------------------- */
/* CAN 接收中斷：只抄數字，不做格式化 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef hdr;
    uint8_t d[8];

    if (hcan != s_hcan) return;

    while (HAL_CAN_GetRxFifoFillLevel(hcan, CAN_RX_FIFO0) > 0) {
        if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &hdr, d) != HAL_OK) break;
        if (hdr.IDE != CAN_ID_EXT || ((hdr.ExtId >> 8) & 0xFF) != FUNC_STATUS || hdr.DLC < 8) continue;

        Sample s;
        s.t_ms    = HAL_GetTick();
        s.pos_int = (int16_t)(((uint16_t)d[0] << 8) | d[1]);
        s.spd_int = (int16_t)(((uint16_t)d[2] << 8) | d[3]);
        s.cur_int = (int16_t)(((uint16_t)d[4] << 8) | d[5]);
        s.temp    = (int8_t)d[6];
        s.err     = d[7];

        /* 給主程式即時讀的版本 */
        canlog_status.t_ms    = s.t_ms;
        canlog_status.pos_deg = (float)s.pos_int * 0.1f;
        canlog_status.spd_rpm = (float)s.spd_int * 10.0f / (CANLOG_POLE_PAIRS * CANLOG_GEAR_RATIO);
        canlog_status.iq_A    = (float)s.cur_int * 0.01f;
        canlog_status.temp_C  = s.temp;
        canlog_status.err     = s.err;
        canlog_status.count++;

        /* 排進佇列；滿了就丟最舊的 */
        if (s_q_head - s_q_tail >= QUEUE_LEN) s_q_tail++;
        s_q[s_q_head & (QUEUE_LEN - 1)] = s;
        s_q_head++;
    }
}

/* ---------------------------------------------------------------- */
/* UART 中斷式發送引擎 */
static void tx_kick(void)
{
    if (s_tx_busy) return;
    uint32_t n = s_tx_head - s_tx_tail;
    if (n == 0) return;
    uint32_t i = s_tx_tail & (TXBUF_LEN - 1);
    if (n > TXBUF_LEN - i) n = TXBUF_LEN - i;        /* 不跨越緩衝區尾端 */
    s_tx_chunk = (uint16_t)n;
    s_tx_busy  = 1;
    if (HAL_UART_Transmit_IT(s_huart, &s_tx[i], (uint16_t)n) != HAL_OK) s_tx_busy = 0;
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart != s_huart) return;
    s_tx_tail += s_tx_chunk;
    s_tx_busy  = 0;
    tx_kick();
}

static void tx_line(const char *s, uint32_t len)
{
    if (TXBUF_LEN - (s_tx_head - s_tx_tail) < len) return;   /* 空間不足：整行丟掉 */
    for (uint32_t k = 0; k < len; k++) s_tx[(s_tx_head + k) & (TXBUF_LEN - 1)] = (uint8_t)s[k];
    s_tx_head += len;
    __disable_irq();
    tx_kick();
    __enable_irq();
}

/* 定點輸出：val / 10^dec，例如 (-1234, 2) → "-12.34"（避免用 printf %f） */
static int fmt_fixed(char *out, int32_t val, int dec)
{
    int32_t div = 1;
    for (int k = 0; k < dec; k++) div *= 10;
    int32_t a = val < 0 ? -val : val;
    return sprintf(out, "%s%ld.%0*ld", val < 0 ? "-" : "", (long)(a / div), dec, (long)(a % div));
}

/* ---------------------------------------------------------------- */
void CANLOG_Init(CAN_HandleTypeDef *hcan, UART_HandleTypeDef *huart)
{
    s_hcan  = hcan;
    s_huart = huart;
    memset(&canlog_status, 0, sizeof(canlog_status));

    if (huart->Init.BaudRate != CANLOG_UART_BAUD) {
        HAL_UART_DeInit(huart);
        huart->Init.BaudRate = CANLOG_UART_BAUD;
        HAL_UART_Init(huart);
    }

    HAL_NVIC_SetPriority(CAN1_RX0_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(CAN1_RX0_IRQn);
    HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO0_MSG_PENDING);

    static const char hdr[] = "t_ms,pos_deg,spd_rpm,iq_A,torque_Nm,temp_C,err\r\n";
    tx_line(hdr, sizeof(hdr) - 1);
}

void CANLOG_Process(void)
{
    char line[96];

    while (s_q_head != s_q_tail) {
        Sample s = s_q[s_q_tail & (QUEUE_LEN - 1)];
        s_q_tail++;

        /* 輸出端 rpm ×100、扭矩 ×1000 都用整數算，避免浮點誤差與 printf %f */
        int32_t rpm100   = (int32_t)s.spd_int * 1000 / (CANLOG_POLE_PAIRS * CANLOG_GEAR_RATIO);
        float   tq       = (float)s.cur_int * 0.01f * CANLOG_KT_NM_PER_A * 1000.0f;
        int32_t torque1k = (int32_t)(tq >= 0.0f ? tq + 0.5f : tq - 0.5f);

        int n = sprintf(line, "%lu,", (unsigned long)s.t_ms);
        n += fmt_fixed(line + n, s.pos_int, 1);  line[n++] = ',';
        n += fmt_fixed(line + n, rpm100, 2);     line[n++] = ',';
        n += fmt_fixed(line + n, s.cur_int, 2);  line[n++] = ',';
        n += fmt_fixed(line + n, torque1k, 3);   line[n++] = ',';
        n += sprintf(line + n, "%d,%u\r\n", (int)s.temp, (unsigned)s.err);

        tx_line(line, (uint32_t)n);
    }
}

#endif /* MOTOR_LOG_MODE == 2 */
