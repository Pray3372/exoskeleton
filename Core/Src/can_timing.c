/*
 * can_timing.c  —  指令取樣頻率 / 回傳延遲 量測器，說明見 can_timing.h
 */
#include "log_config.h"
#if MOTOR_LOG_MODE == 1

#include "can_timing.h"
#include <stdio.h>
#include <string.h>

#define FUNC_STATUS  0x29

/* 簡單統計累加器 */
typedef struct {
    uint32_t n;
    uint64_t sum;
    uint32_t min, max;
} Acc;

static inline void acc_add(volatile Acc *a, uint32_t v)
{
    a->n++; a->sum += v;
    if (v < a->min) a->min = v;
    if (v > a->max) a->max = v;
}
static inline void acc_reset(volatile Acc *a)
{
    a->n = 0; a->sum = 0; a->min = 0xFFFFFFFFu; a->max = 0;
}

static CAN_HandleTypeDef  *s_hcan;
static UART_HandleTypeDef *s_huart;
static uint32_t s_cyc_per_us;

static volatile uint32_t s_tx_last_cyc, s_tx_prev_cyc, s_rx_prev_cyc;
static volatile uint8_t  s_tx_pending, s_tx_valid, s_rx_valid;
static volatile Acc      s_cmd_dt, s_rep_dt, s_lat;
static volatile uint32_t s_missed, s_unsol;
static volatile int16_t  s_last_cur;

static uint32_t s_report_tick;
static char     s_line[160];

/* ---------------------------------------------------------------- */
static inline uint32_t cyc(void) { return DWT->CYCCNT; }

void CANTIME_MarkTx(void)
{
    uint32_t now = cyc();
    if (s_tx_pending) s_missed++;              /* 上一筆命令沒等到回傳 */
    if (s_tx_valid) acc_add(&s_cmd_dt, (now - s_tx_prev_cyc) / s_cyc_per_us);
    s_tx_prev_cyc = now;
    s_tx_last_cyc = now;
    s_tx_valid    = 1;
    s_tx_pending  = 1;
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef hdr;
    uint8_t d[8];
    uint32_t now = cyc();

    if (hcan != s_hcan) return;

    while (HAL_CAN_GetRxFifoFillLevel(hcan, CAN_RX_FIFO0) > 0) {
        if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &hdr, d) != HAL_OK) break;
        if (hdr.IDE != CAN_ID_EXT || ((hdr.ExtId >> 8) & 0xFF) != FUNC_STATUS) continue;

        if (s_tx_pending) {
            acc_add(&s_lat, (now - s_tx_last_cyc) / s_cyc_per_us);
            s_tx_pending = 0;
        } else {
            s_unsol++;
        }
        if (s_rx_valid) acc_add(&s_rep_dt, (now - s_rx_prev_cyc) / s_cyc_per_us);
        s_rx_prev_cyc = now;
        s_rx_valid    = 1;
        if (hdr.DLC >= 6) s_last_cur = (int16_t)(((uint16_t)d[4] << 8) | d[5]);
    }
}

/* ---------------------------------------------------------------- */
void CANTIME_Init(CAN_HandleTypeDef *hcan, UART_HandleTypeDef *huart)
{
    s_hcan  = hcan;
    s_huart = huart;
    s_cyc_per_us = SystemCoreClock / 1000000U;     /* 168 */

    /* DWT 週期計數器 */
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL  |= DWT_CTRL_CYCCNTENA_Msk;

    acc_reset(&s_cmd_dt); acc_reset(&s_rep_dt); acc_reset(&s_lat);
    s_missed = s_unsol = 0;
    s_tx_pending = s_tx_valid = s_rx_valid = 0;

    HAL_NVIC_SetPriority(CAN1_RX0_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(CAN1_RX0_IRQn);
    HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO0_MSG_PENDING);

    s_report_tick = HAL_GetTick();
    static const char hdr[] = "# can_timing: cmd/reply Hz, dt & latency in us (mean/min/max)\r\n";
    HAL_UART_Transmit_IT(huart, (uint8_t *)hdr, sizeof(hdr) - 1);
}

static int fmt_hz(char *p, uint32_t mean_us)          /* 1e6/mean，印 1 位小數 */
{
    if (!mean_us) return sprintf(p, "0.0Hz");
    uint32_t hz10 = (uint32_t)(10000000ULL / mean_us);
    return sprintf(p, "%lu.%luHz", (unsigned long)(hz10 / 10), (unsigned long)(hz10 % 10));
}

void CANTIME_Process(void)
{
    uint32_t tick = HAL_GetTick();
    if (tick - s_report_tick < CANTIME_REPORT_MS) return;
    s_report_tick = tick;
    if (s_huart->gState != HAL_UART_STATE_READY) return;   /* 上一行還沒送完就跳過這秒 */

    /* 快照 + 清零 */
    __disable_irq();
    Acc cmd = *(Acc *)&s_cmd_dt, rep = *(Acc *)&s_rep_dt, lat = *(Acc *)&s_lat;
    uint32_t missed = s_missed, unsol = s_unsol;
    int16_t  cur    = s_last_cur;
    acc_reset(&s_cmd_dt); acc_reset(&s_rep_dt); acc_reset(&s_lat);
    s_missed = s_unsol = 0;
    __enable_irq();

    uint32_t cmd_mean = cmd.n ? (uint32_t)(cmd.sum / cmd.n) : 0;
    uint32_t rep_mean = rep.n ? (uint32_t)(rep.sum / rep.n) : 0;
    uint32_t lat_mean = lat.n ? (uint32_t)(lat.sum / lat.n) : 0;

    int n = sprintf(s_line, "cmd ");
    n += fmt_hz(s_line + n, cmd_mean);
    n += sprintf(s_line + n, " dt %lu/%lu/%luus | reply ",
                 (unsigned long)cmd_mean, (unsigned long)(cmd.n ? cmd.min : 0), (unsigned long)cmd.max);
    n += fmt_hz(s_line + n, rep_mean);
    n += sprintf(s_line + n, " | lat %lu/%lu/%luus | missed %lu unsol %lu | iq %s%d.%02dA\r\n",
                 (unsigned long)lat_mean, (unsigned long)(lat.n ? lat.min : 0), (unsigned long)lat.max,
                 (unsigned long)missed, (unsigned long)unsol,
                 cur < 0 ? "-" : "", (int)((cur < 0 ? -cur : cur) / 100), (int)((cur < 0 ? -cur : cur) % 100));

    HAL_UART_Transmit_IT(s_huart, (uint8_t *)s_line, (uint16_t)n);
}

#endif /* MOTOR_LOG_MODE == 1 */
