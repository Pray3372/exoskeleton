/*
 * can_logger.h  —  AK60-6 電流紀錄器（精簡版）
 *
 * 只做一件事：把馬達每次回傳的 0x29 狀態幀（手冊 2.3.1）解成
 *     時間(ms), 位置(°), 輸出端轉速(rpm), Iq 電流(A), 估算扭矩(N·m), 驅動板溫度(°C), 錯誤碼
 * 一行一行以 CSV 送到 USART2，PC 端用 tools/log_to_csv.py 存檔。
 *
 * 使用方式（main.c）：
 *   #include "can_logger.h"
 *   HAL_CAN_Start(&hcan1);
 *   CANLOG_Init(&hcan1, &huart2);     // 要在 DisableKey_Init 之前
 *   while (1) { CANLOG_Process(); ... }
 *
 * 需要 stm32f4xx_it.c 有 CAN1_RX0_IRQHandler() → HAL_CAN_IRQHandler(&hcan1)。
 *
 * 接線：F4 Discovery 的 ST-Link 虛擬 COM 沒接到 PA2，要用 USB-TTL：PA2(TX) → 轉接板 RX，共地。
 */

#ifndef CAN_LOGGER_H
#define CAN_LOGGER_H

#include <stdint.h>
#include "main.h"

/* ---- 依馬達型號調整（AK60-6，手冊 2.2 參數表） ---- */
#define CANLOG_POLE_PAIRS   14        /* 極對數，請與上位機「單位設置」一致 */
#define CANLOG_GEAR_RATIO   6         /* 減速比 */
#define CANLOG_KT_NM_PER_A  0.5994f   /* 扭矩係數 N·m/A，扭矩 ≈ Kt × Iq */

#define CANLOG_UART_BAUD    115200U   /* 100 Hz 以下用 115200 就夠；設到 1000 Hz 請改 921600 */

/* 最新一筆解析結果，主程式可直接讀（例如做過流保護） */
typedef struct {
    volatile uint32_t t_ms;
    volatile float    pos_deg;
    volatile float    spd_rpm;      /* 輸出端轉速 */
    volatile float    iq_A;
    volatile int8_t   temp_C;
    volatile uint8_t  err;
    volatile uint32_t count;        /* 收到幾筆 0x29 */
} CANLOG_Status;

extern CANLOG_Status canlog_status;

void CANLOG_Init(CAN_HandleTypeDef *hcan, UART_HandleTypeDef *huart);
void CANLOG_Process(void);

#endif /* CAN_LOGGER_H */
