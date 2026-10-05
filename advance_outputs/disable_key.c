/*
 * disable_key.c
 *
 * 鍵盤 'S' → AK60-6 失能（CAN 擴展 ID = (15 << 8) | 馬達 ID，DLC = 0）
 * 鍵盤 'E' → 清除失能旗標（主迴圈恢復送 pack_cmd）
 *
 * ============================================================
 *  接線 / CubeMX 設定（你的 .ioc 目前沒有開任何 UART）
 * ============================================================
 *  1. CubeMX 開一個 UART，例如 USART2（PA2 = TX, PA3 = RX），
 *     115200 8N1，NVIC 勾選 "USART2 global interrupt"。
 *  2. PC 端接 USB-TTL 轉接板（TX→PA3, RX→PA2, GND 共地），
 *     用 PuTTY / Tera Term 開 115200，直接在視窗按 S。
 *
 * ============================================================
 *  main.c 需要加的三行（放在 USER CODE 區塊即可）
 * ============================================================
 *  #include "disable_key.h"                       // 檔頭
 *
 *  DisableKey_Init(&huart2);                      // HAL_CAN_Start() 之後
 *
 *  if (!DisableKey_IsDisabled()) {                // while(1) 裡，包住原本的 pack_cmd
 *      pack_cmd(1, 0.0f, 0.0f, 1.0f, 1.0f, 0.0f);
 *  }
 *
 *  ★ 最後這個 if 一定要加：AK60-6 收到「任何」控制命令就會重新使能，
 *    如果主迴圈每 10 ms 繼續送 pack_cmd，按 S 失能後馬達馬上又會被叫醒。
 */

#include "disable_key.h"

/* 你的馬達 ID（和 pack_cmd 第一個參數一致） */
#ifndef MOTOR_ID
#define MOTOR_ID  1
#endif

/* AK.c 裡已經有這個函式，只是沒放進 AK.h，這裡自己宣告 extern 即可 */
extern uint8_t SERVO_Can_Send_Msg(uint32_t ExtId, uint8_t *msg, uint8_t len);

static UART_HandleTypeDef *s_huart = 0;
static uint8_t  s_rx_byte = 0;
static volatile uint8_t s_disabled = 0;

/* ------------------------------------------------------------ */
void Motor_Disable(uint8_t controller_id)
{
    uint8_t dummy[8] = {0};
    uint32_t ext_id = ((uint32_t)CAN_PACKET_DISABLE << 8) | controller_id;

    /* 手冊：無需發送資料內容，發送對應功能幀即可失能（DLC = 0）。
     * 為保險連送 3 次，避免匯流排忙碌時第一幀被丟。 */
    for (int i = 0; i < 3; i++) {
        SERVO_Can_Send_Msg(ext_id, dummy, 0);
        HAL_Delay(2);
    }
    s_disabled = 1;
}

uint8_t DisableKey_IsDisabled(void)
{
    return s_disabled;
}

void DisableKey_Clear(void)
{
    s_disabled = 0;
}

/* ------------------------------------------------------------ */
void DisableKey_Init(UART_HandleTypeDef *huart)
{
    s_huart = huart;
    HAL_UART_Receive_IT(s_huart, &s_rx_byte, 1);
}

/* HAL 的弱符號 callback：每收到 1 byte 進來一次。
 * 如果你的專案別處已經定義了 HAL_UART_RxCpltCallback，
 * 就把下面 if 區塊搬進那個函式裡即可。 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart == s_huart) {
        if (s_rx_byte == 'S' || s_rx_byte == 's') {
            /* 在中斷裡不要用 HAL_Delay，只設旗標，實際送幀交給下面的 poll。
             * 但為了「按下去立刻停」，先直接送一幀，剩下的由旗標處理。 */
            uint8_t dummy[8] = {0};
            SERVO_Can_Send_Msg(((uint32_t)CAN_PACKET_DISABLE << 8) | MOTOR_ID, dummy, 0);
            s_disabled = 1;
        }
        else if (s_rx_byte == 'E' || s_rx_byte == 'e') {
            s_disabled = 0;
        }
        /* 重新掛上下一個 byte 的接收 */
        HAL_UART_Receive_IT(s_huart, &s_rx_byte, 1);
    }
}

/* UART 發生 overrun / framing error 時中斷接收會停掉，這裡把它重新掛回去 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart == s_huart) {
        __HAL_UART_CLEAR_OREFLAG(huart);
        HAL_UART_Receive_IT(s_huart, &s_rx_byte, 1);
    }
}
