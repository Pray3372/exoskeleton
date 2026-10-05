/*
 * disable_key.c   (PA0 藍色 User 按鈕版本)
 *
 * 只需要這兩個檔案，main.c 維持原樣：
 *   #include "disable_key.h"
 *   DisableKey_Init(&huart2);                 // HAL_CAN_Start() 之後
 *   if (!DisableKey_IsDisabled()) pack_cmd(...);
 *
 * 操作方式：
 *   短按藍色按鈕          → 馬達失能（送 ID 15），主迴圈停止送 pack_cmd
 *   按住藍色按鈕 ≥ 2 秒放開 → 解除失能，主迴圈恢復送 pack_cmd（馬達會重新使能！）
 *   UART 'S' / 'E'        → 同上（目前 DISC1 的 ST-Link VCP 沒接到 PA2/PA3，
 *                            要接 USB-TTL 才有作用，不影響按鈕）
 */

#include "disable_key.h"

/* 你的馬達 ID（和 pack_cmd 第一個參數一致） */
#ifndef MOTOR_ID
#define MOTOR_ID  1
#endif

/* 時間參數 (ms) */
#define DEBOUNCE_MS        30      /* 按鈕去彈跳 */
#define LONG_PRESS_MS      2000    /* 長按解除失能 */
#define RESEND_PERIOD_MS   20      /* 失能期間重送失能幀的週期 */

/* 藍色按鈕：main.h 由 CubeMX 產生 B1_Pin / B1_GPIO_Port，若沒有就退回 PA0 */
#ifndef B1_Pin
#define B1_Pin        GPIO_PIN_0
#define B1_GPIO_Port  GPIOA
#endif

/* AK.c 裡已經有這個函式，只是沒放進 AK.h，這裡自己宣告 extern 即可 */
extern uint8_t SERVO_Can_Send_Msg(uint32_t ExtId, uint8_t *msg, uint8_t len);

/* ------------------------------------------------------------ */
static UART_HandleTypeDef *s_huart = 0;
static uint8_t  s_rx_byte = 0;
static volatile uint8_t s_disabled = 0;

/* 按鈕狀態機 */
static uint8_t  s_btn_stable   = 0;   /* 去彈跳後的穩定電位 (1 = 按下) */
static uint8_t  s_btn_raw_last = 0;
static uint32_t s_btn_edge_tick = 0;  /* 最後一次原始電位變化的時間 */
static uint32_t s_btn_press_tick = 0; /* 穩定按下的起始時間 */
static uint32_t s_last_resend_tick = 0;

/* ------------------------------------------------------------ */
static void send_disable_frame(void)
{
    uint8_t dummy[8] = {0};
    /* 手冊：無需發送資料內容，發送對應功能幀即可失能（DLC = 0） */
    SERVO_Can_Send_Msg(((uint32_t)CAN_PACKET_DISABLE << 8) | MOTOR_ID, dummy, 0);
}

void Motor_Disable(uint8_t controller_id)
{
    uint8_t dummy[8] = {0};
    uint32_t ext_id = ((uint32_t)CAN_PACKET_DISABLE << 8) | controller_id;

    /* 連送 3 次，避免匯流排忙碌時第一幀被丟。此函式只在主迴圈情境呼叫。 */
    for (int i = 0; i < 3; i++) {
        SERVO_Can_Send_Msg(ext_id, dummy, 0);
        HAL_Delay(2);
    }
    s_disabled = 1;
    s_last_resend_tick = HAL_GetTick();
}

void DisableKey_Clear(void)
{
    s_disabled = 0;
}

/* ------------------------------------------------------------ */
void DisableKey_Init(UART_HandleTypeDef *huart)
{
    uint32_t now = HAL_GetTick();

    s_huart = huart;
    if (s_huart) {
        HAL_UART_Receive_IT(s_huart, &s_rx_byte, 1);
    }

    /* 讀一次目前電位當作初始穩定狀態，避免開機時誤判成「按下」邊緣 */
    s_btn_raw_last  = (HAL_GPIO_ReadPin(B1_GPIO_Port, B1_Pin) == GPIO_PIN_SET);
    s_btn_stable    = s_btn_raw_last;
    s_btn_edge_tick = now;
    s_btn_press_tick = now;
}

/* ------------------------------------------------------------ */
/* 主迴圈每圈呼叫。把按鈕輪詢放在這裡，是為了不需要改 main.c。 */
uint8_t DisableKey_IsDisabled(void)
{
    uint32_t now = HAL_GetTick();

    /* ---- 1. 按鈕去彈跳 ---- */
    uint8_t raw = (HAL_GPIO_ReadPin(B1_GPIO_Port, B1_Pin) == GPIO_PIN_SET);
    if (raw != s_btn_raw_last) {
        s_btn_raw_last  = raw;
        s_btn_edge_tick = now;              /* 電位剛變，重新計時 */
    }
    else if (raw != s_btn_stable && (now - s_btn_edge_tick) >= DEBOUNCE_MS) {
        /* 電位已穩定超過 DEBOUNCE_MS，確認為真正的邊緣 */
        s_btn_stable = raw;

        if (s_btn_stable) {
            /* ---- 按下：立刻失能 ---- */
            s_btn_press_tick = now;
            send_disable_frame();           /* 先送一幀，不等 */
            s_disabled = 1;
            s_last_resend_tick = now;
        }
        else {
            /* ---- 放開：若按住時間夠長，解除失能 ---- */
            if ((now - s_btn_press_tick) >= LONG_PRESS_MS) {
                s_disabled = 0;
            }
        }
    }

    /* ---- 2. 失能期間週期重送失能幀，補上可能被丟掉的那一幀 ---- */
    if (s_disabled && (now - s_last_resend_tick) >= RESEND_PERIOD_MS) {
        send_disable_frame();
        s_last_resend_tick = now;
    }

    return s_disabled;
}

/* ------------------------------------------------------------ */
/* UART 'S' / 'E'（保留；接了 USB-TTL 才會有作用） */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart == s_huart) {
        if (s_rx_byte == 'S' || s_rx_byte == 's') {
            send_disable_frame();           /* 中斷內不可用 HAL_Delay，只送一幀 */
            s_disabled = 1;                 /* 之後由 IsDisabled() 週期重送 */
        }
        else if (s_rx_byte == 'E' || s_rx_byte == 'e') {
            s_disabled = 0;
        }
        HAL_UART_Receive_IT(s_huart, &s_rx_byte, 1);
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart == s_huart) {
        __HAL_UART_CLEAR_OREFLAG(huart);
        HAL_UART_Receive_IT(s_huart, &s_rx_byte, 1);
    }
}
