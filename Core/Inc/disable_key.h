/*
 * disable_key.h   (PA0 藍色 User 按鈕版本)
 *
 * 功能：
 *   - 按一下板子上的藍色 User 按鈕 (B1, PA0) → 對 AK60-6 送 CAN 失能命令
 *     (控制模式 ID 15，手冊 4.1.8 節)，並鎖住旗標讓主迴圈停止送 pack_cmd。
 *   - 失能期間每 20 ms 重送一次失能幀，避免匯流排忙碌時第一幀被丟掉。
 *   - 按住按鈕 2 秒以上再放開 → 解除失能旗標，主迴圈恢復送控制命令。
 *   - UART 'S' / 'E' 仍然保留（之後接 USB-TTL 就能用），不影響按鈕功能。
 *
 * 為了不更動 main.c，介面刻意和舊版完全相同：
 *   DisableKey_Init(&huart2)    → 仍然接受 UART handle（可傳 NULL）
 *   DisableKey_IsDisabled()     → 主迴圈每圈都會呼叫，按鈕偵測就放在這裡面做
 *
 * PA0 在 CubeMX 已設成 B1 [Blue PushButton] (GPIO_MODE_EVT_RISING, NOPULL)，
 * 這個模式下 MODER 仍是輸入，HAL_GPIO_ReadPin 可以直接讀，
 * 所以不需要改 .ioc、不需要開 EXTI 中斷。Discovery 板按下時 PA0 = HIGH。
 */

#ifndef DISABLE_KEY_H
#define DISABLE_KEY_H

#include <stdint.h>
#include "main.h"

/* 手冊 4.1 節：電機失能模式的控制模式 ID */
#define CAN_PACKET_DISABLE   15

/* 初始化。huart 可以是 &huart2（保留鍵盤 S/E 功能）或 NULL（只用按鈕）。 */
void DisableKey_Init(UART_HandleTypeDef *huart);

/* 立刻送出失能命令並鎖住旗標（可在任何地方呼叫） */
void Motor_Disable(uint8_t controller_id);

/* 主迴圈每圈呼叫：
 *   內部順便做 PA0 按鈕偵測（去彈跳）、失能幀週期重送、長按解除。
 *   回傳 1 = 已失能，主迴圈不得再送控制命令；0 = 正常。 */
uint8_t DisableKey_IsDisabled(void);

/* 手動清除失能旗標（等同於長按按鈕 2 秒或 UART 'E'） */
void DisableKey_Clear(void);

#endif /* DISABLE_KEY_H */
