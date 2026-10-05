/*
 * disable_key.h
 *
 * 按下鍵盤 'S'（經由 UART 序列埠終端機送到 STM32）→ 對 AK60-6 送出
 * CAN 失能命令（控制模式 ID 15，手冊 4.1.8 節）。
 * 按 'E' 可解除失能旗標，讓主迴圈恢復送控制命令。
 *
 * 這是一個獨立的新檔案，不需要修改 AK.c / AK.h。
 * 它重用 AK.c 裡已經存在的 SERVO_Can_Send_Msg()。
 *
 * 使用方式（只需要在 main.c 加三行，見 disable_key.c 檔頭說明）
 */

#ifndef DISABLE_KEY_H
#define DISABLE_KEY_H

#include <stdint.h>
#include "main.h"

/* 手冊 4.1 節：電機失能模式的控制模式 ID */
#define CAN_PACKET_DISABLE   15

/* 初始化：傳入要監聽鍵盤的 UART handle（例如 &huart2），
 * 內部會啟動 1-byte 的中斷接收。 */
void DisableKey_Init(UART_HandleTypeDef *huart);

/* 立刻送出失能命令（可在任何地方呼叫，例如緊急停止按鈕） */
void Motor_Disable(uint8_t controller_id);

/* 目前是否處於「已失能、主迴圈不得再送控制命令」的狀態 */
uint8_t DisableKey_IsDisabled(void);

/* 手動清除失能旗標（等同於按 'E'） */
void DisableKey_Clear(void);

#endif /* DISABLE_KEY_H */
