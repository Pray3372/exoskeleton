/*
 * can_timing.h  —  指令取樣頻率 / 回傳延遲 量測器（MOTOR_LOG_MODE 1）
 *
 * 量什麼：
 *   cmd_hz     : 你送出 MIT 命令的頻率 = 1 / (兩次 SERVO_Can_Send_Msg 的間隔)
 *   reply_hz   : 馬達 0x29 回傳幀的頻率
 *   latency_us : 從「命令放進 CAN 發送信箱」到「收到下一筆 0x29 回傳」的時間
 *   missed     : 送出命令後，下一筆命令之前沒收到任何回傳的次數（問答模式下 = 掉包）
 *   unsol      : 沒有對應命令就收到的回傳（周期回報模式下這個數字會很大，屬正常）
 *
 * 怎麼量：DWT 週期計數器（168 MHz，解析度 6 ns，以 µs 輸出），所有工作都在
 *   CANTIME_MarkTx() / CAN RX 中斷裡做幾個加法，每秒才由 CANTIME_Process()
 *   印一行摘要到 USART2，不會在每一幀之間插入 UART 動作。
 *
 * 輸出格式（每秒一行，115200）：
 *   cmd 50.0Hz dt 20000/19998/20013us | reply 50.0Hz | lat 312/290/345us | missed 0 unsol 0 | iq -0.12A
 *   (數字順序皆為 mean/min/max)
 */
#ifndef CAN_TIMING_H
#define CAN_TIMING_H

#include <stdint.h>
#include "main.h"

#define CANTIME_REPORT_MS   1000      /* 摘要輸出週期 */

void CANTIME_Init(CAN_HandleTypeDef *hcan, UART_HandleTypeDef *huart);
void CANTIME_MarkTx(void);            /* SERVO_Can_Send_Msg() 送出成功後呼叫 */
void CANTIME_Process(void);           /* 主迴圈每圈呼叫 */

#endif /* CAN_TIMING_H */
