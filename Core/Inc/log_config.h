/*
 * log_config.h  —  選擇要燒進去的量測程式（兩者共用 CAN RX 中斷，一次只能選一個）
 *
 *   MOTOR_LOG_MODE 1 → can_timing.c  : 量「指令取樣頻率」與「命令→回傳延遲」
 *                                      只在 RAM 累積統計，每秒印一行摘要，不會干擾時序。
 *   MOTOR_LOG_MODE 2 → can_logger.c  : 紀錄每一筆回傳的電流 (CSV 一幀一行)。
 *   MOTOR_LOG_MODE 0 → 兩個都關掉。
 *
 * main.c 只要呼叫 LOG_Init(&hcan1, &huart2) 與 LOG_Process()，
 * AK.c 的 SERVO_Can_Send_Msg() 送出後呼叫 LOG_MarkTx()。
 */
#ifndef LOG_CONFIG_H
#define LOG_CONFIG_H

#ifndef MOTOR_LOG_MODE
#define MOTOR_LOG_MODE  2
#endif

#if MOTOR_LOG_MODE == 1
  #include "can_timing.h"
  #define LOG_Init(hcan, huart)   CANTIME_Init((hcan), (huart))
  #define LOG_Process()           CANTIME_Process()
  #define LOG_MarkTx()            CANTIME_MarkTx()
#elif MOTOR_LOG_MODE == 2
  #include "can_logger.h"
  #define LOG_Init(hcan, huart)   CANLOG_Init((hcan), (huart))
  #define LOG_Process()           CANLOG_Process()
  #define LOG_MarkTx()            ((void)0)
#else
  #define LOG_Init(hcan, huart)   ((void)0)
  #define LOG_Process()           ((void)0)
  #define LOG_MarkTx()            ((void)0)
#endif

#endif /* LOG_CONFIG_H */
