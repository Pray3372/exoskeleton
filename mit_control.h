#ifndef AK_H
#define AK_H

#include <stdint.h>
#include "main.h"
#include <math.h>

// 宣告外部的 CAN 控制代碼 (在 main.c 中定義)
extern CAN_HandleTypeDef hcan1;

volatile float motor_actual_p = 0.0f; // 存放馬達 CAN 回傳的當前實際角度
volatile float start_p = 0.0f;
volatile uint8_t is_first_run = 1; // 用來標記是否為開機第一次執行
// 1. AK60-6 V3.0 規格與 CAN ID
#define DRIVER_ID     1
#define CAN_EXT_ID    ((8 << 8) | DRIVER_ID)
#define P_MIN  -12.56f
#define P_MAX   12.56f

#define V_MIN  -60.0f
#define V_MAX   60.0f

#define KP_MIN   0.0f
#define KP_MAX 500.0f

#define KD_MIN   0.0f
#define KD_MAX   5.0f

#define T_MIN  -12.0f
#define T_MAX   12.0f

// --- 函式宣告 ---
// CAN 過濾器初始化
void CAN_Filter_Init(void);

// MIT 模式封包發送函式
void pack_cmd(uint8_t controller_id, float p_des, float v_des, float kp, float kd, float t_ff);

#endif /* AK_H */

