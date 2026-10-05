#ifndef AK_H
#define AK_H

#include "main.h"

// 宣告外部的 CAN 控制代碼 (在 main.c 中定義)
extern CAN_HandleTypeDef hcan1;

// --- 函式宣告 ---
// CAN 過濾器初始化
void CAN_Filter_Init(void);

// MIT 模式封包發送函式
void pack_cmd(uint8_t controller_id, float p_des, float v_des, float kp, float kd, float t_ff);

#endif /* AK_H */
