#include "AK.h"
#include <stdint.h>
#include "log_config.h"

// --- 馬達控制的極限參數 ---
const float P_MIN = -12.56f;
const float P_MAX = 12.56f;
const float V_MIN = -60.0f;
const float V_MAX = 60.0f;
const float T_MIN = -12.0f;
const float T_MAX = 12.0f;
const float Kp_MIN = 0.0f;
const float Kp_MAX = 500.0f;
const float Kd_MIN = 0.0f;
const float Kd_MAX = 5.0f;

#define CAN_PACKET_SET_mit 8

// --- 數學輔助函式 ---
float fmaxf(float x, float y) { return (((x)>(y))?(x):(y)); }
float fminf(float x, float y) { return (((x)<(y))?(x):(y)); }

int float_to_uint(float x, float x_min, float x_max, unsigned int bits){
    float span = x_max - x_min;
    if(x < x_min) x = x_min;
    else if(x > x_max) x = x_max;
    return (int) ((x - x_min) * ((float)(1 << bits) / span));
}

// --- 底層 CAN 發送函式 (HAL 庫) ---
uint8_t SERVO_Can_Send_Msg(uint32_t ExtId, uint8_t* msg, uint8_t len)
{
    CAN_TxHeaderTypeDef TxHeader;
    uint32_t TxMailbox;
    uint16_t timeout = 0;

    TxHeader.ExtId = ExtId;
    TxHeader.IDE = CAN_ID_EXT;
    TxHeader.RTR = CAN_RTR_DATA;
    TxHeader.DLC = len;
    TxHeader.TransmitGlobalTime = DISABLE;

    while(HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) == 0) {
        timeout++;
        if(timeout > 0xFFF) return 1;
    }

    if (HAL_CAN_AddTxMessage(&hcan1, &TxHeader, msg, &TxMailbox) != HAL_OK) {
        return 1;
    }
    LOG_MarkTx();   /* 時序量測用：記下送出時間（MODE 2 時為空巨集） */
    return 0;
}

// --- 應用層：打包指令並發送 ---
void pack_cmd(uint8_t controller_id, float p_des, float v_des, float kp, float kd, float t_ff) {
    uint8_t buffer[8];

    p_des = fminf(fmaxf(P_MIN, p_des), P_MAX);
    v_des = fminf(fmaxf(V_MIN, v_des), V_MAX);
    kp = fminf(fmaxf(Kp_MIN, kp), Kp_MAX);
    kd = fminf(fmaxf(Kd_MIN, kd), Kd_MAX);
    t_ff = fminf(fmaxf(T_MIN, t_ff), T_MAX);

    int p_int = float_to_uint(p_des, P_MIN, P_MAX, 16);
    int v_int = float_to_uint(v_des, V_MIN, V_MAX, 12);
    int kp_int = float_to_uint(kp, Kp_MIN, Kp_MAX, 12);
    int kd_int = float_to_uint(kd, Kd_MIN, Kd_MAX, 12);
    int t_int = float_to_uint(t_ff, T_MIN, T_MAX, 12);

    buffer[0] = kp_int >> 4;
    buffer[1] = ((kp_int & 0xF) << 4) | (kd_int >> 8);
    buffer[2] = kd_int & 0xFF;
    buffer[3] = p_int >> 8;
    buffer[4] = p_int & 0xFF;
    buffer[5] = v_int >> 4;
    buffer[6] = ((v_int & 0xF) << 4) | (t_int >> 8);
    buffer[7] = t_int & 0xFF;

    uint32_t ext_id = ((uint32_t)CAN_PACKET_SET_mit << 8) | controller_id;
    SERVO_Can_Send_Msg(ext_id, buffer, 8);
}

// --- 初始化 CAN 過濾器 ---
void CAN_Filter_Init(void)
{
    CAN_FilterTypeDef sFilterConfig;

    sFilterConfig.FilterBank = 0;
    sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
    sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
    sFilterConfig.FilterIdHigh = 0x0000;
    sFilterConfig.FilterIdLow = 0x0000;
    sFilterConfig.FilterMaskIdHigh = 0x0000;
    sFilterConfig.FilterMaskIdLow = 0x0000;
    sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
    sFilterConfig.FilterActivation = ENABLE;
    sFilterConfig.SlaveStartFilterBank = 14;

    if (HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig) != HAL_OK) {
        Error_Handler();
    }
}
