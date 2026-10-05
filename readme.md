AK60-6 MIT-control

/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "can.h"
#include "i2c.h"
#include "i2s.h"
#include "spi.h"
#include "usart.h"
#include "usb_host.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN Includes */
#include <math.h>
/* USER CODE END Includes */

/* USER CODE BEGIN PV */
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

// 註：hcan1 已經由 CubeMX 於 main.h / main.c 中宣告，不需重複定義
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_USB_HOST_Process(void);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// 發送設置臨時原點指令 (set_origin_mode: 0 = 臨時原點/斷電消除, 1 = 永久零點)
HAL_StatusTypeDef set_motor_temporary_origin(uint8_t set_origin_mode) {
    CAN_TxHeaderTypeDef txHeader;
    uint8_t txData[1];
    uint32_t txMailbox;

    txData[0] = set_origin_mode; // 0 代表臨時原點 (斷電消除)

    // 依據 CubeMars 協定：(CAN_PACKET_SET_ORIGIN_HERE << 8) | DRIVER_ID
    // CAN_PACKET_SET_ORIGIN_HERE 即為截圖中的 64 (0x40)
    txHeader.ExtId = (64 << 8) | DRIVER_ID;
    txHeader.IDE   = CAN_ID_EXT;
    txHeader.RTR   = CAN_RTR_DATA;
    txHeader.DLC   = 1; // 資料長度 1 Byte

    return HAL_CAN_AddTxMessage(&hcan1, &txHeader, txData, &txMailbox);
}

// 數值範圍轉換輔助函式
float uint_to_float(int uint_val, float min_val, float max_val, int bits) {
    float span = max_val - min_val;
    float offset = min_val;
    return ((float)uint_val) * span / ((float)((1 << bits) - 1)) + offset;
}

// CAN 回傳資料解包函式
// 修正後的 unpack_motor_feedback 函式（請確認第 100 ~ 124 行如下所示）
void unpack_motor_feedback(uint8_t* rxData) {
    uint16_t p_int = (rxData[1] << 8) | rxData[2];
    float raw_p = uint_to_float(p_int, P_MIN, P_MAX, 16);

    static uint8_t initialized = 0;
    if (!initialized) {
        motor_actual_p = raw_p;
        initialized = 1;
        return;
    }

    float delta = raw_p - motor_actual_p;

    if (delta > 3.14159f) {
        delta -= 6.28318f;
    } else if (delta < -3.14159f) {
        delta += 6.28318f;
    }

    motor_actual_p += delta;
}

// 讀取 CAN 接收暫存區 (FIFO0) 函式
void update_motor_position(void) {
    CAN_RxHeaderTypeDef rxHeader;
    uint8_t rxData[8];

    if (HAL_CAN_GetRxFifoFillLevel(&hcan1, CAN_RX_FIFO0) > 0) {
        if (HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &rxHeader, rxData) == HAL_OK) {
            unpack_motor_feedback(rxData);
        }
    }
}

// 發送 AK60-6 馬達關閉/Disable 指令 (8 個 0xFD)
HAL_StatusTypeDef disable_motor(void) {
    CAN_TxHeaderTypeDef txHeader;
    uint8_t txData[8] = {0xFD, 0xFD, 0xFD, 0xFD, 0xFD, 0xFD, 0xFD, 0xFD};
    uint32_t txMailbox;

    txHeader.StdId = 0x00;
    txHeader.ExtId = CAN_EXT_ID;
    txHeader.IDE   = CAN_ID_EXT;
    txHeader.RTR   = CAN_RTR_DATA;
    txHeader.DLC   = 8;

    return HAL_CAN_AddTxMessage(&hcan1, &txHeader, txData, &txMailbox);
}

static int float_to_uint(float x, float x_min, float x_max, int bits) {
    if (x < x_min) x = x_min;
    if (x > x_max) x = x_max;
    return (int)((x - x_min) * ((1 << bits) - 1) / (x_max - x_min));
}

// 單行發送封裝函式：send_mit(p, v, kp, kd, t)
HAL_StatusTypeDef send_mit(float p, float v, float kp, float kd, float t) {
    uint16_t p_i  = float_to_uint(p,  P_MIN,  P_MAX,  16);
    uint16_t v_i  = float_to_uint(v,  V_MIN,  V_MAX,  12);
    uint16_t kp_i = float_to_uint(kp, KP_MIN, KP_MAX, 12);
    uint16_t kd_i = float_to_uint(kd, KD_MIN, KD_MAX, 12);
    uint16_t t_i  = float_to_uint(t,  T_MIN,  T_MAX,  12);

    uint8_t d[8] = {
        (uint8_t)(kp_i >> 4),
        (uint8_t)(((kp_i & 0x0F) << 4) | (kd_i >> 8)),
        (uint8_t)(kd_i & 0xFF),
        (uint8_t)(p_i >> 8),
        (uint8_t)(p_i & 0xFF),
        (uint8_t)(v_i >> 4),
        (uint8_t)(((v_i & 0x0F) << 4) | (t_i >> 8)),
        (uint8_t)(t_i & 0xFF)
    };

    CAN_TxHeaderTypeDef header = {
        .ExtId = CAN_EXT_ID,
        .IDE   = CAN_ID_EXT,
        .RTR   = CAN_RTR_DATA,
        .DLC   = 8
    };
    uint32_t mailbox;
    return HAL_CAN_AddTxMessage(&hcan1, &header, d, &mailbox);
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Initialize all configured peripherals */
    MX_GPIO_Init();
    MX_CAN1_Init(); // CubeMX 自動生成的 CAN 初始化

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_CAN1_Init();
  MX_I2C1_Init();
  MX_I2S3_Init();
  MX_SPI1_Init();
  MX_USART2_UART_Init();
  MX_USB_HOST_Init();

  /* USER CODE BEGIN 2 */
    // ⚠️【重點】必須手動啟動 CAN 週邊！
    if (HAL_CAN_Start(&hcan1) != HAL_OK) {
        Error_Handler();
    }

    // 馬達運動與時間參數定義
    const float target_p = 1.5708f;               // 90 度 (rad)
    const float target_v = 0.17488f;              // 1.67 rpm (rad/s)
    const float total_time = target_p / target_v; // 約 8.98 秒
    const float Kp_run = 35.0f, Kd_run = 1.8f, T_max = 10.0f;
    uint32_t tick_start;
    /* USER CODE END 2 */

  /* Infinite loop */
    /* USER CODE BEGIN WHILE */
        while (1)
        {
            // --- 1. 開機首次執行保護：擷取開機實體角度並給予 0.5 秒阻尼 ---
            if (is_first_run) {
                uint32_t init_tick = HAL_GetTick();
                while ((HAL_GetTick() - init_tick) < 500) {
                    if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == GPIO_PIN_SET) {
                        disable_motor();
                        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_SET);
                        while (1) { HAL_Delay(1000); }
                    }
                    send_mit(0.0f, 0.0f, 0.0f, 1.0f, 0.0f);
                    update_motor_position(); // 【關鍵】持續更新 CAN 回傳
                    HAL_Delay(10);
                }
                start_p = motor_actual_p; // 【關鍵】把開機當下的實體位置當作第一個起點
                is_first_run = 0;
            }

            // --- 2. 按鈕檢查：若按下藍色按鈕 (PA0)，執行完全停機 ---
            if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == GPIO_PIN_SET) {
                for(int i = 0; i < 3; i++) {
                    disable_motor();
                    HAL_Delay(10);
                }
                HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13 | GPIO_PIN_15, GPIO_PIN_RESET);
                HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_SET);
                while (1) { HAL_Delay(1000); }
            }

            // --- 3. 階段 1：勻速旋轉 90 度 ---
            tick_start = HAL_GetTick();
            while (1) {
                if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == GPIO_PIN_SET) {
                    disable_motor();
                    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_SET);
                    while (1) { HAL_Delay(1000); }
                }

                float elapsed_s = (HAL_GetTick() - tick_start) / 1000.0f;
                if (elapsed_s >= total_time) break;

                float p_des = start_p + (target_v * elapsed_s);
                float t_ff  = T_max * sinf(p_des);

                if (send_mit(p_des, target_v, Kp_run, Kd_run, t_ff) != HAL_OK) {
                    goto ERROR_HANDLER;
                }

                update_motor_position(); // 【關鍵】旋轉過程持續更新角度解包
                HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_15); // 藍燈快閃
                HAL_Delay(10);
            }

            // --- 4. 階段 2：馬達洩力 5 秒 (可手動轉動) ---
                        tick_start = HAL_GetTick();
                        while ((HAL_GetTick() - tick_start) < 5000) {
                            if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == GPIO_PIN_SET) {
                                disable_motor();
                                HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_SET);
                                while (1) { HAL_Delay(1000); }
                            }

                            // 發送極小阻尼，允許手動旋轉
                            if (send_mit(0.0f, 0.0f, 0.0f, 0.1f, 0.0f) != HAL_OK) {
                                goto ERROR_HANDLER;
                            }
        }