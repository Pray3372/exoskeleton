/*
 * force_trigger.h
 *
 * 「被推一下就自己轉 90° 然後放鬆」的行為模組（AK60-6，力控/MIT 模式，擴展 ID）
 *
 * 行為：
 *   RELAX  : 馬達完全放鬆 (Kp=0, Kd=IDLE_KD, t=0)，持續讀 0x29 回饋。
 *            當位置偏離靜止點 > TRIG_POS_DEG，或 |Iq| > TRIG_IQ_A，
 *            視為「感受到力」→ 記錄當下位置/電流/時間 → 進入 MOVE。
 *   MOVE   : 以 MIT 控制 (Kp=MOVE_KP, Kd=MOVE_KD) 平滑插值到
 *            觸發位置 ± MOVE_DEG（方向 = 被推的方向，可改成固定方向）。
 *            到達 (誤差 < REACH_TOL_DEG 持續 REACH_HOLD_MS) 或逾時 MOVE_TIMEOUT_MS
 *            → 進入 SETTLE。
 *   SETTLE : 送放鬆幀，等 SETTLE_MS 讓馬達停下來，重新取靜止點 → 回到 RELAX。
 *
 * 這是獨立的新檔案，不需要修改 AK.c / AK.h / disable_key.*。
 * 它自己接管 CAN 接收 (FIFO0 中斷)，解析馬達 0x29 回饋幀。
 *
 * main.c 需要你自己加的三行：
 *   #include "force_trigger.h"
 *   ForceTrig_Init(1);                    // DisableKey_Init() 之後
 *   if (!DisableKey_IsDisabled()) {
 *       ForceTrig_Step();                 // ★ 取代原本的 pack_cmd(1, 0, 0, 10, 1, 0);
 *   }
 *   （原本那行 pack_cmd 一定要拿掉，否則兩邊會互相打架）
 */

#ifndef FORCE_TRIGGER_H
#define FORCE_TRIGGER_H

#include <stdint.h>
#include "main.h"

/* ------------------------------------------------------------------ */
/*  可調參數（改這裡就好）                                             */
/* ------------------------------------------------------------------ */
#define FT_TRIG_POS_DEG      3.0f    /* 偏離靜止點超過幾度算「被推」          */
#define FT_TRIG_IQ_A         1.5f    /* 或 |Iq| 超過幾安培算「被推」(IDLE_KD=0 時幾乎不會觸發) */
#define FT_MOVE_DEG          90.0f   /* 觸發後要相對移動幾度                  */
#define FT_MOVE_DIR          0       /* 0 = 跟被推的方向, +1 = 固定正向, -1 = 固定負向 */
#define FT_MOVE_TIME_MS      1500    /* 插值走完 90° 的時間 (越長越溫和)      */
#define FT_MOVE_KP           20.0f   /* 移動時的剛性 (手冊範圍 0~500)         */
#define FT_MOVE_KD           1.0f    /* 移動時的阻尼 (手冊範圍 0~5)           */
#define FT_IDLE_KD           0.0f    /* 放鬆時的阻尼，0 = 完全無力；0.2 會有一點黏滯感 */
#define FT_REACH_TOL_DEG     3.0f    /* 到達判定誤差                          */
#define FT_REACH_HOLD_MS     200     /* 誤差內持續多久算到達                  */
#define FT_MOVE_TIMEOUT_MS   4000    /* 移動最長時間，超過就放棄並放鬆        */
#define FT_SETTLE_MS         600     /* 放鬆後等多久再重新武裝觸發            */
#define FT_CMD_PERIOD_MS     10      /* 送命令的週期                          */
#define FT_FB_TIMEOUT_MS     300     /* 回饋超過這麼久沒更新就視為斷線        */
#define FT_LOG_SIZE          64      /* 記錄最近幾次觸發                      */

/* 馬達端輸出軸位置的軟體極限 (rad)，手冊 P 範圍 ±12.56，留一點餘裕 */
#define FT_P_LIMIT_RAD       12.0f

/* ------------------------------------------------------------------ */
/*  型別                                                              */
/* ------------------------------------------------------------------ */
typedef enum {
    FT_STATE_RELAX = 0,
    FT_STATE_MOVE,
    FT_STATE_SETTLE,
} FT_State;

/* 馬達最新回饋（由 0x29 幀解出）*/
typedef struct {
    float    pos_deg;      /* 輸出端位置 (°)，多圈，斷電歸零                */
    float    spd_erpm;     /* 電氣轉速 ERPM (手冊原始單位)                 */
    float    spd_rad_s;    /* 輸出端角速度 (rad/s) = ERPM/極對數/減速比 → rad/s */
    float    iq_A;         /* Iq 電流 (A)                                  */
    int8_t   temp_C;       /* 驅動板溫度                                    */
    uint8_t  error;        /* 錯誤碼 0=OK 1過溫 2過流 3過壓 4欠壓 5編碼器 6MOS過溫 7堵轉 */
    uint8_t  disabled_ack; /* 收到 DATA[7]=0x77 (失能成功) 時置 1            */
    uint32_t tick;         /* 最後更新時間 (HAL_GetTick)                    */
    uint32_t count;        /* 累計收到幾幀                                  */
} FT_Feedback;

/* 每次「感受到力」的記錄 */
typedef struct {
    uint32_t tick;         /* 觸發時間 (ms)                                 */
    float    pos_deg;      /* 觸發時的位置                                  */
    float    iq_A;         /* 觸發時的電流                                  */
    float    spd_rad_s;    /* 觸發時的速度                                  */
    int8_t   dir;          /* 移動方向 +1/-1                                */
    float    target_deg;   /* 這次要去的目標位置                            */
    float    end_pos_deg;  /* 移動結束時的實際位置                          */
    uint8_t  reached;      /* 1 = 到達, 0 = 逾時放棄, 0xFF = 進行中          */
    uint8_t  clamped;      /* 1 = 目標被軟體極限夾住                        */
} FT_LogEntry;

/* ------------------------------------------------------------------ */
/*  API                                                               */
/* ------------------------------------------------------------------ */
void  ForceTrig_Init(uint8_t motor_id);   /* 在 HAL_CAN_Start() 之後呼叫 */
void  ForceTrig_Step(void);               /* 主迴圈每圈呼叫 (自帶 10 ms 節流) */
void  ForceTrig_Reset(void);              /* 強制回到 RELAX 並重新取靜止點 */

FT_State           ForceTrig_GetState(void);
const FT_Feedback *ForceTrig_GetFeedback(void);
uint8_t            ForceTrig_FeedbackOk(void);     /* 1 = 最近有收到回饋 */

uint32_t           ForceTrig_LogCount(void);       /* 累計觸發次數 */
const FT_LogEntry *ForceTrig_GetLog(uint32_t idx); /* idx 0 = 最新一筆, 最多 FT_LOG_SIZE 筆 */

#endif /* FORCE_TRIGGER_H */
