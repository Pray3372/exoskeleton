/*
 * force_trigger.c  — 見 force_trigger.h 說明
 *
 * 依賴：
 *   AK.h        : pack_cmd()、hcan1
 *   HAL CAN     : 這裡自己開 FIFO0 中斷並定義 CAN1_RX0_IRQHandler，
 *                 不需要在 CubeMX 勾 CAN1 RX0 interrupt，也不用改 stm32f4xx_it.c。
 *                 （若你之後在 CubeMX 勾了，stm32f4xx_it.c 會產生同名 handler，
 *                   屆時把本檔的 CAN1_RX0_IRQHandler 註解掉即可。）
 */

#include "force_trigger.h"
#include "AK.h"
#include <math.h>

/* ------------------------------------------------------------------ */
/*  AK60-6 常數（手冊 4.2 參數表 / 3.1.5.3 單位設定）                   */
/* ------------------------------------------------------------------ */
#define AK_POLE_PAIRS      21.0f
#define AK_GEAR_RATIO      6.0f
#define DEG2RAD            0.017453292519943295f
#define RAD2DEG            57.29577951308232f

/* 回饋幀功能 ID (手冊 4.3.1) */
#define AK_FB_STATUS_ID    0x29
#define AK_FB_POS32_ID     0x2A
#define AK_FB_START_ID     0x2C

/* ------------------------------------------------------------------ */
/*  模組狀態                                                          */
/* ------------------------------------------------------------------ */
static uint8_t     s_motor_id = 1;
static FT_State    s_state    = FT_STATE_RELAX;
static volatile FT_Feedback s_fb;        /* 由中斷更新 */

static float    s_rest_pos_deg  = 0.0f;  /* 靜止點 */
static uint8_t  s_armed         = 0;     /* 靜止點已取得、可以偵測觸發 */
static uint32_t s_state_tick    = 0;     /* 進入目前狀態的時間 */
static uint32_t s_last_cmd_tick = 0;
static uint32_t s_last_step_tick = 0;
static uint32_t s_reach_tick    = 0;     /* 進入到達容差的起始時間 */

/* MOVE 用 */
static float    s_move_start_deg = 0.0f;
static float    s_move_target_deg = 0.0f;

/* 記錄 */
static FT_LogEntry s_log[FT_LOG_SIZE];
static uint32_t    s_log_count = 0;      /* 累計次數 */

/* ------------------------------------------------------------------ */
/*  小工具                                                            */
/* ------------------------------------------------------------------ */
static inline float clampf(float x, float lo, float hi)
{
    return x < lo ? lo : (x > hi ? hi : x);
}

static void send_relax(void)
{
    /* Kp=0, Kd=IDLE_KD, t=0：p_des 給目前位置只是為了讓 Kd 項不會和「速度 0」以外的東西打架，
     * Kp=0 時 p_des 其實無作用。 */
    pack_cmd(s_motor_id, s_fb.pos_deg * DEG2RAD, 0.0f, 0.0f, FT_IDLE_KD, 0.0f);
}

static void send_hold(float p_deg)
{
    pack_cmd(s_motor_id, p_deg * DEG2RAD, 0.0f, FT_MOVE_KP, FT_MOVE_KD, 0.0f);
}

static FT_LogEntry *log_new(void)
{
    FT_LogEntry *e = &s_log[s_log_count % FT_LOG_SIZE];
    s_log_count++;
    return e;
}

static FT_LogEntry *log_latest(void)
{
    if (s_log_count == 0) return 0;
    return &s_log[(s_log_count - 1) % FT_LOG_SIZE];
}

static void enter_state(FT_State st, uint32_t now)
{
    s_state = st;
    s_state_tick = now;
    s_reach_tick = 0;
}

/* ------------------------------------------------------------------ */
/*  CAN 接收                                                          */
/* ------------------------------------------------------------------ */
static void parse_feedback(const CAN_RxHeaderTypeDef *hdr, const uint8_t *d)
{
    if (hdr->IDE != CAN_ID_EXT) return;

    uint32_t func = (hdr->ExtId >> 8) & 0x1FFFFF;
    uint8_t  id   =  hdr->ExtId & 0xFF;
    if (id != s_motor_id) return;

    if (func == AK_FB_STATUS_ID && hdr->DLC >= 8) {
        int16_t pos_i = (int16_t)((d[0] << 8) | d[1]);
        int16_t spd_i = (int16_t)((d[2] << 8) | d[3]);
        int16_t cur_i = (int16_t)((d[4] << 8) | d[5]);

        s_fb.pos_deg   = pos_i * 0.1f;
        s_fb.spd_erpm  = spd_i * 10.0f;
        /* ERPM → 輸出端 rpm → rad/s */
        s_fb.spd_rad_s = (s_fb.spd_erpm / AK_POLE_PAIRS / AK_GEAR_RATIO) * (2.0f * 3.14159265f / 60.0f);
        s_fb.iq_A      = cur_i * 0.01f;
        s_fb.temp_C    = (int8_t)d[6];
        s_fb.error     = d[7];
        if (d[7] == 0x77) s_fb.disabled_ack = 1;   /* 手冊 4.1.8：失能成功回傳 */
        s_fb.tick      = HAL_GetTick();
        s_fb.count++;
    }
    /* 0x2A (int32 位置) 若有開啟，這裡可再解析；目前用 0x29 的 int16 就夠 */
}

/* HAL 弱符號：FIFO0 有幀進來 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef hdr;
    uint8_t data[8];
    while (HAL_CAN_GetRxFifoFillLevel(hcan, CAN_RX_FIFO0) > 0) {
        if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &hdr, data) == HAL_OK) {
            parse_feedback(&hdr, data);
        } else {
            break;
        }
    }
}

/* 中斷向量：startup 檔裡是弱符號，這裡直接定義即可 */
void CAN1_RX0_IRQHandler(void)
{
    HAL_CAN_IRQHandler(&hcan1);
}

/* 保險：就算中斷沒開，主迴圈也把 FIFO 掏空 */
static void poll_rx(void)
{
    HAL_CAN_RxFifo0MsgPendingCallback(&hcan1);
}

/* ------------------------------------------------------------------ */
/*  API                                                               */
/* ------------------------------------------------------------------ */
void ForceTrig_Init(uint8_t motor_id)
{
    s_motor_id = motor_id;

    /* 開 FIFO0 接收中斷 */
    HAL_NVIC_SetPriority(CAN1_RX0_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(CAN1_RX0_IRQn);
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);

    ForceTrig_Reset();
}

void ForceTrig_Reset(void)
{
    uint32_t now = HAL_GetTick();
    s_armed = 0;
    enter_state(FT_STATE_SETTLE, now);   /* 先走一次 SETTLE 再武裝，避免開機誤觸 */
    s_last_step_tick = now;
}

FT_State ForceTrig_GetState(void)            { return s_state; }
const FT_Feedback *ForceTrig_GetFeedback(void){ return (const FT_Feedback *)&s_fb; }
uint32_t ForceTrig_LogCount(void)             { return s_log_count; }

uint8_t ForceTrig_FeedbackOk(void)
{
    return (s_fb.count > 0) && ((HAL_GetTick() - s_fb.tick) < FT_FB_TIMEOUT_MS);
}

const FT_LogEntry *ForceTrig_GetLog(uint32_t idx)
{
    if (idx >= s_log_count || idx >= FT_LOG_SIZE) return 0;
    return &s_log[(s_log_count - 1 - idx) % FT_LOG_SIZE];
}

/* ------------------------------------------------------------------ */
/*  主狀態機                                                          */
/* ------------------------------------------------------------------ */
void ForceTrig_Step(void)
{
    uint32_t now = HAL_GetTick();

    /* 如果主迴圈很久沒呼叫我（例如剛從失能狀態解除），重新來過，避免用舊的靜止點誤觸 */
    if ((now - s_last_step_tick) > 200) {
        ForceTrig_Reset();
        now = HAL_GetTick();
    }
    s_last_step_tick = now;

    poll_rx();

    /* 10 ms 節流：命令與判斷都在這個週期做 */
    if ((now - s_last_cmd_tick) < FT_CMD_PERIOD_MS) return;
    s_last_cmd_tick = now;

    uint8_t fb_ok = ForceTrig_FeedbackOk();

    switch (s_state) {

    /* ---------------------------------------------------------- */
    case FT_STATE_SETTLE:
        send_relax();
        if (fb_ok && (now - s_state_tick) >= FT_SETTLE_MS) {
            s_rest_pos_deg = s_fb.pos_deg;
            s_armed = 1;
            enter_state(FT_STATE_RELAX, now);
        }
        break;

    /* ---------------------------------------------------------- */
    case FT_STATE_RELAX: {
        send_relax();
        if (!fb_ok) { s_armed = 0; break; }      /* 沒回饋就不武裝 */
        if (!s_armed) { enter_state(FT_STATE_SETTLE, now); break; }

        float dev = s_fb.pos_deg - s_rest_pos_deg;
        uint8_t pushed = (fabsf(dev) > FT_TRIG_POS_DEG) || (fabsf(s_fb.iq_A) > FT_TRIG_IQ_A);
        if (!pushed) break;

        /* ---- 感受到力：記錄 + 決定方向與目標 ---- */
        int8_t dir;
        if (FT_MOVE_DIR > 0)       dir = +1;
        else if (FT_MOVE_DIR < 0)  dir = -1;
        else                       dir = (dev >= 0.0f) ? +1 : -1;

        float start  = s_fb.pos_deg;
        float target = start + dir * FT_MOVE_DEG;
        uint8_t clamped = 0;
        float lim_deg = FT_P_LIMIT_RAD * RAD2DEG;
        if (target >  lim_deg) { target =  lim_deg; clamped = 1; }
        if (target < -lim_deg) { target = -lim_deg; clamped = 1; }

        FT_LogEntry *e = log_new();
        e->tick        = now;
        e->pos_deg     = start;
        e->iq_A        = s_fb.iq_A;
        e->spd_rad_s   = s_fb.spd_rad_s;
        e->dir         = dir;
        e->target_deg  = target;
        e->end_pos_deg = start;
        e->reached     = 0xFF;
        e->clamped     = clamped;

        s_move_start_deg  = start;
        s_move_target_deg = target;
        enter_state(FT_STATE_MOVE, now);
        break;
    }

    /* ---------------------------------------------------------- */
    case FT_STATE_MOVE: {
        uint32_t el = now - s_state_tick;

        /* 線性插值 (可改成 S 曲線)：從觸發點平滑走到目標，避免 Kp 瞬間拉扯 */
        float a = (FT_MOVE_TIME_MS > 0) ? clampf((float)el / (float)FT_MOVE_TIME_MS, 0.0f, 1.0f) : 1.0f;
        float p_cmd = s_move_start_deg + (s_move_target_deg - s_move_start_deg) * a;
        send_hold(p_cmd);

        FT_LogEntry *e = log_latest();

        /* 到達判定 */
        if (fb_ok && a >= 1.0f) {
            float err = fabsf(s_fb.pos_deg - s_move_target_deg);
            if (err < FT_REACH_TOL_DEG) {
                if (s_reach_tick == 0) s_reach_tick = now;
                if ((now - s_reach_tick) >= FT_REACH_HOLD_MS) {
                    if (e) { e->reached = 1; e->end_pos_deg = s_fb.pos_deg; }
                    enter_state(FT_STATE_SETTLE, now);
                    break;
                }
            } else {
                s_reach_tick = 0;
            }
        }

        /* 逾時或回饋斷線 → 放棄，放鬆 */
        if (el >= FT_MOVE_TIMEOUT_MS || !fb_ok) {
            if (e) { e->reached = 0; e->end_pos_deg = s_fb.pos_deg; }
            enter_state(FT_STATE_SETTLE, now);
        }
        break;
    }
    }
}
