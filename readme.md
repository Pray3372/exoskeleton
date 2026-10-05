# AK60-6 MIT Control (STM32F407)

使用 STM32F407 透過 CAN 匯流排，以 MIT（力控）模式控制 CubeMars AK60-6 V3.0 馬達，作為外骨骼關節驅動的開發與測試專案。

Control a CubeMars AK60-6 V3.0 actuator in MIT (force-control) mode over CAN with an STM32F407, for exoskeleton joint development and testing.

> 私人備份用專案 / Personal backup repository.

---

## 硬體 / Hardware

| 項目 Item | 說明 Description |
|---|---|
| MCU | STM32F407VGTx（STM32F4 Discovery） |
| 馬達 Motor | CubeMars AK60-6 V3.0，Driver ID = 1 |
| CAN | CAN1，**1 Mbps**，擴展 ID（Extended ID）<br>PD0 = CAN1_RX，PD1 = CAN1_TX，需外接 CAN 收發器 / external CAN transceiver required |
| UART | USART2，115200 bps，PA2 = TX，PA3 = RX（鍵盤指令 / keyboard commands） |
| 按鈕 Button | PA0 藍色按鈕 / Blue user button |
| LED | PD12–PD15（狀態指示 / status indication） |

## 操作 / Usage

| 輸入 Input | 動作 Action |
|---|---|
| UART 按 `S` | 送出 CAN 失能命令，主迴圈停止送控制命令 / Disable motor and stop sending commands |
| UART 按 `E` | 解除失能旗標，恢復控制 / Clear disable flag and resume control |
| 藍色按鈕 PA0 | 緊急停機（依目前 `main.c` 版本）/ Emergency stop (depends on current `main.c`) |

## 專案結構 / Project Structure

```
AK60-6/
├── AK60-6.ioc              # CubeMX 設定檔 / CubeMX configuration
├── Core/
│   ├── Inc/  Src/          # 主程式與使用者程式碼 / Application code
│   │   ├── main.c          # 主迴圈 / Main loop
│   │   ├── AK.c / AK.h     # AK60-6 MIT 封包與 CAN 收發 / MIT packing & CAN I/O
│   │   └── disable_key.c/h # UART 鍵盤失能功能 / UART disable key
│   └── Startup/            # 啟動組語 / Startup assembly
├── advance_outputs/        # 進階功能模組（尚未整合進主程式）/ Optional modules
│   ├── force_trigger.c/h   # 被推觸發 → 轉 90° → 放鬆 / Push-triggered 90° move
│   └── disable_key.c/h
├── mit_control.c/h         # MIT 控制草稿 / MIT control draft
├── Drivers/  Middlewares/  USB_HOST/   # ST HAL、CMSIS、USB Host（CubeMX 產生）
└── STM32F407VGTX_FLASH.ld / _RAM.ld    # Linker scripts
```

## MIT 控制參數範圍 / MIT Parameter Ranges

依 AK60-6 V3.0 手冊 / Per the AK60-6 V3.0 manual:

| 參數 Param | 最小 Min | 最大 Max | 單位 Unit | 位元 Bits |
|---|---|---|---|---|
| 位置 Position `p` | -12.56 | 12.56 | rad | 16 |
| 速度 Velocity `v` | -60 | 60 | rad/s | 12 |
| 剛性 `Kp` | 0 | 500 | – | 12 |
| 阻尼 `Kd` | 0 | 5 | – | 12 |
| 力矩 Torque `t_ff` | -12 | 12 | N·m | 12 |

MIT 模式 CAN ID：`(8 << 8) | DRIVER_ID`

```c
// pack_cmd(id, p_des, v_des, kp, kd, t_ff)
pack_cmd(1, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f);   // 零力矩、僅阻尼 / zero torque, damping only
```

## 建置與燒錄 / Build & Flash

1. 以 **STM32CubeIDE** 開啟（File → Open Projects from File System）/ Open with STM32CubeIDE.
2. Build（`Ctrl + B`），輸出於 `Debug/` / Output goes to `Debug/`.
3. 透過 ST-LINK 燒錄並執行 / Flash and run via ST-LINK.

> 若用 CubeMX 重新產生程式碼，自訂程式請寫在 `/* USER CODE BEGIN */ ... /* USER CODE END */` 之間，否則會被覆蓋。
> When regenerating code with CubeMX, keep custom code inside `USER CODE BEGIN/END` blocks.

## 使用 advance_outputs 模組 / Using `advance_outputs`

`force_trigger` 為獨立模組，整合方式（詳見 `force_trigger.h` 檔頭）：

```c
#include "force_trigger.h"

ForceTrig_Init(1);              // DisableKey_Init() 之後 / after DisableKey_Init()

while (1) {
    if (!DisableKey_IsDisabled()) {
        ForceTrig_Step();       // 取代原本的 pack_cmd() / replaces pack_cmd()
    }
}
```

注意：使用時要移除主迴圈原本的 `pack_cmd()`，且 `force_trigger.c` 自行定義了 `CAN1_RX0_IRQHandler`，若 CubeMX 也產生同名 handler 需擇一保留。

Note: remove the original `pack_cmd()` call, and keep only one `CAN1_RX0_IRQHandler` definition.

## 安全注意事項 / Safety

- 首次測試請先用低 `Kp`、低力矩，並固定好馬達 / Start with low `Kp` and torque, and secure the motor.
- 隨時保留失能手段（UART `S` 或 PA0 按鈕）/ Always keep a way to disable the motor.
- 馬達位置於斷電後歸零（臨時原點）/ Position resets to zero after power-off (temporary origin).

## 授權 / License

本專案的自寫程式碼僅供個人使用。`Drivers/` 與 `Middlewares/` 內的 STMicroelectronics / ARM 元件依其各自資料夾中的 `LICENSE.txt` 授權。

User-written code is for personal use only. STMicroelectronics / ARM components under `Drivers/` and `Middlewares/` are licensed under the `LICENSE.txt` in their respective folders.
