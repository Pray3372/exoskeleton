#!/usr/bin/env python3
"""
log_to_csv.py — 接收 STM32 (can_logger.c 精簡版) 的電流紀錄並存成 CSV。

    pip install pyserial
    python tools/log_to_csv.py COM5                 # Windows（115200）
    python tools/log_to_csv.py COM5 -o squat1.csv -t 20

結束（Ctrl+C 或 -t 秒數到）時印出：筆數、實際取樣頻率、Iq 電流 / 扭矩 的 max / RMS，
以及 Iq 超過門檻的時間比例（-w 設門檻，預設 ±8 A），方便判斷 MIT 的 Kp/Kd/t_ff 是否太猛。
"""
import argparse, csv, datetime as dt, math, os, sys, time

try:
    import serial
except ImportError:
    sys.exit("請先安裝 pyserial：pip install pyserial")

FIELDS = ["t_ms", "pos_deg", "spd_rpm", "iq_A", "torque_Nm", "temp_C", "err"]

ap = argparse.ArgumentParser()
ap.add_argument("port")
ap.add_argument("-b", "--baud", type=int, default=115200)
ap.add_argument("-o", "--out", default=None)
ap.add_argument("-t", "--seconds", type=float, default=0)
ap.add_argument("-w", "--warn-amp", type=float, default=8.0, help="Iq 警戒值 (A)")
a = ap.parse_args()

LOG_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "log")   # 專案/log
os.makedirs(LOG_DIR, exist_ok=True)
out = a.out or os.path.join(LOG_DIR, dt.datetime.now().strftime("canlog_%Y%m%d_%H%M%S.csv"))
ser = serial.Serial(a.port, a.baud, timeout=0.2)
print(f"開啟 {a.port} @ {a.baud} → {out}（Ctrl+C 停止）")

t, iq, tq, errs = [], [], [], {}
n = bad = 0
t0 = time.time()
with open(out, "w", newline="", encoding="utf-8") as f:
    w = csv.writer(f)
    w.writerow(FIELDS)
    try:
        while not (a.seconds and time.time() - t0 >= a.seconds):
            line = ser.readline().decode("ascii", "replace").strip()
            if not line or line.startswith("t_ms"):
                continue
            p = line.split(",")
            if len(p) != len(FIELDS):
                bad += 1
                continue
            try:
                t.append(int(p[0])); iq.append(float(p[3])); tq.append(float(p[4]))
            except ValueError:
                bad += 1
                continue
            errs[p[6]] = errs.get(p[6], 0) + 1
            w.writerow(p)
            n += 1
            if n % 500 == 0:
                print(f"  {n} 筆  Iq={iq[-1]:+.2f} A  pos={p[1]}°  T={p[5]}°C")
    except KeyboardInterrupt:
        pass
    finally:
        ser.close()

print(f"\n寫入 {n} 筆到 {out}（壞行 {bad}）")
if n >= 2:
    span = (t[-1] - t[0]) / 1000
    print(f"取樣：{span:.1f} s，平均 {(n - 1) / span:.1f} Hz")
    rms = math.sqrt(sum(x * x for x in iq) / n)
    over = sum(1 for x in iq if abs(x) > a.warn_amp) / n * 100
    print(f"Iq 電流：max |{max(iq, key=abs):+.2f}| A   RMS {rms:.2f} A   超過 ±{a.warn_amp} A 的時間 {over:.1f}%")
    print(f"扭矩  ：max |{max(tq, key=abs):+.2f}| N·m  RMS {math.sqrt(sum(x*x for x in tq)/n):.2f} N·m")
    if set(errs) - {"0"}:
        print("錯誤碼：", errs)
