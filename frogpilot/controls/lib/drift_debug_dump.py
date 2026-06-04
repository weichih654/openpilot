#!/usr/bin/env python3
"""手動觸發的側向飄移診斷快照。

由方向盤 distance/LKAS 鈕觸發（見 frogpilot_card.py）。每次觸發在背景執行緒
建立一次性 SubMaster，抓取與飄移相關的關鍵 in-memory 狀態，寫成一個獨立的
JSON 檔到 realdata log 目錄。

未來要新增記錄欄位：只需在 _capture() 加一行即可。
"""
import json
import threading
from datetime import datetime
from pathlib import Path

import cereal.messaging as messaging
from openpilot.common.swaglog import cloudlog
from openpilot.system.hardware.hw import Paths

# 車機：/data/media/0/realdata/drift_debug/    PC：~/.comma/media/0/realdata/drift_debug/
DEBUG_DUMP_DIR = Path(Paths.log_root()) / "drift_debug"

# 需要訂閱的 cereal services
SERVICES = [
  'liveParameters', 'liveTorqueParameters', 'liveLocationKalman',
  'carState', 'controlsState', 'carOutput',
]

# 等待訊息抵達的設定：每次 poll 100ms，最多 20 次（~2s）
_POLL_TIMEOUT_MS = 100
_MAX_POLLS = 20


def _capture(sm):
  """讀取所有診斷欄位。新增欄位：在此函式加一行即可。"""
  cs = sm['controlsState']
  lat_state = cs.lateralControlState
  torque_state = lat_state.torqueState if lat_state.which() == 'torqueState' else None

  return {
    "trigger_time": datetime.now().isoformat(),

    # 最核心：paramsd 的 angleOffset，飄移時懷疑此值跑偏
    "liveParameters": {
      "angleOffsetDeg":        sm['liveParameters'].angleOffsetDeg,
      "angleOffsetAverageDeg": sm['liveParameters'].angleOffsetAverageDeg,
      "angleOffsetFastStd":    sm['liveParameters'].angleOffsetFastStd,
      "angleOffsetAverageStd": sm['liveParameters'].angleOffsetAverageStd,
      "steerRatio":            sm['liveParameters'].steerRatio,
      "stiffnessFactor":       sm['liveParameters'].stiffnessFactor,
      "roll":                  sm['liveParameters'].roll,
      "valid":                 sm['liveParameters'].valid,
      "sensorValid":           sm['liveParameters'].sensorValid,
    },

    # torqued 的線上學習結果
    "liveTorqueParameters": {
      "latAccelFactorRaw":           sm['liveTorqueParameters'].latAccelFactorRaw,
      "latAccelOffsetRaw":           sm['liveTorqueParameters'].latAccelOffsetRaw,
      "latAccelFactorFiltered":      sm['liveTorqueParameters'].latAccelFactorFiltered,
      "latAccelOffsetFiltered":      sm['liveTorqueParameters'].latAccelOffsetFiltered,
      "frictionCoefficientFiltered": sm['liveTorqueParameters'].frictionCoefficientFiltered,
      "liveValid":                   sm['liveTorqueParameters'].liveValid,
      "useParams":                   sm['liveTorqueParameters'].useParams,
    },

    # locationd 品質指標（GPS 中斷時 posenetOK 會變 False）
    "liveLocationKalman": {
      "posenetOK":   sm['liveLocationKalman'].posenetOK,
      "status":      str(sm['liveLocationKalman'].status),
      "angVelCalib": list(sm['liveLocationKalman'].angularVelocityCalibrated.value),
    },

    # 車輛當下狀態
    "carState": {
      "steeringAngleDeg": sm['carState'].steeringAngleDeg,
      "vEgo":             sm['carState'].vEgo,
      "steeringPressed":  sm['carState'].steeringPressed,
    },

    # 控制器輸出
    "controlsState": {
      "latActive":        cs.active,
      "desiredCurvature": cs.desiredCurvature,
      "latStateWhich":    lat_state.which(),
      "torqueError":      torque_state.error               if torque_state else None,
      "torqueI":          torque_state.i                   if torque_state else None,
      "actualLatAccel":   torque_state.actualLateralAccel  if torque_state else None,
      "desiredLatAccel":  torque_state.desiredLateralAccel if torque_state else None,
    },

    "carOutput": {
      "steer": sm['carOutput'].actuatorsOutput.steer,
    },
  }


def _worker():
  try:
    # SubMaster 必須完全在這條背景執行緒內建立與使用（msgq socket 非 thread-safe）
    sm = messaging.SubMaster(SERVICES)

    # 單次 update() 不保證收到訊息（conflate=True，socket 剛建立第一個 poll 可能還沒到）。
    # 用 sm.updated[...] 當新鮮度判斷，loop 直到收到所有 service。
    for _ in range(_MAX_POLLS):
      sm.update(_POLL_TIMEOUT_MS)
      if all(sm.updated[s] or sm.seen[s] for s in SERVICES):
        break

    missing = [s for s in SERVICES if not sm.seen[s]]
    if missing:
      cloudlog.warning(f"drift debug dump: never received {missing}; dumping anyway with defaults")

    data = _capture(sm)
    data["_missing_services"] = missing

    DEBUG_DUMP_DIR.mkdir(parents=True, exist_ok=True)
    ts = datetime.now().strftime("%Y%m%d_%H%M%S_%f")[:-3]
    out = DEBUG_DUMP_DIR / f"drift_debug_{ts}.json"
    with open(out, "w") as f:
      json.dump(data, f, indent=2)
    cloudlog.info(f"drift debug dump written to {out}")
  except Exception:
    cloudlog.exception("drift debug dump failed")


def trigger_drift_debug_dump():
  """在 button handler 中呼叫此函式，立即返回，dump 在背景執行。"""
  threading.Thread(target=_worker, daemon=True).start()
