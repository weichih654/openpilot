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
  'liveCalibration', 'carState', 'controlsState', 'carOutput',
  'frogpilotPlan', 'frogpilotCarState',
  'onroadEvents', 'frogpilotOnroadEvents',
  # raw IMU — 繞過 locationd 可能被凍住的 angVelCalib
  'gyroscope', 'accelerometer',
]

# 等待訊息抵達的設定：每次 poll 100ms，最多 20 次（~2s）
_POLL_TIMEOUT_MS = 100
_MAX_POLLS = 20


def _sensor_block(msg):
  """SensorEventData (union: gyro / gyroUncalibrated / acceleration / ...) → dict。"""
  which = msg.which()
  out = {"which": which, "timestamp": msg.timestamp, "source": str(msg.source)}
  variant = getattr(msg, which, None)
  if variant is not None and hasattr(variant, 'v'):
    out["v"] = list(variant.v)
    out["status"] = variant.status
  return out


def _messaging_stats(sm):
  """各 service 的 SubMaster 即時統計 — alive/valid/freq、最後一筆訊息間隔。"""
  out = {}
  for s in SERVICES:
    dts = list(sm.recv_dts.get(s, ()))
    out[s] = {
      "alive":          sm.alive.get(s),
      "valid":          sm.valid.get(s),
      "logMonoTimeNs":  sm.logMonoTime.get(s),
      "recvFrame":      sm.recv_frame.get(s),
      "recvDtCount":    len(dts),
      "recvDtMean":     (sum(dts) / len(dts)) if dts else None,
      "recvDtMin":      min(dts) if dts else None,
      "recvDtMax":      max(dts) if dts else None,
    }
  return out


def _capture(sm):
  """讀取所有診斷欄位。新增欄位：在此函式加一行即可。"""
  cs = sm['controlsState']
  lat_state = cs.lateralControlState
  torque_state = lat_state.torqueState if lat_state.which() == 'torqueState' else None

  return {
    "trigger_time": datetime.now().isoformat(),

    # 各 service 的訊息健康度（alive/valid/freq）— 判斷 paramsd all_checks 哪個 gate 失敗
    "messaging": _messaging_stats(sm),

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
      # paramsd L225 hardcode 為 True — 留著當對照
      "posenetValid":          sm['liveParameters'].posenetValid,
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

    # locationd 品質指標 — 用來判斷 status=uninitialized 的根因
    "liveLocationKalman": {
      "posenetOK":        sm['liveLocationKalman'].posenetOK,
      "status":           str(sm['liveLocationKalman'].status),
      "inputsOK":         sm['liveLocationKalman'].inputsOK,
      "gpsOK":            sm['liveLocationKalman'].gpsOK,
      "sensorsOK":        sm['liveLocationKalman'].sensorsOK,
      "deviceStable":     sm['liveLocationKalman'].deviceStable,
      "excessiveResets":  sm['liveLocationKalman'].excessiveResets,
      "timeSinceReset":   sm['liveLocationKalman'].timeSinceReset,
      # positionECEF.std 大於 50m 就會讓 status 變 UNINITIALIZED
      "positionECEFStd":  list(sm['liveLocationKalman'].positionECEF.std),
      "positionECEFValid":sm['liveLocationKalman'].positionECEF.valid,
      "orientationNEDStd":list(sm['liveLocationKalman'].calibratedOrientationNED.std),
      "angVelCalib":      list(sm['liveLocationKalman'].angularVelocityCalibrated.value),
    },

    # 校準狀態（controlsd 用來閘 lateral）
    "liveCalibration": {
      "calStatus": str(sm['liveCalibration'].calStatus),
      "calPerc":   sm['liveCalibration'].calPerc,
    },

    # 車輛當下狀態（含 paramsd.self.active 計算用到的 aEgo / steeringRateDeg）
    # 在 Mazda+TI 上 carState.steeringTorque 是 TI 硬體的 TI_TORQUE_SENSOR 讀值
    "carState": {
      "steeringAngleDeg": sm['carState'].steeringAngleDeg,
      "vEgo":             sm['carState'].vEgo,
      "steeringPressed":  sm['carState'].steeringPressed,
      "aEgo":             sm['carState'].aEgo,
      "steeringRateDeg":  sm['carState'].steeringRateDeg,
      "steeringTorque":   sm['carState'].steeringTorque,     # Mazda+TI: TI_TORQUE_SENSOR
      "steeringTorqueEps":sm['carState'].steeringTorqueEps,  # EPS 實際輸出扭矩
      "yawRate":          sm['carState'].yawRate,            # 車輛 CAN 上的 yaw rate
    },

    # 控制器輸出 + 當前 alert（鎖定為什麼 latActive=False）
    "controlsState": {
      "state":            str(cs.state),
      "enabled":          cs.enabled,
      "latActive":        cs.active,
      "alertText1":       cs.alertText1,
      "alertText2":       cs.alertText2,
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

    # 當下 fire 中的 events — 最精準告訴你為什麼 latActive=False
    "onroadEvents": [
      {
        "name":             str(e.name),
        "noEntry":          e.noEntry,
        "softDisable":      e.softDisable,
        "immediateDisable": e.immediateDisable,
        "userDisable":      e.userDisable,
        "warning":          e.warning,
        "permanent":        e.permanent,
      } for e in sm['onroadEvents']
    ],
    "frogpilotOnroadEvents": [
      {"name": str(e.name)} for e in sm['frogpilotOnroadEvents']
    ],

    # FrogPilot 自己的橫向 gate
    "frogpilotPlan": {
      "lateralCheck": sm['frogpilotPlan'].lateralCheck,
    },
    "frogpilotCarState": {
      "alwaysOnLateralEnabled": sm['frogpilotCarState'].alwaysOnLateralEnabled,
      "pauseLateral":           sm['frogpilotCarState'].pauseLateral,
      # Mazda TI 硬體狀態（非 Mazda / 無 TI 車種會是預設 0/False）
      "tiState":         sm['frogpilotCarState'].tiState,
      "tiViolation":     sm['frogpilotCarState'].tiViolation,
      "tiError":         sm['frogpilotCarState'].tiError,
      "tiRampDown":      sm['frogpilotCarState'].tiRampDown,
      "tiLkasAllowed":   sm['frogpilotCarState'].tiLkasAllowed,
      "tiVersion":       sm['frogpilotCarState'].tiVersion,
    },

    # 原始 IMU — locationd 凍住時，這裡是車身運動的 ground truth
    # SensorEventData 是 union；用 which() 區分 gyro / gyroUncalibrated
    "gyroscope": _sensor_block(sm['gyroscope']),
    "accelerometer": _sensor_block(sm['accelerometer']),
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
