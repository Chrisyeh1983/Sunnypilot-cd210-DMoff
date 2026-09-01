#!/usr/bin/env bash

# ---- 自訂背景服務：影像串流 + 莫名減速記錄器 -----------------------------
# custom helpers: MJPEG stream to the android box + longitudinal-source logger
# 整段丟進背景子行程，絕對不能擋到下面的 openpilot 啟動。
# 不要了就還原 /data/continue.sh.bak.20260901
(
  sleep 25                       # 等 openpilot / camerad 先起來 / let camerad come up
  export PYTHONPATH=/data/openpilot
  export OPS_CAM=narrow
  PY=/usr/local/venv/bin/python3
  pgrep -f /data/opstream.py >/dev/null || \
    setsid "$PY" /data/opstream.py >/data/opstream.log 2>&1 </dev/null &
  pgrep -f /data/decel_watch.py >/dev/null || \
    setsid "$PY" /data/decel_watch.py >/data/decel_watch.err 2>&1 </dev/null &
) >/dev/null 2>&1 &
# --------------------------------------------------------------------------

cd /data/openpilot
exec ./launch_openpilot.sh
