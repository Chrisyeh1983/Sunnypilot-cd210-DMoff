"""
opmodel - project openpilot's modelV2 lane lines / path into stream-image pixels.
把 openpilot 模型的車道線、路徑投影到串流影像的像素座標，給網頁畫圖層用。

畫法、顏色、寬度全部照抄車機的
`selfdrive/ui/mici/onroad/model_renderer.py`，不要自己發明。
"""
import json, os, threading, time
import numpy as np

from openpilot.cereal import messaging
from openpilot.common.params import Params
from openpilot.common.transformations.camera import DEVICE_CAMERAS, view_frame_from_device_frame
from openpilot.common.transformations.orientation import rot_from_euler

PATH_OFFSET_Z = 1.22      # HEIGHT_INIT，把路徑貼到地面 / device height
PATH_HALF_W = 0.9
MAX_DIST = 100.0
MIN_X = 1.0               # 太近的點投影會爆炸 / points too close blow up the projection
CAM = os.getenv("OPS_CAM", "narrow").lower()   # 要跟 opstream 用同一顆 / must match opstream

# 車機 LANE_LINE_COLORS：接手時最近兩條是綠的 / green when engaged
GREEN = (0, 255, 64)
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)

_state = {"json": b'{"ready":false}', "ts": 0.0}
_slock = threading.Lock()

# 信心球顏色，照抄 mici/onroad/confidence_ball.py / straight from the car's confidence_ball.py
BALL_GREEN = ("rgb(0,255,204)", "rgb(0,255,38)")      # conf > 0.5
BALL_AMBER = ("rgb(255,200,0)", "rgb(255,115,0)")     # conf > 0.2
BALL_RED = ("rgb(255,0,21)", "rgb(255,0,89)")         # 其餘
BALL_OVERRIDE = ("rgb(255,255,255)", "rgb(82,82,82)")
BALL_OFF = ("rgb(50,50,50)", "rgb(13,13,13)")
BALL_LAT = ("rgb(0,200,200)", "rgb(0,200,200)")       # BORDER_COLORS_SP LAT_ONLY
BALL_LONG = ("rgb(150,28,168)", "rgb(150,28,168)")    # BORDER_COLORS_SP LONG_ONLY


def _ui_status(sm):
    """照抄 ui_state.py + sunnypilot/ui_state.py 的 update_status。"""
    ss = sm['selfdriveState']
    state = str(ss.state)
    try:
        mads = sm['selfdriveStateSP'].mads
    except Exception:
        return "override" if state in ("preEnabled", "overriding") else \
               ("engaged" if ss.enabled else "disengaged")
    if state == "preEnabled":
        return "override"
    if state == "overriding":
        if not mads.available:
            return "override"
        if any(e.overrideLongitudinal for e in sm['onroadEvents']):
            return "override"
    if str(mads.state) in ("paused", "overriding"):
        return "override"
    if not mads.available:
        return "engaged" if ss.enabled else "disengaged"
    if not mads.enabled and not ss.enabled:
        return "disengaged"
    if mads.enabled and ss.enabled:
        return "engaged"
    if mads.enabled:
        return "lat_only"
    return "long_only"


def _conf_target(status, meta):
    """信心值＝1 - 最大脫離機率。未接手時 -0.5，球會沉到畫面外（車機就是這樣藏球的）。"""
    dp = meta.disengagePredictions
    brake = 1.0 - max(list(dp.brakeDisengageProbs) or [1.0])
    steer = 1.0 - max(list(dp.steerOverrideProbs) or [1.0])
    if status == "disengaged":
        return -0.5
    if status == "lat_only":
        return steer
    if status == "long_only":
        return brake
    return brake * steer


# 警示底色，照抄 alert_renderer.py 的 ALERT_COLORS / alert background colors
ALERT_COLORS = {0: (0, 0, 0), 1: (255, 115, 0), 2: (255, 0, 21)}   # normal / userPrompt / critical


def _alert(ss):
    """把 selfdriveState 的警示原樣送出去，畫法交給網頁。"""
    try:
        size = int(getattr(ss.alertSize, "raw", ss.alertSize))
    except Exception:
        return None
    if size == 0:
        return None
    try:
        status = int(getattr(ss.alertStatus, "raw", ss.alertStatus))
    except Exception:
        status = 0
    at = str(ss.alertType or "")
    ev = at.split("/")[0]
    # 底圖高度（車機 content rect 高 240）/ background height on the car's 240px view
    if ev in ("preLaneChangeLeft", "preLaneChangeRight", "laneChange"):
        bgh = 140.0 / 240.0
    elif ev == "laneChangeBlocked":
        bgh = 200.0 / 240.0
    else:
        bgh = 1.0
    # 圖示：方向燈=閃爍箭頭，被擋=盲點圖 / icon: blinking arrow, or blind-spot glyph
    icon, side = None, None
    if ev == "preLaneChangeLeft":
        icon, side = "ts", "left"
    elif ev == "preLaneChangeRight":
        icon, side = "ts", "right"
    elif ev == "laneChange":
        icon, side = "ts", None          # 沿用上一次的方向 / keep last side
    elif ev == "laneChangeBlocked":
        icon, side = "bs", None
    # steerRequired 會把方向盤圖示換成紅色警告版 / swaps the wheel icon to the critical one
    try:
        steer_req = str(ss.alertHudVisual) == "steerRequired"
    except Exception:
        steer_req = False
    c = ALERT_COLORS.get(status, ALERT_COLORS[0])
    return {"steerRequired": steer_req,
            "t1": str(ss.alertText1 or "").lower(), "t2": str(ss.alertText2 or "").lower(),
            "size": size, "status": status, "ev": ev, "bgh": round(bgh, 4),
            "icon": icon, "side": side,
            "bg": "%d,%d,%d" % c}


ACCEL_G = 9.81
DEFAULT_MAX_LAT_ACCEL = 3.0

# 狀態邊框顏色，照抄 onroad/augmented_road_view.py 的 BORDER_COLORS
BORDER_COLORS = {"disengaged": "18,40,57",     # 深藍 / blue
                 "override": "137,146,141",    # 灰 / gray
                 "engaged": "22,127,64",       # 綠 / green
                 "lat_only": "0,200,200",      # 青 / cyan
                 "long_only": "150,28,168"}    # 紫 / purple


def _torque(sm, max_lat_accel):
    """照抄 torque_bar.py TorqueBar._update_state：估 AI 用掉多少橫向扭力（-1~1）。"""
    cs = sm['controlsState']
    which = None
    try:
        which = cs.lateralControlState.which()
    except Exception:
        pass
    if which in ('angleState', 'curvatureState'):
        car_state = sm['carState']
        v = float(car_state.vEgo)
        actual = float(cs.curvature) * v * v
        desired = float(cs.desiredCurvature) * v * v
        accel_diff = desired - actual
        try:
            roll = float(sm['vehicleParameters'].roll)
        except Exception:
            roll = 0.0
        # 低速時 roll 估不準，按車速漸進帶入 / roll is noisy near standstill
        roll_comp = roll * ACCEL_G * float(np.interp(v, [5, 15], [0.0, 1.0]))
        lat = actual - roll_comp
        if not bool(sm['carControl'].latActive):
            return 0.0
        return float(np.clip((lat + accel_diff) / max(max_lat_accel, 1e-3), -1.0, 1.0))
    return float(-sm['carOutput'].actuatorsOutput.torque)


def _ball_colors(status, conf):
    if status in ("engaged",):
        return BALL_GREEN if conf > 0.5 else (BALL_AMBER if conf > 0.2 else BALL_RED)
    if status == "lat_only":
        return BALL_LAT
    if status == "long_only":
        return BALL_LONG
    if status == "override":
        return BALL_OVERRIDE
    return BALL_OFF


def _project(T, pts3, ds):
    """3D 車體座標 -> 串流影像像素。不丟點，讓 canvas 自己裁。"""
    if pts3.shape[0] == 0:
        return np.empty((0, 2), np.float32)
    p = T @ pts3.T
    z = np.where(np.abs(p[2]) < 1e-6, 1e-6, p[2])
    return (p[:2] / z).T / ds


def _pack(a):
    return [[round(float(x), 1), round(float(y), 1)] for x, y in a]


def _poly(T, pts3, half_w, z_off, ds):
    """照抄 _map_line_to_polygon：左右各推 half_w 公尺，繞一圈成多邊形。
    這就是「近粗遠細」的來源，用固定像素寬度畫線是錯的。"""
    if pts3.shape[0] < 2 or half_w <= 0:
        return []
    off = np.array([0.0, 0.0, z_off], np.float32)
    left = _project(T, pts3 + off + np.array([0, -half_w, 0], np.float32), ds)
    right = _project(T, pts3 + off + np.array([0, half_w, 0], np.float32), ds)
    return _pack(np.vstack((left, right[::-1])))


def _rgba(rgb, alpha):
    return "rgba(%d,%d,%d,%.3f)" % (rgb[0], rgb[1], rgb[2], alpha)


def worker(get_size, hz=15.0):
    services = ['modelV2', 'extrinsicsCalibration', 'deviceState', 'wideRoadCameraState',
                'selfdriveState', 'radarState', 'carState', 'onroadEvents',
                'controlsState', 'carControl', 'carOutput', 'vehicleParameters']
    try:
        sm = messaging.SubMaster(services + ['selfdriveStateSP'])
    except Exception:
        sm = messaging.SubMaster(services)      # 沒有 MADS 的分支 / branch without MADS
    params = Params()
    try:
        camera_offset = float(params.get("CameraOffset", return_default=True) or 0.0)
    except Exception:
        camera_offset = 0.0
    try:
        is_metric = bool(params.get_bool("IsMetric"))
    except Exception:
        is_metric = True
    cam = None
    rainbow, rb_t = False, 0.0
    period = 1.0 / hz
    # FirstOrderFilter(-0.5, rc=0.5, dt) 照抄車機 / same smoothing as the car
    conf_x, conf_k = -0.5, period / (0.5 + period)
    v_cluster_seen = False
    # 盲點圖示淡入淡出 FirstOrderFilter(0, rc=0.15) / blind spot fade
    bs_l, bs_r, bs_k = 0.0, 0.0, period / (0.15 + period)
    show_ts, show_bs = True, True
    # 扭力弧線 / 方向盤圖示的濾波器（rc 照抄車機）/ same rc values as the car
    tq_x, tq_k = 0.0, period / (0.1 + period)
    tqa_x = 0.0
    wa_x, wa_k = 0.0, period / (0.05 + period)
    wy_x = 0.0
    # 加速/煞車力條：車機是每格 (v-x)/5 @20fps ≒ rc 0.2s / rocket_fuel smoothing
    rf_x, rf_k = 0.0, period / (0.2 + period)
    show_rf = True
    try:
        from opendbc.car.structs import car as _car
        with _car.CarParams.from_bytes(params.get("CarParamsPersistent")) as _cp:
            max_lat_accel = float(_cp.maxLateralAccel)
    except Exception:
        max_lat_accel = DEFAULT_MAX_LAT_ACCEL
    if not max_lat_accel or max_lat_accel <= 0:
        max_lat_accel = DEFAULT_MAX_LAT_ACCEL
    while True:
        t0 = time.time()
        sm.update(200)
        w, h, ds = get_size()
        if not w:
            time.sleep(0.3)
            continue
        if cam is None:
            try:
                cam = DEVICE_CAMERAS[(str(sm['deviceState'].deviceType),
                                      str(sm['wideRoadCameraState'].sensor))]
            except Exception:
                cam = DEVICE_CAMERAS.get(("mici", "os04c10")) or list(DEVICE_CAMERAS.values())[0]
        # 窄角(車機螢幕用的)不套 wideFromDeviceEuler / narrow cam skips the wide euler
        intr = cam.wide_road.intrinsics if CAM == "wide" else cam.narrow_road.intrinsics

        calib = sm['extrinsicsCalibration']
        try:
            rpy = np.array(calib.rpyCalib, dtype=np.float64)
            wfd = np.array(calib.wideFromDeviceEuler, dtype=np.float64)
            if len(rpy) != 3:
                rpy = np.zeros(3)
            if len(wfd) != 3:
                wfd = np.zeros(3)
            view = view_frame_from_device_frame @ rot_from_euler(rpy)
            if CAM == "wide":
                view = view_frame_from_device_frame @ rot_from_euler(wfd) @ rot_from_euler(rpy)
        except Exception:
            view = view_frame_from_device_frame.copy()
        T = intr @ view

        if time.time() - rb_t > 2.0:          # 每 2 秒跟車機的設定對一次 / follow the car's toggle
            rb_t = time.time()
            try:
                rainbow = bool(params.get_bool("RainbowMode"))
            except Exception:
                rainbow = False
            try:
                show_ts = bool(params.get_bool("ShowTurnSignals"))
                show_bs = bool(params.get_bool("BlindSpot"))
                show_rf = bool(params.get_bool("RocketFuel"))
            except Exception:
                show_ts, show_bs, show_rf = True, True, True

        m = sm['modelV2']
        engaged = bool(sm['selfdriveState'].enabled)

        # 狀態 / 信心球 / 車速 ---------------------------------------------
        try:
            status = _ui_status(sm)
        except Exception:
            status = "engaged" if engaged else "disengaged"
        try:
            conf_x += conf_k * (_conf_target(status, m.meta) - conf_x)
        except Exception:
            conf_x += conf_k * (-0.5 - conf_x)
        ball_top, ball_bot = _ball_colors(status, conf_x)

        cs = sm['carState']
        v_cluster = float(cs.vEgoCluster)
        v_cluster_seen = v_cluster_seen or v_cluster != 0.0
        v_ego = v_cluster if v_cluster_seen else float(cs.vEgo)
        speed = max(0.0, v_ego * (3.6 if is_metric else 2.236936))

        # 方向燈 / 盲點 -----------------------------------------------------
        bs_l += bs_k * ((1.0 if cs.leftBlindspot else 0.0) - bs_l)
        bs_r += bs_k * ((1.0 if cs.rightBlindspot else 0.0) - bs_r)

        # 扭力弧線 + 方向盤圖示 ---------------------------------------------
        try:
            tq_x += tq_k * (_torque(sm, max_lat_accel) - tq_x)
        except Exception:
            tq_x += tq_k * (0.0 - tq_x)
        # 弧線只在「有橫向控制」時亮 / bar shows only when lateral control is on
        tqa_x += tq_k * ((0.0 if status in ("disengaged", "long_only") else 1.0) - tqa_x)

        alert = _alert(sm['selfdriveState'])
        wheel_critical = bool(alert and alert.get("steerRequired"))
        bs_detected = show_bs and (bs_l > 0.01 or bs_r > 0.01)
        if wheel_critical:
            wa_t, wy_t = 1.0, 0.0
        elif status == "disengaged" or bs_detected:
            wa_t, wy_t = 0.0, 25.0      # 淡出並往下滑（圖示高 50 的一半）/ fade out and slide down
        else:
            wa_t, wy_t = 0.9, 0.0
        wa_x += wa_k * (wa_t - wa_x)
        wy_x += tq_k * (wy_t - wy_x)

        # 加速/煞車力條（原始碼只平滑 aEgo，長度換算交給網頁）/ smoothed aEgo only
        rf_x += rf_k * (float(cs.aEgo) - rf_x)

        out = {"ready": bool(sm.updated['modelV2'] or sm.recv_frame['modelV2'] > 0),
               "w": w, "h": h, "t": time.time(),
               "engaged": engaged, "rainbow": rainbow, "status": status,
               "speed": int(round(speed)), "unit": "km/h" if is_metric else "mph",
               "conf": round(float(conf_x), 3), "ballTop": ball_top, "ballBot": ball_bot,
               "blinkL": bool(cs.leftBlinker), "blinkR": bool(cs.rightBlinker),
               "bsL": round(bs_l, 3), "bsR": round(bs_r, 3),
               "showTS": show_ts, "showBS": show_bs, "showRF": show_rf,
               "accel": round(rf_x, 3),
               "border": BORDER_COLORS.get(status, BORDER_COLORS["disengaged"]),
               "torque": round(tq_x, 3), "torqueA": round(tqa_x, 3),
               "steerAngle": round(float(cs.steeringAngleDeg), 1),
               "wheelA": round(wa_x, 3), "wheelY": round(wy_x, 2),
               "wheelCritical": wheel_critical,
               "alert": alert,
               "calibrated": len(getattr(calib, 'rpyCalib', [])) == 3,
               "lines": [], "path": [], "leads": []}
        try:
            def xyz(o):
                a = np.column_stack((np.array(o.x, np.float32),
                                     np.array(o.y, np.float32) + camera_offset,
                                     np.array(o.z, np.float32)))
                return a[(a[:, 0] >= MIN_X) & (a[:, 0] < MAX_DIST)]

            # 車機 _get_ll_color：非最近兩條=白，最近兩條=狀態色，全程未接手=黑
            def ll_color(prob, adjacent):
                alpha = float(np.clip(prob, 0.0, 0.7))
                if not engaged:
                    return _rgba(BLACK, alpha)
                return _rgba(GREEN if adjacent else WHITE, alpha)

            probs = list(m.laneLineProbs)
            stds = list(m.roadEdgeStds)

            # 車機的寬度：0.12，但 i=1,2 起改成 0.16 之後就沒有重設（i=3 與路緣沿用 0.16）
            lwf = 0.12
            for i, ll in enumerate(m.laneLines):
                if i in (1, 2):
                    lwf = 0.16
                pr = float(probs[i]) if i < len(probs) else 0.0
                out["lines"].append({"poly": _poly(T, xyz(ll), lwf * pr, 0.0, ds),
                                     "c": ll_color(pr, i in (1, 2))})
            for i, re_ in enumerate(m.roadEdges):
                pr = 1.0 - (float(stds[i]) if i < len(stds) else 1.0)
                adj = (float(probs[i + 1]) if i + 1 < len(probs) else 1.0) < 0.25
                out["lines"].append({"poly": _poly(T, xyz(re_), lwf, 0.0, ds),
                                     "c": ll_color(pr, adj)})

            pos = xyz(m.position)
            out["path"] = _poly(T, pos, PATH_HALF_W, PATH_OFFSET_Z, ds)

            for ld in (sm['radarState'].leadsV3[:2] if hasattr(sm['radarState'], 'leadsV3') else []):
                if not ld.prob > 0.5:
                    continue
                idx = int(np.clip(ld.dRel / max(MAX_DIST, 1) * len(pos), 0, max(len(pos) - 1, 0)))
                z = float(pos[idx, 2]) if len(pos) else 0.0
                pt = _project(T, np.array([[ld.dRel, -ld.yRel + camera_offset,
                                            z + PATH_OFFSET_Z]], np.float32), ds)
                if len(pt):
                    out["leads"].append({"x": round(float(pt[0][0]), 1),
                                         "y": round(float(pt[0][1]), 1),
                                         "d": round(float(ld.dRel), 1)})
        except Exception as e:
            out["err"] = str(e)[:200]

        with _slock:
            _state["json"] = json.dumps(out).encode()
            _state["ts"] = time.time()
        time.sleep(max(0.0, period - (time.time() - t0)))


def latest():
    with _slock:
        return _state["json"]
