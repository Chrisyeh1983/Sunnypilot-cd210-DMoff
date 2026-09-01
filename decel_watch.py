#!/usr/bin/env python3
# Watch who wins the longitudinal plan (who slows the car down).
# 監聽誰搶贏縱向控制（誰在踩煞車）。Log: /data/media/0/decel_watch.log
import time, datetime, traceback
from openpilot.cereal import messaging

LOG = "/data/media/0/decel_watch.log"

def w(line):
    ts = datetime.datetime.now().strftime("%m-%d %H:%M:%S")
    with open(LOG, "a") as f:
        f.write(f"{ts} {line}\n")
        f.flush()

def main():
    sm = messaging.SubMaster(['longitudinalPlanSP', 'carState', 'selfdriveState'])
    w("=== watcher started ===")
    last_key = None
    last_periodic = 0.0
    while True:
        sm.update(1000)
        if not sm.updated['longitudinalPlanSP']:
            continue
        p = sm['longitudinalPlanSP']
        cs = sm['carState']
        v = cs.vEgo * 3.6
        if v < 5:                      # parked / crawling, skip 停車時不記
            continue
        a = float(p.aTarget)
        src = str(p.longitudinalPlanSource)
        dec = str(p.dec.state)
        key = (src, dec, a < -0.25)    # log on state change or braking onset
        now = time.time()
        if key != last_key or (now - last_periodic > 10 and a < -0.25):
            extra = ""
            try:
                scc = p.smartCruiseControl
                extra = f" scc={scc.to_dict()}"
            except Exception:
                pass
            try:
                extra += f" sl={float(p.speedLimit)*3.6:.0f}"
            except Exception:
                pass
            w(f"{v:5.1f}km/h a={a:+5.2f} vT={float(p.vTarget)*3.6:5.1f} src={src:<18} DEC={dec}{extra}")
            last_key = key
            last_periodic = now

while True:
    try:
        main()
    except Exception:
        w("ERROR " + traceback.format_exc().replace("\n", " | "))
        time.sleep(5)
