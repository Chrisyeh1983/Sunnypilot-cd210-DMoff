#!/usr/bin/env python3
"""
opstream - serve comma road camera as MJPEG over HTTP.
把 comma 的前鏡頭影像用 MJPEG 網頁串流出去，安卓盒子用瀏覽器連 IP 就能看。
Usage: opstream.py [--bench]
Env: OPS_PORT OPS_Q OPS_DS OPS_FPS OPS_CAM(narrow|wide)
"""
import io, os, sys, time, threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import numpy as np
from PIL import Image
from msgq.visionipc import VisionIpcClient
from openpilot.cereal.visionipc import VisionStreamType

try:
    import opmodel
except Exception:
    opmodel = None

PORT = int(os.getenv("OPS_PORT", 8099))
JPEG_QUALITY = int(os.getenv("OPS_Q", 40))
DOWNSAMPLE = int(os.getenv("OPS_DS", 2))     # 1344x760 -> 672x380
CAM = os.getenv("OPS_CAM", "narrow").lower()   # narrow=車機螢幕那顆 / the one the comma screen shows
MAX_FPS = float(os.getenv("OPS_FPS", 30))    # 要 > 相機的 20fps，否則會卡拍丟格 / must exceed camera rate

ICON_DIR = "/data/openpilot/openpilot/selfdrive/assets/icons_mici/onroad"
ICONS = {"ts.png": ICON_DIR + "/turn_signal_left.png",     # 方向燈箭頭 / blinker arrow
         "bs.png": ICON_DIR + "/blind_spot_left.png"}      # 盲點 / blind spot

_latest = {"jpeg": None, "ts": 0.0, "fps": 0.0}
_lock = threading.Condition()
_viewers = 0          # 沒人看就完全不做事 / zero work when nobody is watching
_size = [0, 0, DOWNSAMPLE]   # 串流影像尺寸，給圖層投影用 / stream size for the overlay


def nv12_to_ycbcr(buf, w, h, stride, uv_offset, ds=2):
    """NV12 -> YCbCr uint8 array, downsampled by `ds`.
    JPEG 內部就是 YCbCr，直接餵可以完全跳過色彩轉換運算。"""
    a = np.frombuffer(buf, dtype=np.uint8)
    y = a[:uv_offset].reshape(-1, stride)[:h, :w][::ds, ::ds]
    uv = a[uv_offset:uv_offset + (h // 2) * stride].reshape(-1, stride)[:h // 2, :w]
    hh, ww = y.shape
    return np.dstack((y, uv[:, 0::2][:hh, :ww], uv[:, 1::2][:hh, :ww]))


def grabber(bench=False):
    st = (VisionStreamType.VISION_STREAM_WIDE_ROAD if CAM == "wide"
          else VisionStreamType.VISION_STREAM_NARROW_ROAD)
    client = None
    n, t0, tconv, tjpg = 0, time.time(), 0.0, 0.0
    min_dt = 1.0 / MAX_FPS
    last_sent = 0.0
    while True:
        if client is None or not client.is_connected():
            client = VisionIpcClient("camerad", st, True)
            if not client.connect(False):
                with _lock:                       # 相機不在，清掉舊畫面 / drop stale frame
                    _latest["jpeg"] = None
                    _lock.notify_all()
                time.sleep(0.5)
                continue
            _size[0] = client.width // DOWNSAMPLE
            _size[1] = client.height // DOWNSAMPLE
            print(f"connected {client.width}x{client.height} stride={client.stride} "
                  f"uv={client.uv_offset} -> stream {_size[0]}x{_size[1]}", flush=True)
        if _viewers == 0 and not bench:      # 沒人看 -> 不編碼，幾乎不吃 CPU
            time.sleep(0.2)
            continue
        buf = client.recv(timeout_ms=1000)
        if buf is None:
            continue
        now = time.time()
        if now - last_sent < min_dt:              # 節流，不睡覺（睡覺會累積延遲）
            continue
        last_sent = now
        s = time.time()
        img = nv12_to_ycbcr(buf.data, client.width, client.height,
                            client.stride, client.uv_offset, DOWNSAMPLE)
        m = time.time()
        bio = io.BytesIO()
        Image.fromarray(img, "YCbCr").save(bio, "JPEG", quality=JPEG_QUALITY)
        e = time.time()
        tconv += m - s; tjpg += e - m; n += 1
        with _lock:
            _latest["jpeg"] = bio.getvalue()
            _latest["ts"] = e
            _lock.notify_all()
        if bench and n >= 60:
            dt = time.time() - t0
            print(f"BENCH {n} frames in {dt:.2f}s -> {n/dt:.1f} fps | "
                  f"convert {tconv/n*1000:.1f}ms jpeg {tjpg/n*1000:.1f}ms | "
                  f"size {len(_latest['jpeg'])//1024}KB {img.shape}", flush=True)
            return
        if n % 200 == 0:
            dt = time.time() - t0
            print(f"[{n}] {n/dt:.1f} fps avg, frame {len(_latest['jpeg'])//1024}KB", flush=True)


PAGE = r"""<!doctype html><meta name=viewport content="width=device-width,initial-scale=1">
<title>comma live</title>
<style>
html,body{margin:0;background:#000;height:100%;overflow:hidden}
#wrap{position:fixed;inset:0}
#v,#c{position:absolute;inset:0;width:100%;height:100%}
#v{object-fit:contain;display:block}
#s{position:absolute;left:8px;top:6px;font:12px/1.4 system-ui,sans-serif;
   color:#9fe;text-shadow:0 1px 2px #000;pointer-events:none}
</style>
<div id=wrap><img id=v src="/mjpeg"><canvas id=c></canvas><div id=s></div></div>
<script>
const img=document.getElementById('v'),cv=document.getElementById('c'),
      st=document.getElementById('s'),g=cv.getContext('2d');
let M=null;

function fit(){
  const d=window.devicePixelRatio||1;
  const w=Math.round(cv.clientWidth*d), h=Math.round(cv.clientHeight*d);
  if(w&&h&&(cv.width!==w||cv.height!==h)){cv.width=w;cv.height=h;}
}
addEventListener('resize',fit); fit();

// 影像用 object-fit:contain，算出它實際畫在哪 / where the image really sits
function box(){
  const d=window.devicePixelRatio||1, cw=cv.clientWidth*d, ch=cv.clientHeight*d;
  const iw=img.naturalWidth||(M&&M.w)||672, ih=img.naturalHeight||(M&&M.h)||380;
  const k=Math.min(cw/iw,ch/ih);
  return {k, ox:(cw-iw*k)/2, oy:(ch-ih*k)/2, k2:k*((M&&M.w)?(iw/M.w):1)};
}

// 車機是畫「真實寬度的多邊形」，近粗遠細 / real-world-width polygons, like the car
function poly(pts,b){
  if(!pts||pts.length<3) return false;
  g.beginPath();
  for(let i=0;i<pts.length;i++){
    const x=b.ox+pts[i][0]*b.k2, y=b.oy+pts[i][1]*b.k2;
    i?g.lineTo(x,y):g.moveTo(x,y);
  }
  g.closePath(); return true;
}

// 車機圖示 / icons served from the car's own asset folder
const IM={ts:new Image(), bs:new Image()};
IM.ts.src='/icon/ts.png'; IM.bs.src='/icon/bs.png';
let lastSide='left', aA=0, prevAlert=null;

function drawIcon(im,x,y,w,h,alpha,flip){
  if(!im.complete||!im.naturalWidth||alpha<=0.01) return;
  g.save(); g.globalAlpha=Math.min(1,alpha);
  if(flip){ g.translate(x+w,y); g.scale(-1,1); g.drawImage(im,0,0,w,h); }
  else g.drawImage(im,x,y,w,h);
  g.restore();
}

// 方向燈閃爍：週期 0.75s（車機 TURN_SIGNAL_BLINK_PERIOD），亮起後往 20% 衰減
function blinkA(){
  const ph=(performance.now()/1000)%0.75;
  return Math.min(255, 51+(510-51)*Math.exp(-ph/0.3))/255;
}

function wrap(text,maxw){
  const words=text.split(/\s+/), out=[]; let line='';
  for(const w of words){
    const t=line?line+' '+w:w;
    if(g.measureText(t).width>maxw && line){ out.push(line); line=w; } else line=t;
  }
  if(line) out.push(line);
  return out;
}

// 方向燈 + 盲點。盲點照抄 blind_spot_indicators.py（車機畫面基準高 240）
function signals(b){
  const IW=M.w*b.k2, IH=M.h*b.k2, S=IH/240;
  if(M.showBS!==false){
    const bw=108*S, bh=128*S, mx=20*S, by=b.oy+100*S;
    drawIcon(IM.bs, b.ox+mx,            by, bw, bh, M.bsL||0, false);
    drawIcon(IM.bs, b.ox+IW-mx-bw,      by, bw, bh, M.bsR||0, true);
  }
  // 方向燈常駐箭頭。車機 mici 螢幕沒有這個（只在變換車道警示裡出現），
  // 這是嚕寶要求另加的，所以放兩側邊緣避免壓到時速 / extra, not on the car's mici screen
  if(M.showTS!==false && (M.blinkL||M.blinkR) && !M.alert && aA<=0.01){
    const h=0.14*IH, w=h*120/109, mx=0.02*IW, y=b.oy+0.14*IH, a=blinkA();
    if(M.blinkL) drawIcon(IM.ts, b.ox+mx,        y, w, h, a, false);
    if(M.blinkR) drawIcon(IM.ts, b.ox+IW-mx-w,   y, w, h, a, true);
  }
}

// 系統警示，照抄 mici/onroad/alert_renderer.py（基準 476x240 的 content rect）
function alertHud(b){
  const A=M.alert||null;
  aA += 0.25*((A?1:0)-aA);
  if(A) prevAlert=A;
  const al=A||prevAlert;
  if(!al||aA<0.01){ if(!A&&aA<0.01) prevAlert=null; return; }
  const IW=M.w*b.k2, IH=M.h*b.k2, S=IH/240;

  // 底圖：上面 20% 實色，其餘漸層淡出 / solid top fifth, then fade out
  const bgh=(al.bgh||1)*IH, solid=bgh*0.2;
  g.save(); g.beginPath(); g.rect(b.ox,b.oy,IW,IH); g.clip();
  g.fillStyle='rgba('+al.bg+','+(0.9*aA).toFixed(3)+')';
  g.fillRect(b.ox,b.oy,IW,solid);
  const gr=g.createLinearGradient(0,b.oy+solid,0,b.oy+bgh);
  gr.addColorStop(0,'rgba('+al.bg+','+(0.9*aA).toFixed(3)+')');
  gr.addColorStop(1,'rgba('+al.bg+',0)');
  g.fillStyle=gr; g.fillRect(b.ox,b.oy+solid,IW,bgh-solid);

  // 圖示 / icon
  let side=al.side||lastSide; if(al.side) lastSide=al.side;
  let iw=0;
  if(al.icon){
    const isTs=al.icon==='ts';
    const w=(isTs?104:134)*S, h=(isTs?96:150)*S;
    const mx=(isTs?2:8)*S, my=(isTs?5:0)*S;
    const x=(side==='left')?b.ox+mx:b.ox+IW-mx-w;
    drawIcon(isTs?IM.ts:IM.bs, x, b.oy+my, w, h, (isTs?blinkA():1)*aA, side==='right');
    iw=w+mx;
  }

  // 文字：車機是全部小寫 / the car lowercases everything
  const left=(al.icon&&side==='left');
  const tx=left? b.ox+iw+8*S : b.ox+18*S;
  const tw=IW-iw-26*S;
  let fs=(al.t1.length<=12?82:(al.t1.length<=16?70:54));
  if(al.icon) fs-=10;
  g.textBaseline='top'; g.textAlign=left?'right':'left';
  const ax=left? tx+tw : tx;
  g.fillStyle='rgba(255,255,255,'+(0.9*aA).toFixed(3)+')';
  g.font='700 '+(fs*S).toFixed(1)+'px system-ui,-apple-system,sans-serif';
  let y=b.oy+8*S;
  for(const line of wrap(al.t1,tw)){ g.fillText(line,ax,y); y+=fs*S*0.92; }
  if(al.t2){
    const fs2=(al.t2.length>24?32:(al.t2.length>18?36:40));
    g.font='400 '+(fs2*S).toFixed(1)+'px system-ui,-apple-system,sans-serif';
    g.fillStyle='rgba(255,255,255,'+(0.65*aA).toFixed(3)+')';
    y+=2*S;
    for(const line of wrap(al.t2,tw)){ g.fillText(line,ax,y); y+=fs2*S*0.92; }
  }
  g.restore();
}

// 車速 + 信心球。比例照抄車機 2160x1080 螢幕上的座標 / same proportions as the car UI
function hud(b){
  const IW=M.w*b.k2, IH=M.h*b.k2, cx=b.ox+IW/2;
  g.save();
  g.beginPath(); g.rect(b.ox,b.oy,IW,IH); g.clip();
  // 有警示時把上方元素藏起來，跟車機 set_can_draw_top_icons 一樣 / hide top stuff during alerts
  if(!M.alert && aA<=0.01){
    // 車速：字高 176/1080、中心 y=180/1080；單位：66/1080、y=290/1080
    g.textAlign='center'; g.textBaseline='middle';
    g.shadowColor='rgba(0,0,0,.55)'; g.shadowBlur=IH*0.018;
    g.fillStyle='#fff';
    g.font='700 '+(IH*0.163).toFixed(1)+'px system-ui,-apple-system,sans-serif';
    g.fillText(String(M.speed==null?0:M.speed), cx, b.oy+IH*0.167);
    g.fillStyle='rgba(255,255,255,.78)';
    g.font='500 '+(IH*0.061).toFixed(1)+'px system-ui,-apple-system,sans-serif';
    g.fillText(M.unit||'km/h', cx, b.oy+IH*0.268);
  }
  // 信心球：半徑 24/1080，貼右緣，越高＝AI 越有把握；未接手時沉出畫面
  g.shadowBlur=0;
  const r=IH*0.0222, conf=(M.conf==null?-0.5:M.conf);
  const y=b.oy+(1-conf)*(IH-2*r)+r, x=b.ox+IW-r;
  const gr=g.createLinearGradient(0,y-r,0,y+r);
  gr.addColorStop(0,M.ballTop||'rgb(50,50,50)');
  gr.addColorStop(1,M.ballBot||'rgb(13,13,13)');
  g.beginPath(); g.arc(x,y,r,0,7); g.fillStyle=gr; g.fill();
  g.restore();
}

function draw(){
  fit();                       // 每張都確認畫布尺寸 / re-check every frame
  g.clearRect(0,0,cv.width,cv.height);
  if(M&&M.ready){
    const b=box();
    // 車道線 + 路緣（顏色寬度都由車機規則算好了）/ colors+widths already decided server-side
    (M.lines||[]).forEach(L=>{ if(poly(L.poly,b)){ g.fillStyle=L.c; g.fill(); } });
    // 行駛路徑 / driving path
    if(poly(M.path,b)){
      let y0=1e9,y1=-1e9;
      M.path.forEach(p=>{const y=b.oy+p[1]*b.k2; if(y<y0)y0=y; if(y>y1)y1=y;});
      const gr=g.createLinearGradient(0,y1,0,y0);     // y1=底(近) y0=頂(遠)
      if(M.rainbow){
        // 照抄車機 rainbow_path.py：8 段、50 度/秒、s=.9 l=.6、alpha .8 往上退 .3
        const N=8, hue0=(performance.now()/1000*50)%360;
        for(let i=0;i<N;i++){
          const pos=i/(N-1);
          gr.addColorStop(pos,'hsla('+((hue0+pos*360)%360).toFixed(1)+',90%,60%,'
                              +(0.8*(1-pos*0.3)).toFixed(3)+')');
        }
      } else {
        gr.addColorStop(0,'rgba(13,248,122,.40)');
        gr.addColorStop(.5,'rgba(114,255,92,.35)');
        gr.addColorStop(1,'rgba(114,255,92,0)');
      }
      g.fillStyle=gr; g.fill();
    }
    // 前車 / lead car
    (M.leads||[]).forEach(L=>{
      const x=b.ox+L.x*b.k2,y=b.oy+L.y*b.k2,r=Math.max(8,90/Math.max(L.d,5))*b.k;
      g.beginPath();g.arc(x,y,r,0,7);g.fillStyle='rgba(255,200,0,.55)';g.fill();
      g.lineWidth=2*b.k;g.strokeStyle='#ffdd44';g.stroke();
    });
    hud(b);
    alertHud(b);
    signals(b);
    st.textContent=(M.engaged?'ENGAGED':'ready')+(M.calibrated?'':' (未校正)')
                   +'  '+cv.width+'x'+cv.height+'  lines:'+(M.lines?M.lines.length:0)
                   +(M.leads&&M.leads.length?'  lead '+M.leads[0].d+'m':'');
  } else { st.textContent='等待模型資料… '+cv.width+'x'+cv.height; }
  requestAnimationFrame(draw);
}
draw();

async function poll(){
  try{ M=await (await fetch('/model',{cache:'no-store'})).json(); }catch(e){}
  setTimeout(poll,80);
}
poll();
</script>
""".encode()


class Handler(BaseHTTPRequestHandler):
    protocol_version = "HTTP/1.1"
    disable_nagle_algorithm = True      # 關掉 Nagle，每張圖立刻送出

    def log_message(self, *a):
        pass

    def do_GET(self):
        if self.path in ("/", "/index.html"):
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Cache-Control", "no-store, must-revalidate")
            self.send_header("Content-Length", str(len(PAGE)))
            self.end_headers()
            self.wfile.write(PAGE)
            return
        if self.path.startswith("/icon/"):
            name = ICONS.get(self.path[6:])
            if not name or not os.path.exists(name):
                self.send_error(404)
                return
            with open(name, "rb") as f:
                body = f.read()
            self.send_response(200)
            self.send_header("Content-Type", "image/png")
            self.send_header("Cache-Control", "max-age=86400")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)
            return
        if self.path == "/model":
            body = opmodel.latest() if opmodel else b'{"ready":false}'
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.send_header("Cache-Control", "no-store")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)
            return
        if self.path != "/mjpeg":
            self.send_error(404)
            return
        self.send_response(200)
        self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=f")
        self.send_header("Cache-Control", "no-store")
        self.send_header("Connection", "close")
        self.end_headers()
        self.close_connection = True
        global _viewers
        with _lock:
            _viewers += 1
        last = 0.0
        try:
            while True:
                with _lock:
                    while _latest["ts"] <= last or _latest["jpeg"] is None:
                        _lock.wait(5)
                    frame, last = _latest["jpeg"], _latest["ts"]
                self.wfile.write(b"--f\r\nContent-Type: image/jpeg\r\nContent-Length: "
                                 + str(len(frame)).encode() + b"\r\n\r\n" + frame + b"\r\n")
        except (BrokenPipeError, ConnectionResetError, OSError):
            pass
        finally:
            with _lock:
                _viewers -= 1


class Server(ThreadingHTTPServer):
    daemon_threads = True
    allow_reuse_address = True


if __name__ == "__main__":
    if "--bench" in sys.argv:
        grabber(bench=True)
        sys.exit(0)
    os.nice(10)      # 讓 openpilot 永遠優先搶到 CPU / openpilot always outranks us
    threading.Thread(target=grabber, daemon=True).start()
    if opmodel:
        threading.Thread(target=opmodel.worker, args=(lambda: tuple(_size),), daemon=True).start()
    print(f"serving on :{PORT} q={JPEG_QUALITY} ds={DOWNSAMPLE} fps={MAX_FPS}", flush=True)
    Server(("0.0.0.0", PORT), Handler).serve_forever()
