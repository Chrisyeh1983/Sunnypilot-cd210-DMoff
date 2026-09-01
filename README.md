# comma4-stream

把 **comma 4**（跑 sunnypilot）的前鏡頭影像 + openpilot 圖層，用一個網頁串到車上的
安卓盒子（Linkifun GT6）看，像特斯拉那樣。盒子只要開瀏覽器連 IP，不用裝 App。

Stream the comma 4 road camera plus openpilot's own overlays to any browser on the
car's head unit. No app to install — just open `http://<comma-ip>:8099/`.

## 檔案 / Files

| 檔案 | 放哪 | 做什麼 |
|---|---|---|
| `opstream.py` | `/data/opstream.py` | MJPEG 影像伺服器 + 網頁（HTML/JS 都內嵌在裡面）|
| `opmodel.py` | `/data/opmodel.py` | 把 modelV2 的車道線/路徑投影成像素，算好顏色與寬度給網頁 |
| `continue.sh` | `/data/continue.sh` | 開機自動啟動（在 `exec ./launch_openpilot.sh` 前插入背景段）|
| `decel_watch.py` | `/data/decel_watch.py` | 高速莫名減速的兇手記錄器 |
| `snapcheck.py` | `/data/snapcheck.py` | 抓一張圖 + `/model`，用 PIL 合成 `overlay_check.jpg` 驗證投影 |

## 跑起來 / Run

```sh
cd /data && setsid nohup env PYTHONPATH=/data/openpilot OPS_CAM=narrow \
  /usr/local/venv/bin/python3 /data/opstream.py >/data/opstream.log 2>&1 &
```

環境變數：`OPS_PORT`(8099) `OPS_Q`(40) `OPS_DS`(2) `OPS_FPS`(30) `OPS_CAM`(narrow|wide)

端點：`/` 網頁、`/mjpeg` 純影像、`/model` 圖層 JSON、`/icon/ts.png` `/icon/bs.png` 圖示

## 畫了什麼 / What it draws

車道線、路緣、行駛路徑（含彩虹模式）、前車、車速、AI 信心球、方向燈、盲點、系統警示、
會轉的方向盤圖示、底部橫向扭力弧線、狀態邊框、加速/煞車力條。
**所有顏色/寬度/位置都照抄車機自己的 renderer**，不是自己發明的：
`selfdrive/ui/mici/onroad/{model_renderer,confidence_ball,alert_renderer,hud_renderer}.py`
與 `selfdrive/ui/mici/onroad/torque_bar.py`、`selfdrive/ui/onroad/augmented_road_view.py`、
`selfdrive/ui/sunnypilot/onroad/{rainbow_path,blind_spot_indicators,turn_signal,rocket_fuel}.py`。

## 幾個關鍵細節 / Gotchas

- comma 4（mici）螢幕是 **536x240**，不是 2160x1080（`big_ui()` 只有 tici/tizi 為真）。
  alert / 盲點的比例要用 240 當基準。
- 車道線是「**真實寬度的多邊形**」（近粗遠細），不是固定像素寬的線。
- NV12 → 直接堆成 YCbCr 餵 PIL 存 JPEG，跳過色彩轉換：55ms → 2.6ms。
- 影像來源用 VisionIPC，**不要**開 `IsLiveStreaming`（會拉起車機沒裝的 aiortc）。
- 窄角相機不套 `wideFromDeviceEuler`；路徑要 `z + 1.22`，車道線不用。

離線也能驗圖層：在頁面 console 攔截 `fetch('/model')` 回傳假資料即可。

MIT
