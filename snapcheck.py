import json, urllib.request, io, re
from PIL import Image, ImageDraw
r = urllib.request.urlopen('http://127.0.0.1:8099/mjpeg', timeout=10)
buf = b''
while True:
    buf += r.read(4096)
    i = buf.find(b'\xff\xd8'); j = buf.find(b'\xff\xd9', i + 2)
    if i >= 0 and j > 0:
        jpg = buf[i:j + 2]; break
r.close()
M = json.load(urllib.request.urlopen('http://127.0.0.1:8099/model', timeout=5))
im = Image.open(io.BytesIO(jpg)).convert('RGB')
ov = Image.new('RGBA', im.size, (0,0,0,0))
d = ImageDraw.Draw(ov, 'RGBA')
for L in M.get('lines', []):
    p = [tuple(q) for q in L['poly']]
    m = re.findall(r'[\d.]+', L['c'])
    if len(p) > 2:
        d.polygon(p, fill=(int(m[0]),int(m[1]),int(m[2]),int(float(m[3])*255)))
p = [tuple(q) for q in M.get('path', [])]
if len(p) > 2:
    d.polygon(p, fill=(0,200,255,110))
im = Image.alpha_composite(im.convert('RGBA'), ov).convert('RGB')
im.save('/data/overlay_check.jpg', quality=80)
print('ok', im.size, 'lines', len(M.get('lines',[])))
