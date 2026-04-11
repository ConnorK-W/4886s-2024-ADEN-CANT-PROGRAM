#!/usr/bin/env python3
# Launch: ./tools/.venv/bin/python3 tools/vision_tuner.py
"""
Hancock Technologies(TM) Vision Tuning Tool

Data sources (auto-detected):
  - Brain user port : reads VIS: printf lines for detections
  - Sensor USB-C    : polls AI_OBJECTS for detections (fallback)
  - Mac webcam      : live feed for click-to-pick color

Requirements: pip3 install pyserial opencv-python Pillow
"""

import sys, os, struct, time, threading, colorsys, re, subprocess, signal
from collections import deque

try:
    import serial
    import serial.tools.list_ports
except ImportError:
    print("pip3 install pyserial opencv-python Pillow")
    sys.exit(1)

try:
    import cv2
    HAS_CV2 = True
except ImportError:
    HAS_CV2 = False

try:
    from PIL import Image as PILImage, ImageTk
    HAS_PIL = True
except ImportError:
    HAS_PIL = False

import tkinter as tk

# ── Constants ────────────────────────────────────────────────────
SENSOR_W, SENSOR_H = 320, 240
SCALE = 2
CANVAS_W, CANVAS_H = SENSOR_W * SCALE, SENSOR_H * SCALE
TARGET_X = 160

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
LOGO_PATH  = os.path.join(SCRIPT_DIR, '..', 'Hancock-Sarah.jpg')

# Theme — clean dark
BG       = '#0e0e12'
BG2      = '#161620'
BG3      = '#1e1e2e'
ACCENT   = '#c49a6c'   # warm gold to match the brand
GREEN    = '#a6da95'
RED      = '#ed8796'
YELLOW   = '#eed49f'
TEXT     = '#cad3f5'
DIM      = '#5b6078'
GRID_CLR = '#181825'
CANVAS_BG= '#0b0b10'

# Regex for VIS printf
RE_VIS = re.compile(r'^VIS: X:(-?\d+) W:(\d+) H:(\d+) Cnt:(\d+)')

# ── CDC2 Protocol ────────────────────────────────────────────────
CMD_HDR   = bytes([0xC9, 0x36, 0xB8, 0x47])
REPLY_HDR = bytes([0xAA, 0x55])
CDC_USER  = 0x56
ACK_OK    = 0x76

def _crc16(data):
    crc = 0
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) if crc & 0x8000 else (crc << 1)
        crc &= 0xFFFF
    return crc

def _varu16(n):
    return bytes([n]) if n <= 0x7F else bytes([(n >> 8) | 0x80, n & 0xFF])

def cdc2_build(ecmd, payload=b''):
    body = CMD_HDR + bytes([CDC_USER, ecmd]) + _varu16(len(payload)) + payload
    return body + struct.pack('>H', _crc16(body))

def cdc2_parse(buf):
    idx = buf.find(REPLY_HDR)
    if idx < 0: return None, 0
    buf = buf[idx:]
    if len(buf) < 5: return None, 0
    cmd = buf[2]
    if buf[3] & 0x80:
        if len(buf) < 6: return None, 0
        sz = ((buf[3] & 0x7F) << 8) | buf[4]; hdr_end = 5
    else:
        sz = buf[3]; hdr_end = 4
    total = hdr_end + sz
    if len(buf) < total: return None, 0
    return {'cmd': cmd, 'ecmd': buf[hdr_end], 'ack': buf[hdr_end+1],
            'payload': bytes(buf[hdr_end+2:total-2])}, idx + total


# ── Detection Frame ──────────────────────────────────────────────
class Frame:
    __slots__ = ('exists','count','cx','cy','w','h','ts')
    def __init__(self, exists=False, count=0, cx=0, cy=SENSOR_H//2, w=0, h=0):
        self.exists, self.count = exists, count
        self.cx, self.cy, self.w, self.h = cx, cy, w, h
        self.ts = time.time()


# ── Data Sources ─────────────────────────────────────────────────

class BrainUserPort:
    def __init__(self):
        self.ser = None; self.port = None; self.running = False
        self._buf = ''; self._cbs = []; self._thread = None

    def connect(self, port):
        self.disconnect()
        try:
            self.ser = serial.Serial(port, 115200, timeout=0.05)
            self.port = port; self.running = True
            self._thread = threading.Thread(target=self._loop, daemon=True)
            self._thread.start(); return True
        except (serial.SerialException, OSError): return False

    def disconnect(self):
        self.running = False
        if self._thread: self._thread.join(timeout=1); self._thread = None
        if self.ser:
            try: self.ser.close()
            except: pass
        self.ser = None

    @property
    def connected(self): return self.ser is not None and self.ser.is_open and self.running
    def on_line(self, cb): self._cbs.append(cb)

    def _loop(self):
        while self.running:
            try:
                if not (self.ser and self.ser.is_open): break
                n = self.ser.in_waiting
                if n:
                    self._buf += self.ser.read(n).decode('ascii', errors='replace')
                    while '\n' in self._buf:
                        line, self._buf = self._buf.split('\n', 1)
                        line = line.strip()
                        if line:
                            for cb in self._cbs: cb(line)
                else: time.sleep(0.005)
            except: break
        self.running = False


class SensorDirect:
    def __init__(self):
        self.ser = None; self.port = None; self._lock = threading.Lock()

    def connect(self, port):
        self.disconnect()
        try:
            self.ser = serial.Serial(port, 115200, timeout=0.15)
            self.port = port; self.ser.reset_input_buffer(); return True
        except: return False

    def disconnect(self):
        if self.ser:
            try: self.ser.close()
            except: pass
        self.ser = None

    @property
    def connected(self): return self.ser is not None and self.ser.is_open

    def get_objects(self):
        if not self.ser: return []
        with self._lock:
            try:
                self.ser.reset_input_buffer()
                self.ser.write(cdc2_build(0x68, bytes([8])))
                buf = b''; deadline = time.time() + 0.15
                while time.time() < deadline:
                    n = self.ser.in_waiting
                    if n: buf += self.ser.read(n)
                    else: time.sleep(0.005)
                if buf:
                    r, _ = cdc2_parse(buf)
                    if r and r['ack'] == ACK_OK and len(r['payload']) > 1:
                        return self._parse(r['payload'])
            except: pass
        return []

    def _parse(self, p):
        count = p[0]
        if count == 0: return []
        objs = []; i = 1
        for _ in range(count):
            if i + 18 > len(p): break
            oid, otyp = p[i], p[i+1]
            if otyp in (1,2,4):
                xo, yo, w, h = struct.unpack_from('<HHHH', p, i+2)
                objs.append({'id':oid,'type':otyp,'cx':xo+w//2,'cy':yo+h//2,'w':w,'h':h})
            i += 18
        return objs


class Webcam:
    def __init__(self):
        self.cap = None; self._frame = None
        self._lock = threading.Lock(); self._running = False

    def start(self):
        if not HAS_CV2: return False
        self.cap = cv2.VideoCapture(0)
        if not self.cap.isOpened(): return False
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, CANVAS_W)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, CANVAS_H)
        self._running = True
        threading.Thread(target=self._loop, daemon=True).start()
        return True

    def stop(self):
        self._running = False
        if self.cap: self.cap.release(); self.cap = None

    def grab(self):
        with self._lock: return self._frame

    def _loop(self):
        while self._running and self.cap and self.cap.isOpened():
            ret, frame = self.cap.read()
            if ret:
                frame = cv2.resize(frame, (CANVAS_W, CANVAS_H))
                frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                with self._lock: self._frame = frame
            time.sleep(0.033)


# ═════════════════════════════════════════════════════════════════
#  GUI
# ═════════════════════════════════════════════════════════════════

class VisionTuner:
    def __init__(self):
        self.root = tk.Tk()
        self.root.title('Hancock Technologies\u2122')
        self.root.configure(bg=BG)
        self.root.resizable(False, False)

        self.brain_user = BrainUserPort()
        self.brain_user.on_line(self._on_serial_line)
        self.sensor = SensorDirect()
        self.webcam = Webcam()

        self.frame = Frame()
        self.history = deque(maxlen=100)
        self._line_q = []; self._lock = threading.Lock()
        self._log = deque(maxlen=6)
        self._last_vis = 0.0
        self._cam_photo = None
        self._samples = []

        self.r   = tk.IntVar(value=255)
        self.g   = tk.IntVar(value=175)
        self.b   = tk.IntVar(value=83)
        self.hue = tk.DoubleVar(value=20.0)
        self.sat = tk.DoubleVar(value=1.0)

        self._build()
        for v in (self.r, self.g, self.b, self.hue, self.sat):
            v.trace_add('write', self._on_sig)

        self._refresh_sig()
        self._build_hue_bar()
        self._draw_hue_overlay()
        self._connect_all()
        self._auto_reconnect()
        self._poll()

    def _build(self):
        # ── Header with logo and brand ──
        header = tk.Frame(self.root, bg=BG, padx=16, pady=12)
        header.pack(fill='x')

        # Logo
        self._logo_photo = None
        if HAS_PIL and os.path.exists(LOGO_PATH):
            try:
                img = PILImage.open(LOGO_PATH)
                img = img.resize((44, 44), PILImage.LANCZOS)
                # Make circular mask
                mask = PILImage.new('L', (44, 44), 0)
                from PIL import ImageDraw
                ImageDraw.Draw(mask).ellipse((0, 0, 44, 44), fill=255)
                img.putalpha(mask)
                self._logo_photo = ImageTk.PhotoImage(img)
                tk.Label(header, image=self._logo_photo, bg=BG).pack(side='left', padx=(0, 12))
            except:
                pass

        brand = tk.Frame(header, bg=BG)
        brand.pack(side='left')
        tk.Label(brand, text='Hancock Technologies\u2122',
                 fg=ACCENT, bg=BG, font=('Helvetica Neue', 16, 'bold')).pack(anchor='w')
        tk.Label(brand, text='Vision Tuning Tool',
                 fg=DIM, bg=BG, font=('Helvetica Neue', 11)).pack(anchor='w')

        # Source indicators (right side of header)
        ind_frame = tk.Frame(header, bg=BG)
        ind_frame.pack(side='right')
        self.ind_brain = tk.Label(ind_frame, text='\u25cf Brain', fg=RED, bg=BG,
                                  font=('Menlo', 9))
        self.ind_brain.pack(anchor='e')
        self.ind_sensor = tk.Label(ind_frame, text='\u25cf Sensor', fg=RED, bg=BG,
                                   font=('Menlo', 9))
        self.ind_sensor.pack(anchor='e')
        self.ind_cam = tk.Label(ind_frame, text='\u25cf Webcam', fg=RED, bg=BG,
                                font=('Menlo', 9))
        self.ind_cam.pack(anchor='e')

        # Divider
        tk.Frame(self.root, bg=BG3, height=1).pack(fill='x', padx=16)

        # ── Main ──
        main = tk.Frame(self.root, bg=BG, padx=16, pady=12)
        main.pack(fill='both')

        # Left: canvas
        left = tk.Frame(main, bg=BG)
        left.pack(side='left')

        cam_header = tk.Frame(left, bg=BG)
        cam_header.pack(fill='x', pady=(0, 6))
        tk.Label(cam_header, text='LIVE VIEW', fg=DIM, bg=BG,
                 font=('Helvetica Neue', 9, 'bold')).pack(side='left')
        tk.Label(cam_header, text='click to sample color', fg=DIM, bg=BG,
                 font=('Helvetica Neue', 9)).pack(side='right')

        self.canvas = tk.Canvas(left, width=CANVAS_W, height=CANVAS_H,
                                bg=CANVAS_BG, highlightthickness=1,
                                highlightbackground=BG3)
        self.canvas.pack()
        self.canvas.bind('<Button-1>', self._on_canvas_click)
        self._draw_grid()

        self.stats = tk.Label(left, text='', fg=DIM, bg=BG,
                              font=('Menlo', 10), anchor='w')
        self.stats.pack(fill='x', pady=(6, 0))

        # Right: controls
        rt = tk.Frame(main, bg=BG, padx=20)
        rt.pack(side='right', fill='y')

        self._section(rt, 'COLOR SIGNATURE')
        self._slider(rt, 'R', self.r, 0, 255)
        self._slider(rt, 'G', self.g, 0, 255)
        self._slider(rt, 'B', self.b, 0, 255)
        tk.Frame(rt, bg=BG, height=8).pack()
        self._section(rt, 'HUE TOLERANCE')
        self._slider(rt, 'Hue \u00b1', self.hue, 1.0, 60.0, res=0.5)

        tk.Frame(rt, bg=BG, height=8).pack()
        self.preview = tk.Canvas(rt, width=220, height=32, bg=BG, highlightthickness=0)
        self.preview.pack()
        tk.Frame(rt, bg=BG, height=4).pack()
        self.hue_cv = tk.Canvas(rt, width=220, height=24, bg=BG, highlightthickness=0)
        self.hue_cv.pack()
        self._hue_img = None

        tk.Frame(rt, bg=BG, height=10).pack()
        self._section(rt, 'STABILITY')
        self.stab_cv = tk.Canvas(rt, width=220, height=12, bg=BG3, highlightthickness=0)
        self.stab_cv.pack(pady=(2, 0))
        self.stab_lbl = tk.Label(rt, text='\u2014', fg=DIM, bg=BG, font=('Menlo', 9))
        self.stab_lbl.pack()

        # Buttons
        tk.Frame(rt, bg=BG, height=14).pack()
        bf = tk.Frame(rt, bg=BG)
        bf.pack(fill='x')
        tk.Button(bf, text='Apply to Code', command=self._apply_to_code,
                  bg=ACCENT, fg='#000', font=('Helvetica Neue', 11, 'bold'),
                  relief='flat', padx=14, pady=4, cursor='hand2').pack(fill='x', pady=(0, 6))
        tk.Button(bf, text='Push Live to Brain', command=self._push_live,
                  bg=BG3, fg=GREEN, font=('Helvetica Neue', 10, 'bold'),
                  relief='flat', padx=14, pady=3, cursor='hand2').pack(fill='x', pady=(0, 6))
        tk.Button(bf, text='Copy Line', command=self._copy,
                  bg=BG3, fg=TEXT, font=('Helvetica Neue', 10),
                  relief='flat', padx=14, pady=3, cursor='hand2').pack(fill='x', pady=(0, 6))

        bf2 = tk.Frame(bf, bg=BG)
        bf2.pack(fill='x')
        tk.Button(bf2, text='Clear Picks', command=self._clear_samples,
                  bg=BG3, fg=RED, font=('Helvetica Neue', 9),
                  relief='flat', padx=8, pady=2, cursor='hand2').pack(side='left')
        self.sample_lbl = tk.Label(bf2, text='  0 samples', fg=DIM, bg=BG,
                                   font=('Menlo', 9))
        self.sample_lbl.pack(side='left', padx=(8, 0))
        tk.Button(bf2, text='Reconnect', command=self._connect_all,
                  bg=BG3, fg=DIM, font=('Helvetica Neue', 9),
                  relief='flat', padx=8, pady=2, cursor='hand2').pack(side='right')

        # ── Output ──
        tk.Frame(self.root, bg=BG3, height=1).pack(fill='x', padx=16, pady=(0, 0))
        of = tk.Frame(self.root, bg=BG2, padx=16, pady=8)
        of.pack(fill='x')
        self.out_lbl = tk.Label(of, text='', fg=ACCENT, bg=BG2,
                                font=('Menlo', 10), anchor='w')
        self.out_lbl.pack(fill='x')

        # ── Log ──
        lf = tk.Frame(self.root, bg=BG, padx=16, pady=8)
        lf.pack(fill='x')
        self.log_lbl = tk.Label(lf, text='', fg=DIM, bg=BG,
                                font=('Menlo', 8), anchor='w', justify='left')
        self.log_lbl.pack(fill='x')

        self.status = tk.Label(lf, text='', fg=DIM, bg=BG, font=('Menlo', 8))

    def _section(self, p, t):
        tk.Label(p, text=t, fg=DIM, bg=BG,
                 font=('Helvetica Neue', 9, 'bold')).pack(anchor='w', pady=(0, 4))

    def _slider(self, p, label, var, lo, hi, res=1):
        row = tk.Frame(p, bg=BG); row.pack(fill='x', pady=1)
        tk.Label(row, text=f'{label:>5}', fg=TEXT, bg=BG,
                 font=('Menlo', 10), width=6, anchor='e').pack(side='left')
        tk.Scale(row, variable=var, from_=lo, to=hi, orient='horizontal', length=140,
                 resolution=res, bg=BG, fg=TEXT, troughcolor=BG3, highlightthickness=0,
                 sliderrelief='flat', font=('Menlo', 8), showvalue=True,
                 activebackground=ACCENT).pack(side='left', padx=(4, 0))

    # ─── Canvas ──────────────────────────────────────────────────

    def _draw_grid(self):
        c = self.canvas
        for x in range(0, CANVAS_W+1, CANVAS_W//8):
            c.create_line(x, 0, x, CANVAS_H, fill=GRID_CLR, dash=(2, 4))
        for y in range(0, CANVAS_H+1, CANVAS_H//5):
            c.create_line(0, y, CANVAS_W, y, fill=GRID_CLR, dash=(2, 4))
        tx = TARGET_X * SCALE
        c.create_line(tx, 0, tx, CANVAS_H, fill=RED, dash=(6, 3), width=1, tags='grid')
        c.create_text(tx+5, 10, text='TARGET', fill=RED, anchor='nw', font=('Menlo', 8), tags='grid')

    def _draw_cam_frame(self):
        if not HAS_CV2: return
        img = self.webcam.grab()
        if img is None: return
        try:
            rgb = cv2.resize(img, (CANVAS_W, CANVAS_H), interpolation=cv2.INTER_NEAREST)
            h, w = rgb.shape[:2]
            ppm = f'P6\n{w} {h}\n255\n'.encode() + rgb.tobytes()
            photo = tk.PhotoImage(data=ppm)
            self._cam_photo = photo
            self.canvas.delete('cam')
            self.canvas.create_image(0, 0, anchor='nw', image=photo, tags='cam')
            self.canvas.tag_lower('cam')
        except: pass

    def _draw_detections(self):
        c = self.canvas; c.delete('det')
        f = self.frame
        if not f.exists:
            if self.webcam.grab() is None:
                c.create_text(CANVAS_W//2, CANVAS_H//2, text='NO DETECTION',
                              fill=DIM, font=('Helvetica Neue', 14, 'bold'), tags='det')
            return
        x1 = (f.cx-f.w//2)*SCALE; y1 = (f.cy-f.h//2)*SCALE
        x2 = (f.cx+f.w//2)*SCALE; y2 = (f.cy+f.h//2)*SCALE
        c.create_rectangle(x1, y1, x2, y2, outline=GREEN, width=2, tags='det')
        ox, oy = f.cx*SCALE, f.cy*SCALE
        c.create_line(ox-10, oy, ox+10, oy, fill=GREEN, tags='det')
        c.create_line(ox, oy-10, ox, oy+10, fill=GREEN, tags='det')
        c.create_text(x1, y1-5, text=f'{f.w}\u00d7{f.h}  ({f.cx},{f.cy})',
                      fill=GREEN, anchor='sw', font=('Menlo', 9), tags='det')
        tx = TARGET_X * SCALE
        if abs(ox-tx) > 6:
            c.create_line(tx, oy, ox, oy, fill=YELLOW, dash=(3, 2), tags='det')
            c.create_text((tx+ox)//2, oy-10, text=f'err={f.cx-TARGET_X:+d}',
                          fill=YELLOW, font=('Menlo', 8), tags='det')

    # ─── Color Picking ───────────────────────────────────────────

    def _on_canvas_click(self, event):
        img = self.webcam.grab() if HAS_CV2 else None
        if img is None: return
        y = min(max(event.y, 0), img.shape[0]-1)
        x = min(max(event.x, 0), img.shape[1]-1)
        y0, y1 = max(0, y-2), min(img.shape[0], y+3)
        x0, x1 = max(0, x-2), min(img.shape[1], x+3)
        patch = img[y0:y1, x0:x1]
        sr, sg, sb = int(patch[:,:,0].mean()), int(patch[:,:,1].mean()), int(patch[:,:,2].mean())
        self._samples.append((sr, sg, sb))
        self._log.append(f'Sample {len(self._samples)}: RGB({sr},{sg},{sb})')
        self._compute_range()
        self._update_log()

    def _clear_samples(self):
        self._samples.clear()
        self.sample_lbl.config(text='  0 samples')
        self._log.append('Samples cleared')
        self._update_log()

    def _compute_range(self):
        if not self._samples: return
        hues = []
        for sr, sg, sb in self._samples:
            h, s, v = colorsys.rgb_to_hsv(sr/255, sg/255, sb/255)
            hues.append(h * 360)
        avg_r = int(sum(s[0] for s in self._samples) / len(self._samples))
        avg_g = int(sum(s[1] for s in self._samples) / len(self._samples))
        avg_b = int(sum(s[2] for s in self._samples) / len(self._samples))
        if len(hues) >= 2:
            center_h = colorsys.rgb_to_hsv(avg_r/255, avg_g/255, avg_b/255)[0] * 360
            max_dist = max(abs((h - center_h + 180) % 360 - 180) for h in hues)
            hue_range = max(10.0, min(60.0, max_dist + 5.0))
        else:
            hue_range = 15.0
        self.r.set(avg_r); self.g.set(avg_g); self.b.set(avg_b)
        self.hue.set(round(hue_range, 1)); self.sat.set(1.0)
        self.sample_lbl.config(text=f'  {len(self._samples)} sample{"s" if len(self._samples)!=1 else ""}')

    # ─── Hue Bar ─────────────────────────────────────────────────

    def _build_hue_bar(self):
        w, h = 220, 24
        img = tk.PhotoImage(width=w, height=h)
        for x in range(w):
            r, g, b = colorsys.hsv_to_rgb(x/w, 1.0, 1.0)
            img.put(f'#{int(r*255):02x}{int(g*255):02x}{int(b*255):02x}', to=(x,0,x+1,h))
        self._hue_img = img

    def _draw_hue_overlay(self):
        c = self.hue_cv; c.delete('all')
        w, h = 220, 24
        c.create_image(0, 0, anchor='nw', image=self._hue_img)
        rv, gv, bv = self.r.get(), self.g.get(), self.b.get()
        hsv = colorsys.rgb_to_hsv(rv/255, gv/255, bv/255)
        cd = hsv[0]*360; ha = self.hue.get()
        cp = int(cd/360*w)%w; hp = max(1, int(ha/360*w))
        lo, hi = cp-hp, cp+hp
        if lo >= 0 and hi <= w:
            c.create_rectangle(0,0,lo,h, fill='black', stipple='gray50', outline='')
            c.create_rectangle(hi,0,w,h, fill='black', stipple='gray50', outline='')
        elif lo < 0:
            c.create_rectangle(lo+w,0,w,h, fill='black', stipple='gray50', outline='')
            c.create_rectangle(hi,0,lo+w,h, fill='black', stipple='gray50', outline='')
        else:
            c.create_rectangle(hi%w,0,lo,h, fill='black', stipple='gray50', outline='')
        c.create_line(cp,0,cp,h, fill='white', width=2)
        c.create_rectangle(max(0,lo),0,min(w,hi),h, outline='white', width=1)

    # ─── Sig Display ─────────────────────────────────────────────

    def _refresh_sig(self):
        c = self.preview; c.delete('all')
        rv, gv, bv = self.r.get(), self.g.get(), self.b.get()
        c.create_rectangle(0,0,220,32, fill=f'#{rv:02x}{gv:02x}{bv:02x}', outline=BG3)
        lum = 0.299*rv+0.587*gv+0.114*bv
        c.create_text(110,16, text=f'RGB({rv},{gv},{bv})',
                      fill='#000' if lum>128 else '#fff', font=('Menlo',9,'bold'))
        ha, hs = self.hue.get(), self.sat.get()
        self.out_lbl.config(
            text=f'colordesc(1, {rv}, {gv}, {bv}, {ha:.2f}, {hs:.2f})')
        self._draw_hue_overlay()

    def _on_sig(self, *_): self._refresh_sig()

    # ─── Serial ──────────────────────────────────────────────────

    def _on_serial_line(self, line):
        with self._lock: self._line_q.append(line)

    def _process_lines(self):
        with self._lock:
            lines = self._line_q[:]; self._line_q.clear()
        for line in lines:
            m = RE_VIS.match(line)
            if m:
                cx, w, h, cnt = int(m.group(1)), int(m.group(2)), int(m.group(3)), int(m.group(4))
                self.frame = Frame(exists=cnt>0, count=cnt, cx=cx, cy=SENSOR_H//2, w=w, h=h)
                self.history.append(self.frame)
                self._last_vis = time.time()

    # ─── Polling ─────────────────────────────────────────────────

    def _poll(self):
        self._process_lines()
        now = time.time()
        if now - self._last_vis > 0.3 and self.sensor.connected:
            objs = self.sensor.get_objects()
            if objs:
                o = objs[0]
                self.frame = Frame(exists=True, count=len(objs),
                                   cx=o['cx'], cy=o['cy'], w=o['w'], h=o['h'])
                self.history.append(self.frame)

        if self._last_vis > 0 and now - self._last_vis > 0.5 and self.frame.exists:
            self.frame = Frame()
            self.history.append(self.frame)

        # Detect disconnected Brain (serial thread died)
        if self.brain_user.ser and not self.brain_user.running:
            self.brain_user.disconnect()

        # Indicators
        if self.brain_user.connected:
            self.ind_brain.config(fg=GREEN if self._last_vis > 0 and now - self._last_vis < 1.0 else YELLOW)
        else:
            self.ind_brain.config(fg=RED)
        self.ind_sensor.config(fg=GREEN if self.sensor.connected else RED)
        self.ind_cam.config(fg=GREEN if (HAS_CV2 and self.webcam.grab() is not None) else RED)

        self._draw_cam_frame()
        self._draw_detections()
        self._update_stats()
        self._update_stability()
        self._update_log()
        self.root.after(33, self._poll)

    # ─── Stats ───────────────────────────────────────────────────

    def _update_stats(self):
        f = self.frame
        if not self.history:
            self.stats.config(text='', fg=DIM); return
        if f.exists:
            self.stats.config(
                text=f'Detected: {f.count}   Center: ({f.cx},{f.cy})   '
                     f'Size: {f.w}\u00d7{f.h}   Error: {f.cx-TARGET_X:+d}px', fg=GREEN)
        else:
            self.stats.config(text='No object detected', fg=RED)

    def _update_stability(self):
        c = self.stab_cv; c.delete('all')
        if len(self.history) < 5:
            self.stab_lbl.config(text='\u2014', fg=DIM); return
        recent = list(self.history)[-50:]
        hits = sum(1 for f in recent if f.exists)
        pct = hits/len(recent)
        color = GREEN if pct>0.8 else YELLOW if pct>0.5 else RED
        c.create_rectangle(0,0,int(220*pct),12, fill=color, outline='')
        self.stab_lbl.config(text=f'{hits}/{len(recent)} ({pct:.0%})', fg=color)

    def _update_log(self):
        self.log_lbl.config(text='\n'.join(list(self._log)[-4:]))

    # ─── Connection ──────────────────────────────────────────────

    def _force_release_port(self, port_dev):
        """Kill any OTHER process holding this serial port."""
        my_pid = os.getpid()
        killed = False
        for dev in [port_dev, port_dev.replace('/dev/cu.', '/dev/tty.')]:
            try:
                out = subprocess.check_output(
                    ['lsof', dev], stderr=subprocess.DEVNULL, text=True)
                for line in out.strip().split('\n')[1:]:
                    parts = line.split()
                    if len(parts) >= 2:
                        proc_name, pid = parts[0], int(parts[1])
                        if pid == my_pid:
                            continue
                        # Don't kill system processes
                        if proc_name.lower() in ('kernel_task', 'launchd'):
                            continue
                        os.kill(pid, signal.SIGKILL)
                        self._log.append(f'Killed {proc_name} (pid {pid})')
                        killed = True
            except:
                pass
        if killed:
            time.sleep(0.5)
        return killed

    def _try_connect_brain(self):
        """Try to connect to Brain user port, force-killing holders if needed."""
        ports = list(serial.tools.list_ports.comports())
        brain_ports = sorted([p.device for p in ports if p.vid == 0x2888 and p.pid == 0x0501])
        if not brain_ports:
            return False

        for port in reversed(brain_ports):
            if self.brain_user.connect(port):
                self._log.append(f'Brain: {port}')
                return True
            # Port busy — force release and retry
            if self._force_release_port(port):
                if self.brain_user.connect(port):
                    self._log.append(f'Brain: {port}')
                    return True
        return False

    def _try_connect_sensor(self):
        ports = list(serial.tools.list_ports.comports())
        sensor_port = next((p.device for p in ports if p.vid == 0x2888 and p.pid == 0x0800), None)
        if sensor_port and self.sensor.connect(sensor_port):
            self._log.append(f'Sensor: {sensor_port}')
            return True
        return False

    def _connect_all(self):
        self._log.clear()
        self._try_connect_brain()
        self._try_connect_sensor()
        if HAS_CV2 and self.webcam.grab() is None:
            if self.webcam.start():
                self._log.append('Webcam: active')
        self._update_log()

    def _auto_reconnect(self):
        """Called periodically to reconnect dropped sources."""
        if not self.brain_user.connected:
            self._try_connect_brain()
        if not self.sensor.connected:
            self._try_connect_sensor()
        if HAS_CV2 and self.webcam.grab() is None:
            self.webcam.start()
        self.root.after(3000, self._auto_reconnect)

    # ─── Actions ─────────────────────────────────────────────────

    def _push_live(self):
        """Push signature to Brain via user serial (needs Brain-side listener)."""
        rv, gv, bv = self.r.get(), self.g.get(), self.b.get()
        ha, hs = int(self.hue.get()*100), int(self.sat.get()*100)
        cmd = f'C,1,{rv},{gv},{bv},{ha},{hs}\n'
        if self.brain_user.connected and self.brain_user.ser:
            try:
                self.brain_user.ser.write(cmd.encode('ascii'))
                self._log.append(f'Pushed: RGB({rv},{gv},{bv}) hue\u00b1{self.hue.get():.0f}')
                self.status.config(text='Pushed!', fg=GREEN)
            except:
                self._log.append('Push failed (serial error)')
        else:
            self._log.append('Push failed (Brain not connected)')
        self._update_log()

    def _apply_to_code(self):
        rv, gv, bv = self.r.get(), self.g.get(), self.b.get()
        ha, hs = self.hue.get(), self.sat.get()
        new_line = f'vex::aivision::colordesc yellow = vex::aivision::colordesc(1, {rv}, {gv}, {bv}, {ha:.2f}, {hs:.2f});'
        config_path = os.path.normpath(os.path.join(SCRIPT_DIR, '..', 'src', 'robot-config.cpp'))
        try:
            with open(config_path, 'r') as f: content = f.read()
            pattern = r'vex::aivision::colordesc yellow\s*=\s*vex::aivision::colordesc\([^)]+\);'
            if re.search(pattern, content):
                with open(config_path, 'w') as f: f.write(re.sub(pattern, new_line, content))
                self._log.append(f'Applied: ({rv},{gv},{bv}) hue\u00b1{ha:.0f}')
                self.status.config(text='Code updated!', fg=GREEN)
            else:
                self._log.append('Pattern not found in robot-config.cpp')
        except Exception as e:
            self._log.append(f'Error: {e}')
        self._update_log()

    def _copy(self):
        rv, gv, bv = self.r.get(), self.g.get(), self.b.get()
        ha, hs = self.hue.get(), self.sat.get()
        line = f'vex::aivision::colordesc yellow = vex::aivision::colordesc(1, {rv}, {gv}, {bv}, {ha:.2f}, {hs:.2f});'
        self.root.clipboard_clear()
        self.root.clipboard_append(line)
        self._log.append('Copied to clipboard')
        self._update_log()

    def run(self):
        try:
            self.root.mainloop()
        finally:
            self.brain_user.disconnect()
            self.sensor.disconnect()
            self.webcam.stop()


if __name__ == '__main__':
    VisionTuner().run()
