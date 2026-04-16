#!/usr/bin/env python3
# Launch: ./tools/.venv/bin/python3 tools/blank_uploader.py
"""
Hancock Technologies(TM) Blank Program Uploader

Builds and/or uploads a blank VEX V5 program to a chosen slot on the Brain.
Useful for wiping a slot before a match, or clearing a flaky program.

Blank project: /Users/renner/Documents/vex-vscode-projects/blank
"""

import os
import sys
import pty
import queue
import shlex
import select
import threading
import subprocess
import tkinter as tk

# ── Paths ────────────────────────────────────────────────────────
BLANK_PROJECT = os.path.expanduser(
    '~/Documents/vex-vscode-projects/blank'
)

def _find_vexcode_global():
    """Locate the VEXcode extension's globalStorage — Cursor or VS Code."""
    candidates = [
        '~/Library/Application Support/Cursor/User/globalStorage/vexrobotics.vexcode',
        '~/Library/Application Support/Code/User/globalStorage/vexrobotics.vexcode',
    ]
    for p in candidates:
        p = os.path.expanduser(p)
        if os.path.isdir(p):
            return p
    return os.path.expanduser(candidates[0])

VEXCODE_GLOBAL = _find_vexcode_global()
VEXCOM_BIN = os.path.join(VEXCODE_GLOBAL, 'tools/vexcom/osx/vexcom')
SDK_CPP_V5 = os.path.join(VEXCODE_GLOBAL, 'sdk/cpp/V5')

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
LOGO_PATH  = os.path.join(SCRIPT_DIR, '..', 'Hancock-Sarah.jpg')

try:
    from PIL import Image as PILImage, ImageTk, ImageDraw
    HAS_PIL = True
except ImportError:
    HAS_PIL = False

# ── Theme (matches vision_tuner.py) ──────────────────────────────
BG       = '#0e0e12'
BG2      = '#161620'
BG3      = '#1e1e2e'
ACCENT   = '#c49a6c'
GREEN    = '#a6da95'
RED      = '#ed8796'
YELLOW   = '#eed49f'
TEXT     = '#cad3f5'
DIM      = '#5b6078'


def latest_sdk_path():
    """Return absolute path to the newest V5 SDK version directory, or None."""
    if not os.path.isdir(SDK_CPP_V5):
        return None
    versions = sorted(
        [d for d in os.listdir(SDK_CPP_V5)
         if os.path.isdir(os.path.join(SDK_CPP_V5, d))]
    )
    if not versions:
        return None
    return os.path.join(SDK_CPP_V5, versions[-1])


class BlankUploader:
    def __init__(self):
        self.root = tk.Tk()
        self.root.title('Hancock Technologies\u2122')
        self.root.configure(bg=BG)
        self.root.resizable(False, False)

        self.slot   = tk.IntVar(value=1)
        self.run_after = tk.BooleanVar(value=False)
        self._q = queue.Queue()
        self._busy = False

        self._build()
        self._poll_queue()

    # ── UI ──────────────────────────────────────────────────────
    def _build(self):
        # Header
        header = tk.Frame(self.root, bg=BG, padx=16, pady=12)
        header.pack(fill='x')

        self._logo_photo = None
        if HAS_PIL and os.path.exists(LOGO_PATH):
            try:
                img = PILImage.open(LOGO_PATH).resize((44, 44), PILImage.LANCZOS)
                mask = PILImage.new('L', (44, 44), 0)
                ImageDraw.Draw(mask).ellipse((0, 0, 44, 44), fill=255)
                img.putalpha(mask)
                self._logo_photo = ImageTk.PhotoImage(img)
                tk.Label(header, image=self._logo_photo, bg=BG
                         ).pack(side='left', padx=(0, 12))
            except Exception:
                pass

        brand = tk.Frame(header, bg=BG)
        brand.pack(side='left')
        tk.Label(brand, text='Hancock Technologies\u2122',
                 fg=ACCENT, bg=BG,
                 font=('Helvetica Neue', 16, 'bold')).pack(anchor='w')
        tk.Label(brand, text='Blank Program Uploader',
                 fg=DIM, bg=BG,
                 font=('Helvetica Neue', 11)).pack(anchor='w')

        self.status = tk.Label(header, text='\u25cf idle', fg=DIM, bg=BG,
                               font=('Menlo', 10))
        self.status.pack(side='right')

        tk.Frame(self.root, bg=BG3, height=1).pack(fill='x', padx=16)

        # Controls
        body = tk.Frame(self.root, bg=BG, padx=16, pady=12)
        body.pack(fill='both')

        row = tk.Frame(body, bg=BG)
        row.pack(fill='x', pady=(0, 8))
        tk.Label(row, text='SLOT', fg=DIM, bg=BG,
                 font=('Helvetica Neue', 9, 'bold')).pack(side='left')

        slot_row = tk.Frame(body, bg=BG)
        slot_row.pack(fill='x', pady=(0, 10))
        self._slot_btns = []
        for n in range(1, 9):
            b = tk.Label(
                slot_row, text=str(n),
                bg=BG2, fg=TEXT,
                bd=0, highlightthickness=0,
                padx=10, pady=6,
                font=('Menlo', 11, 'bold'),
                cursor='hand2',
            )
            b.pack(side='left', padx=(0, 4))
            b.bind('<Button-1>', lambda _e, v=n: self._select_slot(v))
            self._slot_btns.append(b)
        self._select_slot(self.slot.get())

        tk.Checkbutton(body, text='run after upload',
                       variable=self.run_after,
                       bg=BG, fg=TEXT, selectcolor=BG2,
                       activebackground=BG, activeforeground=TEXT,
                       bd=0, highlightthickness=0,
                       font=('Helvetica Neue', 10)).pack(anchor='w', pady=(0, 10))

        btn_row = tk.Frame(body, bg=BG)
        btn_row.pack(fill='x', pady=(0, 12))
        self.btn_upload = self._button(btn_row, 'Upload', self._on_upload)
        self.btn_upload.pack(side='left')
        self.btn_build = self._button(btn_row, 'Build & Upload', self._on_build_upload)
        self.btn_build.pack(side='left', padx=(8, 0))

        tk.Label(body, text='LOG', fg=DIM, bg=BG,
                 font=('Helvetica Neue', 9, 'bold')).pack(anchor='w')

        log_frame = tk.Frame(body, bg=BG3, bd=0)
        log_frame.pack(fill='both', pady=(4, 0))
        self.log = tk.Text(log_frame, width=72, height=16,
                           bg='#0b0b10', fg=TEXT, insertbackground=TEXT,
                           bd=0, highlightthickness=1, highlightbackground=BG3,
                           font=('Menlo', 10), wrap='word')
        self.log.pack(fill='both', padx=1, pady=1)
        self.log.configure(state='disabled')

    def _button(self, parent, label, cmd):
        return tk.Button(parent, text=label, command=cmd,
                         bg=ACCENT, fg='#1a1208',
                         activebackground='#d9ad7f', activeforeground='#1a1208',
                         bd=0, highlightthickness=0,
                         padx=16, pady=8,
                         font=('Helvetica Neue', 11, 'bold'))

    # ── Actions ─────────────────────────────────────────────────
    def _on_upload(self):
        self._start([self._step_upload])

    def _on_build_upload(self):
        self._start([self._step_build, self._step_upload])

    def _start(self, steps):
        if self._busy:
            return
        self._busy = True
        self._set_status('running', YELLOW)
        self._set_buttons(False)
        self._clear_log()
        threading.Thread(target=self._run_steps, args=(steps,), daemon=True).start()

    def _run_steps(self, steps):
        try:
            for step in steps:
                ok = step()
                if not ok:
                    self._q.put(('done', False))
                    return
            self._q.put(('done', True))
        except Exception as e:
            self._q.put(('log', f'[error] {e}\n'))
            self._q.put(('done', False))

    # Build step
    def _step_build(self):
        if not os.path.isdir(BLANK_PROJECT):
            self._q.put(('log', f'blank project not found: {BLANK_PROJECT}\n'))
            return False
        sdk = latest_sdk_path()
        if not sdk:
            self._q.put(('log', f'no V5 SDK found under: {SDK_CPP_V5}\n'))
            return False
        cmd = ['make', f'T={sdk}', '-C', BLANK_PROJECT, '-j4']
        self._q.put(('log', '$ ' + ' '.join(shlex.quote(a) for a in cmd) + '\n'))
        return self._stream(cmd)

    # Upload step
    def _step_upload(self):
        bin_path = os.path.join(BLANK_PROJECT, 'build', 'blank.bin')
        if not os.path.exists(bin_path):
            self._q.put(('log',
                f'blank.bin not found at {bin_path}\nuse "Build & Upload" first\n'))
            return False
        if not os.path.exists(VEXCOM_BIN):
            self._q.put(('log', f'vexcom not found: {VEXCOM_BIN}\n'))
            return False
        cmd = [VEXCOM_BIN,
               '--name', 'blank',
               '--slot', str(self.slot.get()),
               '--write', bin_path,
               '--progress']
        if self.run_after.get():
            cmd.append('--run')
        self._q.put(('log', '$ ' + ' '.join(shlex.quote(a) for a in cmd) + '\n'))
        return self._stream(cmd)

    # ── Subprocess streamer ─────────────────────────────────────
    # Runs cmd attached to a pty so tools like vexcom emit their live
    # progress bar (they suppress it when stdout isn't a TTY).
    def _stream(self, cmd):
        try:
            master_fd, slave_fd = pty.openpty()
            p = subprocess.Popen(cmd, stdin=slave_fd,
                                 stdout=slave_fd, stderr=slave_fd,
                                 close_fds=True)
        except FileNotFoundError as e:
            self._q.put(('log', f'[error] {e}\n'))
            return False
        os.close(slave_fd)

        buf = bytearray()
        try:
            while True:
                try:
                    r, _, _ = select.select([master_fd], [], [], 0.1)
                except (OSError, ValueError):
                    break
                if r:
                    try:
                        chunk = os.read(master_fd, 4096)
                    except OSError:
                        chunk = b''
                    if not chunk:
                        break
                    for b in chunk:
                        bc = bytes([b])
                        if bc in (b'\n', b'\r'):
                            if buf:
                                self._q.put(('log_line',
                                    buf.decode('utf-8', 'replace')))
                                buf.clear()
                        else:
                            buf.append(b)
                if p.poll() is not None and not r:
                    break
        finally:
            try:
                os.close(master_fd)
            except OSError:
                pass

        if buf:
            self._q.put(('log_line', buf.decode('utf-8', 'replace')))
        rc = p.wait()
        if rc != 0:
            self._q.put(('log', f'[exit {rc}]\n'))
            return False
        return True

    # ── UI helpers ──────────────────────────────────────────────
    def _poll_queue(self):
        try:
            while True:
                kind, payload = self._q.get_nowait()
                if kind == 'log':
                    self._append_log(payload)
                elif kind == 'log_line':
                    self._replace_last_line(payload)
                elif kind == 'done':
                    self._busy = False
                    self._set_buttons(True)
                    if payload:
                        self._set_status('done', GREEN)
                    else:
                        self._set_status('failed', RED)
        except queue.Empty:
            pass
        self.root.after(50, self._poll_queue)

    def _append_log(self, text):
        self.log.configure(state='normal')
        self.log.insert('end', text)
        self.log.see('end')
        self.log.configure(state='disabled')

    def _replace_last_line(self, text):
        self.log.configure(state='normal')
        last = self.log.get('end-2c linestart', 'end-1c')
        if last and not last.endswith('\n'):
            self.log.delete('end-2c linestart', 'end-1c')
        self.log.insert('end', text + '\n')
        self.log.see('end')
        self.log.configure(state='disabled')

    def _clear_log(self):
        self.log.configure(state='normal')
        self.log.delete('1.0', 'end')
        self.log.configure(state='disabled')

    def _set_status(self, text, color):
        self.status.config(text=f'\u25cf {text}', fg=color)

    def _select_slot(self, n):
        if self._busy:
            return
        self.slot.set(n)
        for i, b in enumerate(self._slot_btns, start=1):
            if i == n:
                b.config(bg=ACCENT, fg='#1a1208')
            else:
                b.config(bg=BG2, fg=TEXT)

    def _set_buttons(self, enabled):
        state = 'normal' if enabled else 'disabled'
        self.btn_upload.config(state=state)
        self.btn_build.config(state=state)

    def run(self):
        self.root.mainloop()


if __name__ == '__main__':
    try:
        BlankUploader().run()
    except KeyboardInterrupt:
        sys.exit(0)
