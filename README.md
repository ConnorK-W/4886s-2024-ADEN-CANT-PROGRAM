# 4886S - Sigma Sigma On The Wall

VEX V5 competition robot code for team 4886S.

## Vision Tuning Tool

Mac app for calibrating the AI Vision sensor's color signature under different lighting conditions. Uses your Mac's webcam to sample colors and push them to the robot in real-time.

### Setup (one time)

```bash
python3.12 -m venv tools/.venv
tools/.venv/bin/pip install pyserial opencv-python Pillow
```

### Run

```bash
./tools/.venv/bin/python3 tools/vision_tuner.py
```

### Usage

1. Plug the V5 Brain into your Mac via USB
2. Run your program on the Brain so `vision_processing_task` is active
3. Launch the tuning tool
4. The webcam feed appears in the main canvas
5. **Click** on the object you want to detect (the yellow goal, etc.) to sample its color
6. Click multiple spots (normal, shaded, glare) to widen the hue range automatically
7. **Apply to Code** writes the new `colordesc` directly into `src/robot-config.cpp`
8. **Push Live to Brain** updates the sensor signature in real-time (no recompile needed)
9. **Copy Line** copies the `colordesc(...)` line to your clipboard

### Features

- Auto-detects Brain, AI Vision sensor, and webcam connections
- Status indicators show which data sources are active
- Multi-sample color picking computes optimal hue range
- Hue bar visualization shows accepted color range
- Detection stability meter
- Auto-reconnects if Brain is power-cycled

Hancock Technologies
