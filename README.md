# Tello VIO (C++)

C++ project for interacting with DJI/RYZE Tello (command/state/video) with a real-time monocular Visual Odometry pipeline that uses onboard telemetry for scale estimation.

## Project status

This project is **in progress**. The current state estimation accuracy is not good yet and is actively being optimized (sensor conditioning, timing/synchronization, and the estimation pipeline).

## Prerequisites

```bash
# Ubuntu / Debian
sudo apt install libasio-dev libopencv-dev libeigen3-dev libavcodec-dev libavutil-dev libswscale-dev
```

## Build

### With Visual Odometry (default)

```bash
mkdir -p build && cd build
cmake ..
make -j4
```

The VO pipeline is compiled into the main `tello` binary and runs automatically when the drone streams video.

### Without Visual Odometry

If you only need command/video/state without VO, comment out the VO source in `CMakeLists.txt` (the `monocular_vo/src/vo.cpp` line in `add_executable`) and remove the VO thread in `main/main.cpp`. The project will fall back to showing the raw pilot view.

## Run

1. Power on the Tello and connect to its Wi-Fi.
2. From the build directory:

```bash
./tello
```

**What happens at startup:**

1. `CommandSocket` sends `command` (SDK mode) and `streamon` (start video).
2. `VideoSocket` receives H.264 packets, decodes them into BGR frames, and pushes them into a `FrameQueue`.
3. `StateSocket` receives telemetry at ~10 Hz and stores velocity/height for VO.
4. A dedicated VO thread pops frames from the queue, undistorts them, runs feature tracking, and estimates the camera trajectory in real time.
5. Two OpenCV windows appear: **Tello Camera** (undistorted live view) and **Trajectory** (2-D overhead plot of the path).

Keyboard controls (while an OpenCV window is focused):

| Key | Action |
|-----|--------|
| `T` | Takeoff |
| `L` | Land |
| `W/A/S/D` | Forward / Left / Back / Right |
| `R/F` | Up / Down |
| `Q/E` | Yaw left / Yaw right |
| `Space` | Hover (zero RC) |
| `X` | Emergency stop |
| `ESC` | Land and exit |

## How inertial / state data is used

The Tello broadcasts a state string on UDP port 8890 at ~10 Hz. `StateSocket` parses every message and exposes the following to other components:

| Field | Unit | Current usage |
|-------|------|---------------|
| `vgx`, `vgy`, `vgz` | cm/s | **VO scale estimation** — converted to m/s, the speed magnitude is multiplied by the inter-frame `dt` to recover the metric displacement that monocular VO cannot determine from images alone. |
| `h` (height) | cm | Exposed for future use (altitude hold, ground-plane constraint). |
| `agx`, `agy`, `agz` | cm/s² | Logged to terminal at 1 Hz. Not yet fused into the VO pipeline. |
| `pitch`, `roll`, `yaw` | degrees | Logged to terminal at 1 Hz. Not yet fused into the VO pipeline. |
| `bat` (battery) | % | Readable via `getLastBattery()`. |

### Scale estimation in detail

Monocular VO recovers the translation direction from the essential matrix, but its magnitude is ambiguous (up-to-scale). The project resolves this with the drone's velocity telemetry:

```
scale = sqrt(vgx² + vgy² + vgz²) / 100  ×  dt
              ^^^^^^^^^^^^^^^^^^^^            ^^
              speed in m/s                    seconds between frames
```

The resulting `scale` (meters) is applied to the unit translation vector before accumulating the pose:

```
t_total = t_total + scale × (R_total × t_current)
R_total = R_current × R_total
```

### What is not yet used (future work)

- **Accelerometer / gyroscope fusion** — pitch/roll/yaw and accelerometer readings could provide an orientation prior or be fused in a full Visual-Inertial Odometry (VIO) filter (e.g. EKF/MSCKF).
- **Height constraint** — the `h` / barometer / ToF values could constrain the vertical component of the trajectory.
- **Attitude-based motion check** — large attitude changes could trigger re-detection instead of waiting for feature count to drop.

## Useful CMake options

| Option | Default | Description |
|--------|---------|-------------|
| `USE_JOYSTICK` | ON | Joystick control |
| `RUN_SLAM` | OFF | Build with OpenVSLAM integration |
| `RECORD` | OFF | Record video to MP4 |
| `USE_TERMINAL` | OFF | Enable CLI terminal |
| `USE_CONFIG` | OFF | Load drone config from `config.yaml` |
| `SIMPLE` | OFF | Simpler terminal output |

## Architecture overview

```
main/main.cpp          Entry point — creates Tello, FrameQueue, launches VO thread
 │
 ├─ CommandSocket      UDP 8889  — sends SDK commands, receives ack
 ├─ VideoSocket        UDP 11111 — receives H.264, decodes via lib_h264decoder,
 │                                  pushes cv::Mat to FrameQueue
 ├─ StateSocket        UDP 8890  — parses telemetry, exposes velocity/height
 │
 └─ VisualOdometry     Runs on its own thread
      ├─ Reads frames from FrameQueue
      ├─ Undistorts using camera_config.yaml calibration
      ├─ FAST feature detection + KLT optical flow tracking
      ├─ Essential matrix → recoverPose (rotation + unit translation)
      ├─ Scale from StateSocket velocity × dt
      └─ Displays trajectory in real time
```

## Camera calibration

Camera intrinsics and distortion coefficients are stored in `camera_config.yaml`. The VO reads `Camera.fx`, `Camera.fy`, `Camera.cx`, `Camera.cy`, and `Camera.k1`–`Camera.k5` at startup. If the file is missing, hardcoded Tello defaults are used.

## Tools / component checks

### IMU/state stream check

Connect to Tello Wi-Fi, power on the drone, then:

```bash
./tello_imu_check
```

### Component tests

This repo contains standalone component tests under `tests/` (command/state/video/joystick). Build/run them from their own CMake setup if needed.
