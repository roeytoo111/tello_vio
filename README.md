# Tello (C++)

C++ project for interacting with DJI/RYZE Tello (command/state/video), plus ongoing work toward more accurate state estimation / VIO.

## Project status

This project is **in progress**. The current **state estimation accuracy is not good yet** and is actively being optimized (sensor parsing/conditioning, timing/synchronization, and the estimation pipeline).

## Build

```bash
sudo apt install libasio-dev libopencv-dev
mkdir -p build && cd build
cmake ..
make -j4
```

## Run

```bash
./tello
```

## Useful CMake options

- **`USE_JOYSTICK`**: joystick control (default ON)
- **`RUN_SLAM`**: build with OpenVSLAM integration (default OFF)
- **`RECORD`**: record video (default OFF)
- **`USE_TERMINAL`**: enable CLI terminal (default OFF)
- **`USE_CONFIG`**: load config from `config.yaml` (default OFF)
- **`SIMPLE`**: simpler terminal output (default OFF)

## Tools / component checks

### IMU/state stream check

Connect to Tello Wi‑Fi, power on the drone, then:

```bash
./tello_imu_check
```

### Component tests

This repo contains standalone component tests under `tests/` (command/state/video/joystick). Build/run them from their own CMake setup if needed.

## Monocular VO demo (KITTI)

`monocular_vo/` contains a monocular visual odometry demo intended for KITTI grayscale sequences and ground-truth pose evaluation.
