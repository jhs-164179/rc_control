# depth_transport

Low-latency, lossless Zstd transport for ROS 2 `16UC1` depth images. The sender
keeps only the newest input frame, compresses it on a dedicated worker thread,
and publishes a `sensor_msgs/CompressedImage`. The receiver reconstructs the
standard `sensor_msgs/Image` message byte-for-byte.

An optional lossy 8-bit JPEG depth preview is available for monitoring. Never
use that preview topic for measurement, SLAM, or point-cloud generation.

## Dependencies

Run in both the Radxa and laptop ROS containers:

```bash
apt-get update
apt-get install -y libzstd-dev ros-humble-cv-bridge
```

## Build

Place this package in the same ROS workspace on both machines:

```bash
mkdir -p ~/ros2_ws/src
cp -r depth_transport ~/ros2_ws/src/
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select depth_transport
source install/setup.bash
```

## Radxa sender

Start the RealSense node first, then:

```bash
ros2 run depth_transport depth_zstd_sender --ros-args \
  -p input_topic:=/camera/camera/depth/image_rect_raw \
  -p output_topic:=/camera/transport/depth_zstd \
  -p zstd_level:=1 \
  -p max_fps:=30.0 \
  -p preview_enabled:=false
```

The sender prints the produced FPS, network payload, compression ratio, average
encoding time, and number of frames replaced before encoding.

For a display-only JPEG preview:

```bash
ros2 run depth_transport depth_zstd_sender --ros-args \
  -p preview_enabled:=true \
  -p preview_quality:=85 \
  -p preview_min_mm:=70 \
  -p preview_max_mm:=500
```

## Laptop receiver

```bash
ros2 run depth_transport depth_zstd_receiver --ros-args \
  -p input_topic:=/camera/transport/depth_zstd \
  -p output_topic:=/camera/transport/depth_raw
```

Point the existing viewer at `/camera/transport/depth_raw`.

## Validate

```bash
ros2 topic hz /camera/transport/depth_zstd
ros2 topic bw /camera/transport/depth_zstd
ros2 topic hz /camera/transport/depth_raw
ros2 topic bw /camera/transport/depth_raw
```

Measure sender CPU in another terminal:

```bash
pidstat -C depth_zstd_sender 1
```

Start with `zstd_level:=1`. If the encoded payload remains too large, compare
levels 2 and 3. If encoding exceeds about 20 ms/frame or frames are frequently
replaced, return to level 1 or set `max_fps:=20.0`.

Do not run a laptop subscriber on the original raw depth topic during this
test; otherwise DDS will transmit both the raw and Zstd streams over Wi-Fi.
