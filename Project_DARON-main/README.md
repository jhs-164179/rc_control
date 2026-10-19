# Project_DARON — DARON 통합 실행 코드 (261011)

굴절형 4륜 로버 DARON 을 돌리는 데 필요한 코드를 한 곳에 모은 묶음이다.

- **RADXA 도커 (로버 본체)**: 로버 제어 노드(굴절·서스펜션 / 조향 / 구동) + D405 카메라 + CV7 IMU + depth zstd 송신
- **지상국 (우분투)**: cuVSLAM·EKF·녹화, 3D 피쳐맵·주행가능성 판정·경로계획, 지상국 GUI(RViz 2창 + 제어창 + 핀 갤러리),
  YOLO 크레이터 검출, 경로 추종기

모든 파일은 10/11 22시 기준 원본의 **무수정 사본**이다. 예외:
- `radxa_docker/daron_cv7.yml` — `daron_run/config/daron_cv7.yml` 에서 port 만 `/dev/ttyACM0`
- `Daron/daron_run/launch/daron_rover.launch.py` — CV7 설정이 없을 때의 오류 문구만 이 묶음의 `radxa_docker/daron_cv7.yml` 을 가리키게
- `daron-nav/daron_planner/ros/pose_relay.py` — 원본 `~/daron-nav` 에서 추가

---

## 1. 전체 구성

```
 ┌──────────── RADXA 도커 (로버) ─────────────┐   WiFi / DDS    ┌──────────────── 지상국 ────────────────────┐
 │ daron_rover.launch.py                      │  DOMAIN 42      │ daron_viz.launch.py  (source:=remote)       │
 │  ├ rover/radxa.py      /rover 노드         │ ─────────────►  │  ├ image_relay  /radxa/camera/* → /camera/… │
 │  │   XM430 굴절·서스펜션, L12 조향, VESC 구동│ /radxa/camera/* │  ├ video_receiver  zstd depth → raw depth   │
 │  ├ realsense2_camera  ns=radxa             │ /ekf/imu/data   │  ├ cuVSLAM, EKF-A/B/C, health, 녹화          │
 │  └ microstrain CV7                         │ /camera/        │ daron_gui.launch.py                         │
 │ depth_zstd_sender (depth 만 압축)          │  transport/…    │  ├ daron_3dmap (피쳐맵·판정·planner_node)    │
 │                                            │                 │  ├ RViz 3D + 탑뷰, 제어창, 핀 갤러리          │
 │                                            │ ◄─────────────  │ YOLO ros2_scnet_depth_node → /cda/detections│
 │                                            │ /rover/*_cmd    │ pose_relay → /localization/pose             │
 │                                            │ /rover/auto/*   │ rover/server/path_follower → /rover/auto/*  │
 └────────────────────────────────────────────┘                 └─────────────────────────────────────────────┘
```

### 폴더

```
Project_DARON/
├── Daron/
│   ├── slam_env.sh                     지상국 환경 (venv + ROS2 + cuVSLAM, DOMAIN=42, CycloneDDS)
│   ├── daron_run/
│   │   ├── launch/   daron_viz (센서·SLAM·EKF·녹화) · daron_gui (매핑+RViz+GUI) · daron_3dmap (매핑)
│   │   │             ekf (daron_viz 가 include) · daron_rover (RADXA: 로버+카메라+CV7)
│   │   ├── nodes/    cuVSLAM·IMU 릴레이·health·image_relay·judge_core, lib/
│   │   │   ├── 3dmap/  live_tsdf_node, traversability_node, depth_colorize + 보조
│   │   │   ├── cda/    pin_node, fake_detector, cda_overlay
│   │   │   └── gui/    ground_station(제어창), pin_gallery, waypoint_seq, rover_teleop·rover_panel, rim/, crater/
│   │   ├── depth_transport/video_receiver.py   zstd depth 수신·재발행 (daron_viz 가 띄움)
│   │   ├── config/   ekf*.yaml, traversability.yaml, daron_cv7.yml, daron.urdf.xacro, record_topics_*.txt
│   │   ├── rviz/     daron_viz3d, daron_3dmap, daron_gui_topview
│   │   ├── tools/    record_rviz.sh, record_gui.sh (화면 녹화)
│   │   └── rover/    로버 제어 (radxa.py, console.py, 드라이버, 캘리브레이션 도구, profiles/, config/)
│   │       └── server/  지상국 경로 추종기 (path_follower, Stanley 곡률 제어)
│   └── yolo/scnet_cda_ros2_handoff_20261002_depth3d/   SCNet 크레이터 검출 ROS2 노드 + 가중치
├── daron-nav/daron_planner/            경로계획 코어 + ros/ (planner_node, pose_relay, live_lab.yaml)
└── radxa_docker/
    ├── depth_transport/                RADXA 도커 colcon 패키지 (depth_zstd_sender / receiver, C++)
    └── daron_cv7.yml                   RADXA 용 CV7 설정 (port=/dev/ttyACM0)
```

---

## 2. 놓을 곳

| 묶음 안 | 놓을 곳 | 쓰는 곳 |
|---|---|---|
| `Daron/slam_env.sh` | `~/Daron/slam_env.sh` (`~/slam_env.sh` 도 같은 내용) | 모든 지상국 터미널 첫 줄 |
| `Daron/daron_run/` | 지상국 `~/Daron/daron_run/` | daron_viz · daron_gui · daron_3dmap, 경로 추종기 |
| `Daron/daron_run/` (최소 `launch/` + `rover/`) | RADXA 도커 (예: `/workspace/daron_run/`) | `daron_rover.launch.py` 가 `../rover/radxa.py` 를 찾는다 |
| `Daron/yolo/scnet_cda_ros2_handoff_20261002_depth3d/` | 지상국 `~/Daron/yolo/…` 같은 경로 | YOLO 검출 노드 |
| `daron-nav/daron_planner/` | 지상국 `~/daron-nav/daron_planner/` | 경로계획 (`daron_3dmap.launch.py` 에 `~/daron-nav/...` 로 고정), pose_relay |
| `radxa_docker/depth_transport/` | RADXA 도커 colcon ws 의 `src/` | depth zstd 송신 |
| `radxa_docker/daron_cv7.yml` | RADXA 도커 `/workspace/daron_cv7.yml` | `daron_rover.launch.py` 의 `cv7_params` 기본값 |

```bash
# 지상국
rsync -a Project_DARON/Daron/     ~/Daron/
rsync -a Project_DARON/daron-nav/ ~/daron-nav/

# RADXA 도커
rsync -a Project_DARON/Daron/daron_run/ /workspace/daron_run/
cp Project_DARON/radxa_docker/daron_cv7.yml /workspace/daron_cv7.yml
# depth 송신기: 이미 빌드돼 있으면 그대로. 새로 깔 때만
cp -r Project_DARON/radxa_docker/depth_transport <ws>/src/
cd <ws> && colcon build --symlink-install --packages-select depth_transport && source install/setup.bash
```

launch 안의 경로는 `daron_run` 위치 기준 상대경로라 폴더째 옮겨도 된다.
bag 기본 폴더는 `daron_run` 옆 `daron_bags/`, 맵 기본 폴더는 옆 `maps/`.

---

## 3. 미리 있어야 하는 것 (환경 — 묶음에 없음)

### 지상국

- ROS 2 Humble + `realsense2_camera`, `robot_localization`, `microstrain_inertial_driver`, `rviz2`, `xacro`,
  `robot_state_publisher`, `interactive_markers`, `rmw_cyclonedds_cpp`
- `~/venv/cuvslam` (cuVSLAM 노드·zstd depth 수신: pycuvslam, `fire`, `zstandard`, cv2)
  + `~/Documents/cuVSLAM` v15.0.0 빌드 + `~/cuvslam_ws/install` — 경로가 `slam_env.sh`·`daron_viz.launch.py` 에 고정
- `~/venv/daron3d` (매핑·판정·경로계획·pose_relay: open3d 0.19, scipy)
- `/usr/bin/python3` 패키지 (나머지 노드·제어창·경로 추종기: numpy, scipy, cv2, PyQt5, vtk, yaml / 콘솔을 쓰면 pygame)
- `~/anaconda3/envs/cda` (YOLO: torch, ultralytics 8.3.203 — `Daron/yolo/.../requirements.txt`)
- 화면 녹화(`record_gui.sh`): ffmpeg(nvenc), xwininfo
- 데이터: `~/Daron/daron_bags/<bag>` 등 재생할 bag

### RADXA 도커

- ROS 2 Humble + `realsense2_camera`, `microstrain_inertial_driver`, `libzstd-dev`, `ros-humble-cv-bridge`
- 로버 파이썬 의존성: `pip3 install -r daron_run/rover/requirements.txt` (dynamixel-sdk, pyserial)
- 시리얼 권한(`dialout`) + FTDI latency 1 ms: `sudo sh daron_run/rover/setup_latency.sh`
  (안 하면 서스펜션 루프가 60 Hz 근처에서 막힌다. 부팅하면 되돌아감 → 스크립트 아래 udev 규칙 참고)
- L12 PWM: `rsetup` Overlays 에서 **PWM8(전방)·PWM9(후방)** 켜고 재부팅. `/sys` 쓰기 권한(root 또는 `--privileged`)
- **서스펜션 프로파일 `rover/profiles/rover.json`** — RADXA 마다 다른 캘리브레이션 결과라 묶음에 없다.
  기존 파일(`/workspace/DARON_keyboard_ctrl/profiles/rover.json`)을 `susp_profile:=` 로 주거나 새로 만든다 (6.1)

### 양쪽 공통

- `ROS_DOMAIN_ID=42`, `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`. 도메인이 다르면 에러 없이 토픽만 안 보인다.
- 멀티캐스트가 막힌 망이면 `export CYCLONEDDS_URI=file://<daron_run>/rover/cyclonedds.xml` (안의 Peer 주소를 상대 IP 로)

---

## 4. 실행 — 실차 (RADXA → WiFi → 지상국)

### RADXA 도커

```bash
# R1 로버 노드 + 카메라(ns=radxa) + CV7 한 번에
ros2 launch /workspace/daron_run/launch/daron_rover.launch.py ros_domain_id:=42 \
  susp_profile:=/workspace/DARON_keyboard_ctrl/profiles/rover.json
#   끄기: rover:=false camera:=false imu:=false / 구동 없이 vesc_enable:=false / 조향 없이 l12_enable:=false
#   하드웨어 없이 점검: susp_mock:=true vesc_enable:=false l12_enable:=false
#   그 밖의 radxa.py 파라미터: rover_params:='jog_limit_deg:=30.0 battery_warn_pct:=25.0'

# R2 depth zstd 송신 — 카메라가 ns=radxa 라 input_topic 을 꼭 바꿀 것 (기본값은 /camera/camera/...)
ros2 run depth_transport depth_zstd_sender --ros-args \
  -p input_topic:=/radxa/camera/depth/image_rect_raw \
  -p output_topic:=/camera/transport/depth_zstd -p zstd_level:=1 -p max_fps:=30.0
```

- 프로파일이 없으면 launch 가 기동 전에 멈추고 안내를 찍는다.
- 종료(Ctrl-C): 구동 0 → VESC off → L12 중립(최대 5 s) → 서스펜션 토크 유지 순. launch 는 이 프로세스만 15 s 기다린다.
- 지상국 `daron_viz` 의 `remote_depth:=zstd`(기본)는 R2 가 떠 있어야 depth 가 들어온다.
  R2 없이 쓰려면 지상국에서 `remote_depth:=raw` (WiFi 대역 더 씀).
- 카메라·CV7 을 따로 띄우던 옛 방식(R1 `rs_launch.py camera_namespace:=radxa …`, R2 `microstrain_launch.py params_file:=/workspace/daron_cv7.yml`)도
  그대로 된다. 그때는 `daron_rover.launch.py camera:=false imu:=false`.

### 지상국

모든 터미널 첫 줄은 `source ~/Daron/slam_env.sh`.

```bash
# 1. 센서 수신·SLAM·EKF·녹화
ros2 launch ~/Daron/daron_run/launch/daron_viz.launch.py source:=remote session_name:=<이름>
#   record:=false 로 녹화 끔 / slam_map_load:=<맵 폴더> 로 이전 맵에서 재추정 (출발 표시에 정지한 채)

# 2. 매핑 + 판정 + 경로계획 + RViz 2창 + 제어창 + 핀 갤러리
ros2 launch ~/Daron/daron_run/launch/daron_gui.launch.py session_name:=<1 과 같게>

# 3. YOLO 크레이터 검출 → /cda/detections
source /opt/ros/humble/setup.bash && export ROS_DOMAIN_ID=42 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
~/anaconda3/envs/cda/bin/python ~/Daron/yolo/scnet_cda_ros2_handoff_20261002_depth3d/ros2_scnet_depth_node.py

# 4. (선택) 화면 녹화 — 3D·탑뷰·제어창 → <prefix>_3d/_top/_ctl.mp4
~/Daron/daron_run/tools/record_gui.sh 360 ~/videos/site_01
```

- `daron_gui` 가 `daron_3dmap`(rviz:=false, cda:=pins, planner:=true)을 포함한다 → 3dmap·rviz2 를 따로 띄우지 말 것.
  끄기: `map:=false viz3d:=false topview:=false gui:=false pin_gallery:=false`.
- `daron_viz source:=remote` 는 `remote_source_check.py`(토픽 도달·지연 점검)와 `image_relay.py`(WiFi 1벌만 받아
  `/camera/camera/*` 로 재발행)를 같이 띄운다. `[relay] infra1 xxHz yyms` 로그가 30 Hz·지연 <0.3 s 면 정상.

### 지상국 GUI 사용

- **웨이포인트**: 두 RViz 창의 2D Goal Pose(G) → `/gui/waypoint_goal` → 제어창 목록(삭제·순서·일시정지)
  → 플래너(`/goal_pose`)에 지금 목표 하나씩. 누른 채 드래그한 방향 = 도착 방위.
  정밀하게는 **탑뷰**에서 (원근 3D 뷰는 z=0 평면 투영이라 바닥 높이만큼 어긋남).
  제어창 '영상에서 찍기' → D405 infra1 창에서 누르고 드래그 (바닥 0.07~1.8 m).
  기록 `<session_dir>/waypoints.jsonl`, 제어창에서 불러오기.
- **크레이터 핀**: RViz 핀 또는 제어창 표 행 클릭 → 정보 창 (대표 검출 프레임, 지름, 좌표, 림 선명도, 크레이터 경사·깊이, '3D 보기').
  이미지·`pins_info.json` 은 `~/daron_gui_runs/<시각>/pins/`.
- **수동 조종**: 제어창 로버 칸 '수동' 버튼 = 키보드 조종 (키는 6.2 와 같음). 로버 칸 상단 `BATT NN%` 배지.
  pygame 콘솔을 쓰려면 `python3 ~/Daron/daron_run/rover/console.py` — **제어창 '수동'과 동시에 켜지 말 것**.

---

## 5. 실행 — bag 재생 / 로컬 카메라

```bash
source ~/Daron/slam_env.sh

# bag 재생 + 지금 코드로 매핑·판정·경로계획을 다시 계산
ros2 launch ~/Daron/daron_run/launch/daron_viz.launch.py bag_path:=~/Daron/daron_bags/<bag> record:=false rebuild:=true
ros2 launch ~/Daron/daron_run/launch/daron_gui.launch.py use_sim_time:=true
#   녹화 당시 결과를 그대로 보려면 rebuild:=false + daron_gui map:=false

# 지상국에 D405·CV7 직결 (source:=local 기본)
ros2 launch microstrain_inertial_driver microstrain_launch.py \
  params_file:=$HOME/Daron/daron_run/config/daron_cv7.yml namespace:=/ configure:=true activate:=true
ros2 launch ~/Daron/daron_run/launch/daron_viz.launch.py session_name:=<이름> \
  record_topics:=$HOME/Daron/daron_run/config/record_topics_3dmap.txt
ros2 launch ~/Daron/daron_run/launch/daron_gui.launch.py session_name:=<같게>
```

- `bag_path` 와 `source:=remote` 는 같이 못 쓴다. bag 재생이면 `use_sim_time` 은 강제로 true.
- 장착 세트 `mount:=auto` — 실시간·remote 는 level(설계값), bag 은 `<bag>.daron_meta.yaml` 의 값 (없으면 10/8 이전 cad0902).

---

## 6. 로버 (`Daron/daron_run/rover/`)

상세: [`rover/README.md`](Daron/daron_run/rover/README.md) · 캘리브레이션 [`rover/CALIBRATION.md`](Daron/daron_run/rover/CALIBRATION.md)
· 경로 추종 [`rover/PATH_TRACKING.md`](Daron/daron_run/rover/PATH_TRACKING.md)

### 6.1 최초 1회 — 서스펜션 캘리브레이션 (RADXA, ROS 없이)

```bash
cd /workspace/daron_run/rover
python3 calibrate.py --auto-zero --out profiles/rover.json
```

### 6.2 조작 (제어창 '수동' / console.py 공통)

| 키 | 동작 |
|----|------|
| `W` / `S` | 전진 / 후진 (누르는 동안) |
| `A` / `D` | 조향 좌 / 우 (누르는 동안 최대 유효조향각 43°, 떼면 중립) |
| `Q` / `E` | 굴절각 감소 / 증가 (누르는 동안) |
| `SPACE` | 전복 토글 |
| `1` / `2` / `0` | 중립 모드(굴절 150°) / 수평 모드(180°) / 수납 모드 |
| `P` | (console.py 만) 자율 허용 토글 |
| `ESC` | (console.py) 종료 |

- 굴절각 부호: `alpha = 180 + theta`. 굴절각이 커질수록 굴절부가 내려오고, 더 키우면 앞바퀴가 들린다.
- 형상 전환 중·수납 상태면 수동·자율 모두 구동 0, 조향 중립.
- 배터리 잔량: VESC `battery_level` (front/back 중 낮은 쪽) → `/rover/diagnostics`·`/rover/battery`.

### 6.3 자율 주행 (경로 추종)

`planner_node` 의 `/planner/path`(map, 두 축 가운데 점) 와 `pose_relay` 의 `/localization/pose`(같은 점) 를 받아
`/rover/auto/drive_rpm`·`/rover/auto/steer_cmd` 를 낸다. 로버는 **키보드 > 자율 > 0** 순으로 중재한다
(키를 뗀 뒤 1 s 지나면 자율로 복귀, 자율 지령이 0.5 s 끊기면 정지).

`pose_relay` 와 `path_follower` 는 **launch 가 이미 띄운다** (261017 정정 — 이전 문서는 "따로 띄우라"고 돼 있었다):
- `daron_gui.launch.py` → `path_follower` (`follower:=true` 기본, 파라미터 `rover/server/config/follower.yaml`, 기동 때는 꺼진 상태)
- `daron_3dmap.launch.py` → `pose_relay` (`planner:=true` 일 때 `pose_relay:=true` 기본)

**따로 또 띄우지 말 것.** 같은 이름의 노드가 두 개 떠서 `/path_follower/enable` 하나에 둘 다 켜지고,
`/rover/auto/drive_rpm`·`steer_cmd` 를 서로 다른 설정으로 번갈아 낸다.

```bash
# 켜기 / 끄기 (제어창 '웨이포인트'·'자율' 버튼이 같은 토픽을 쓴다)
ros2 topic pub --once /path_follower/enable std_msgs/Bool "{data: true}"
ros2 topic pub --once /path_follower/enable std_msgs/Bool "{data: false}"    # 정지 (아무 키 입력도 즉시 수동)

# (실험용) launch 의 추종기를 끄고 직접 띄울 때 — daron_gui follower:=false pose_relay:=false 로 띄운 뒤
source ~/Daron/slam_env.sh
~/venv/daron3d/bin/python ~/daron-nav/daron_planner/ros/pose_relay.py
/usr/bin/python3 ~/Daron/daron_run/rover/server/path_follower.py --ros-args \
  --params-file ~/Daron/daron_run/rover/server/config/follower.yaml
```

⚠ 현재 값 (261017)
- 구동계: `follower.yaml` `wheel_diameter_m 0.108` · `gear_ratio 40.0` · `max_rpm 600.0`, 로버 `config/front.json`·`back.json` 도
  `gear_ratio 40` · `max_speed 600`. **두 곳 값을 같게 유지**할 것 (한쪽만 바꾸면 rpm 환산이 틀어진다).
- 감속비 40 에서 600 rpm 은 약 0.085 m/s 다 (모터 rpm ≈ 7074·v). 그래서 `v_max_mps` 는 0.08 (0.15 로 두면 늘 포화해 감속 로직이 안 듣는다).
- pose 신선도는 수신 시각 기준(`pose_stamp_source: receive`). chrony 로 지상국·RADXA 시계를 맞춘 뒤에는 `daron_gui follower_pose_stamp:=header`.

---

## 7. 주요 토픽

| 토픽 | 타입 | 보내는 쪽 → 받는 쪽 |
|---|---|---|
| `/radxa/camera/*` | Image·CameraInfo | RADXA realsense → `image_relay` → `/camera/camera/*` |
| `/camera/transport/depth_zstd` | CompressedImage | `depth_zstd_sender` → `video_receiver` → `/camera/camera/depth/image_rect_raw` |
| `/ekf/imu/data` | Imu | CV7 → imu_cov_relay → EKF |
| `/odometry/filtered_health`, TF `odom→base_link` | | EKF-C (최종 pose). `map→odom` 은 SLAM(루프 클로저) |
| `/traversability/grid` | OccupancyGrid | 판정 → planner_node |
| `/goal_pose` | PoseStamped | 제어창 → planner_node |
| `/planner/path` | Path (map) | planner_node → path_follower, RViz |
| `/localization/pose` | PoseStamped (map) | pose_relay → path_follower, 제어창(도착 기록) |
| `/cda/detections` | String (JSON) | YOLO → pin_node |
| `/rover/{drive_cmd,steer_cmd,articulation_cmd,inverted,command,heartbeat}` | | 제어창 '수동' / console.py → radxa.py |
| `/rover/auto/{drive_rpm,steer_cmd}`, `/rover/auto_enable` | Float64 / Bool | path_follower / console.py → radxa.py |
| `/rover/diagnostics`, `/rover/battery`, `/rover/odom`, `/joint_states` | | radxa.py → 제어창·console·path_follower |

---

## 8. 묶음에서 뺀 것

- `backup/`, `*.bak*`, `__pycache__`, `tests/`(`rover/server/tests/` 포함), CHANGES·원본 README 문서, 빈 `rover/logs/`(실행 시 자동 생성)
- 오프라인 도구 `tools/{slam_offline,offline_dense_*,dense_map_lib,health_offline}.py`
- `nodes/gui/crater_viewer.py`(GUI 에서 안 띄움), `crater_review.py`(세션 뒤 보기 도구)
- YOLO 벤치마크·결과·테스트, 설치·시험 스크립트(`setup_ros2_humble.sh`, `start_live.sh`, `run_bag_test.sh`, `check_receiver.sh` —
  `yolo/.../README.md` 에 나오지만 묶음에 없음), daron-nav 시뮬·평가·테스트
- 서스펜션 프로파일 `rover/profiles/rover.json` (RADXA 마다 다름)
- 추종기 사양서 `CLAUDE.md`, `follow_test_paths_261011/` (`rover/PATH_TRACKING.md` 가 가리키지만 묶음에 없음 — 원본 `cuVSLAM_and_GUI/`)

---

## 9. 검증 기록 (261011)

- 노드 19개를 실제 인터프리터(cuvslam/시스템/daron3d/cda)로 import — 로컬 모듈이 전부 묶음 안에서 풀림
- 도메인 87, `trav_obstacle` 재생 `rebuild:=true` + daron_gui(RViz 창 끔, 제어창 offscreen) + YOLO:
  cuVSLAM·EKF-A/B/C·피쳐맵·judge_core 판정·핀 3개·림·크레이터 계산·웨이포인트 → 제어창 → 플래너 연결,
  종료 시 피쳐맵·핀·Slam 맵·플래너 기록·waypoints.jsonl 저장
- 도메인 88, RADXA 흉내(bag 을 `/radxa/camera/*` 로 리맵 + 실제 `depth_zstd_sender`) + `source:=remote`:
  zstd depth 수신 29.7 Hz, image_relay IR 30 Hz, cuVSLAM 29.8 Hz, 13개 노드 정상 종료
- 로버 통합: `rover/` 전체 `py_compile`, ROS 없는 모듈 import, 묶음 전체 로컬 import 해석·토픽 송수신 대조
  (`/localization/pose` 는 pose_relay 추가로 해결)
- 확인 못 한 것: 실제 RADXA·WiFi, RViz 창 화면, 실카메라(local) 모드, 로버 실차 + 경로 추종 연동
