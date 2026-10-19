# SCNet CDA ROS 2 전달 패키지

이 폴더는 **학습 없이** 입력 ZIP에 포함된 SCNet 가중치로 infra1 영상을 검출하고, 유효한 aligned Depth로 거리 제한 없이 profiling을 시도한 뒤 `/cda/detections`를 발행하는 전달용 패키지입니다.

## 고정 입출력

- ROS 2 Humble, `ROS_DOMAIN_ID=42`, CycloneDDS
- infra1: `/camera/camera/infra1/image_rect_raw` (`mono8`, 848×480, 30 Hz)
- aligned Depth: `/camera/camera/depth/image_rect_raw` (`16UC1`, 1 mm/LSB)
- CameraInfo: `/camera/camera/infra1/camera_info`
- 출력: `/cda/detections` (`std_msgs/msg/String`)
- SCNet confidence: 0.25
- NMS IoU: 0.4
- Depth 0 또는 60000 이상: 무효
- 기본 intrinsic fallback: `fx=fy=431.94, cx=419.34, cy=228.48`

infra1과 Depth는 **header.stamp가 정확히 같은 프레임끼리만** 처리합니다. 추론이 30 Hz보다 느리면 큐를 누적하지 않고 최신의 완성된 pair만 남깁니다.

## Depth 정책

영상 4 진단에서 검출 bbox의 대표 거리는 대부분 3–10 m였고 중앙값은 약 7.7 m였습니다. 따라서 원거리 Depth로 FP를 제거하지 않습니다.

- sensor-valid Depth는 거리 정보와 주변 지면 보정·10-strip profile 계산에 사용. 기본 0.07 m 이상이며 별도 m 단위 상한 없음.
- **1.0 m 밖에서도 profile 계산**. 원거리에서 계산한 결과가 부정적이거나 불확실해도 기본 정책은 후보 유지.
- 1.0 m는 계산 상한이 아니라 기본 후보 제외 권한의 신뢰 범위임.
- `depth_profile_computed`는 프로파일 계산 완료 여부, `depth_filter_applied`는 실제 제외 권한을 가진 필터 적용 여부임.
- 지면·유효점·중앙 Depth가 부족하면 계산 실패 사유와 함께 후보 유지.
- metric depth에서는 함몰부가 주변 예상 지면보다 **더 먼 거리(positive residual)**가 되는 것을 기본 polarity로 사용

이 정책은 원거리에서도 계산을 시도하면서 불확실한 Depth로 탐지 후보가 사라지는 것을 방지합니다.

### 센서·거리 설정

`start_live.sh` 환경변수(동일한 소문자 ROS 파라미터도 제공):

- `MAX_DEPTH_M=0.0`: m 단위 상한 없음. 센서에 맞는 상한을 원하면 양수 지정.
- `MIN_DEPTH_M=0.07`, `DEPTH_SCALE=0.001`, `INVALID_RAW_MIN=60000`: 원시 단위와 무효값 설정. `INVALID_RAW_MIN=0`이면 0만 무효 처리.
- `PROFILE_LONG_RANGE=true`: 원거리도 계산. false는 이전의 근거리 분석 정책.
- `RELIABLE_DEPTH_MAX_M=1.0`: 기본 후보 제외 권한의 거리 기준.
- `ALLOW_LONG_RANGE_REJECTION=false`: 원거리 미확인 후보를 유지. 센서 검증 후에만 true로 변경.
- `DEPTH_ERROR_COEFFICIENT=0.012`: 기존 sigma(Z)=계수×Z² 모델. 센서별 보정값으로 설정 가능하며 원거리 정확도를 보증하는 수치가 아님.

예: `MAX_DEPTH_M=5.0 ./start_live.sh`. 기본 실행만으로도 1 m 밖의 분석을 시도함.

## 설치

```bash
cd /home/Daron/yolo/scnet_cda_ros2_handoff_20261002_depth3d
chmod +x setup_ros2_humble.sh start_live.sh run_bag_test.sh check_receiver.sh
./setup_ros2_humble.sh
```

설치 마지막에 다음이 확인되어야 합니다.

```text
rclpy/cv_bridge import: PASS
CUDA: True                 # RTX 3060 사용 시
verify_package: passed
PASS: near-range profile ...
model smoke test: passed
```

`CUDA: False`면 실시간 성능을 확인하기 전에 NVIDIA driver/PyTorch CUDA 환경을 먼저 해결하십시오.

## rosbag 종단 시험

```bash
./run_bag_test.sh /path/to/rosbag
```

생성 파일:

```text
results/bag_test/detections.jsonl
results/bag_test/summary.json
```

JSON schema, timestamp 고유성/순서를 자동 검사합니다. GPU가 입력 30 Hz를 따라가지 못하면 입력 프레임보다 발행 프레임 수가 적은 것이 정상입니다.

## 실시간 송신

```bash
./start_live.sh
```

동일 PC에서 빠르게 확인:

```bash
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=42
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ros2 topic info /cda/detections -v
ros2 topic echo /cda/detections std_msgs/msg/String --once
ros2 topic hz /cda/detections
```

## 반대쪽 PC에서 수신 확인

수신 PC도 ROS 2 Humble + CycloneDDS를 사용하고 같은 네트워크에 있어야 합니다. 패키지를 수신 PC에도 복사했다면:

```bash
export ROS_DOMAIN_ID=42
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
./check_receiver.sh
```

패키지를 복사하지 않았다면 최소 확인:

```bash
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=42
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ros2 topic list | grep /cda/detections
ros2 topic info /cda/detections -v
ros2 topic echo /cda/detections std_msgs/msg/String --once
```

`topic list`에도 보이지 않으면 다음을 확인합니다.

1. 양쪽 `ROS_DOMAIN_ID=42`
2. 양쪽 `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`
3. 같은 LAN/서브넷 및 multicast 허용
4. Ubuntu firewall 또는 기관 네트워크가 DDS discovery를 차단하지 않는지
5. 필요 시 양쪽에서 `ros2 daemon stop && ros2 daemon start`

## 지도 모듈 연결

지도 모듈의 필수 필드는 `stamp_ns`, `dets[].box`, `dets[].score`, `dets[].class`입니다.

Depth 계산 여부는 **`depth_profile_computed`**, 제외 권한을 가진 필터 적용 여부는 **`depth_filter_applied`**로 구분합니다. `depth_profile_pass=true`만으로 관측이 충분하다고 해석하지 않습니다.

세부 JSON 형식은 `OUTPUT_CONTRACT.md`를 참고하십시오.

## 권장 최종 smoke test

1. 송신 PC에서 camera/rosbag + `./start_live.sh` 실행
2. 수신 PC에서 `./check_receiver.sh` PASS 확인
3. 지도 핀 모듈 실행 후 `ros2 topic info /cda/detections -v`에서 subscriber 확인
4. `dets:[]` 프레임 처리 확인
5. crater 1개 검출 시 핀 생성 확인
6. 연속 관측 시 같은 crater가 중복 핀으로 폭증하지 않는지 확인

## Depth profiling 구현 세부사항 (강화 버전)

다음 계산은 원거리에서도 수행합니다. 기본 정책에서는 근거리(`<=1.0 m`)에서만 Depth가 실제 reject 권한을 갖습니다.

1. bbox 주변 ROI를 확장하고 bbox+margin을 제외한 주변 지면 픽셀을 선택
2. CameraInfo의 `fx, fy, cx, cy`로 주변 depth를 3-D point cloud로 역투영
3. 결정적 RANSAC 후보 사전평가 + 전체 점 검증 + 공분산 고유분해로 local ground plane 추정
4. plane inlier ratio와 RMSE가 기준을 통과할 때만 필터 활성화
5. bbox 각 픽셀 ray와 ground plane의 교점에서 예상 ground depth를 계산
6. `measured depth - expected ground depth`를 함몰 residual로 사용
7. residual이 센서 오차를 고려한 동적 threshold를 넘는 영역의 비율/연결성/중앙 깊이/10-strip bowl profile을 평가
8. 명확한 함몰이면 `confirmed_crater`; 제외 권한이 있으면 평탄한 후보를 `rejected_flat`로 처리. 원거리는 `advisory_pass`로 보존.

중요하게, **근거리 Depth가 신뢰 가능한 상황에서는 SCNet confidence가 geometry 판정을 우회하지 않습니다.** 반대로 원거리/Depth hole/ground-plane 품질 부족 상황에서는 Depth가 후보를 제거하지 않습니다.


## 2026-10-11 검증 수정본

- CUDA 기본값: FP16 + 고정 입력 크기 CUDA Graph. CPU는 FP32 사용.
- 입력 크기가 바뀌거나 CUDA Graph 적용이 불가능하면 일반 추론으로 복귀.
- 가중치, 864 입력 크기, confidence 0.25, NMS IoU 0.4, 10구간 Depth 정책 유지.
- 기존처럼 `./start_live.sh` 실행. 비교 실행은 `HALF=false CUDA_GRAPH=false ./start_live.sh`.
- 5초마다 `output_fps`, 처리시간 median/p95, `pending_drops`, `errors` 로그 출력.
- output_fps는 실제 발행률이며 처리시간 역수나 카메라 입력 FPS와 다름.
- `python verify_package.py`로 현재 코드·가중치 해시 검사 가능.
- RealOutput5 원본 rosbag의 mono8/16UC1 입력으로 Windows CUDA 계산 코어 재시험.
- Linux ROS2 통신과 현장 장치의 발행률은 별도 현장 확인 필요.
- 예전 benchmark_results와 실행 캐시는 이번 전달본에서 제외. 원래 실행 파일·하위 경로 유지.
