# DARON 경로 추종 — Stanley 계열 곡률 제어 (261011)

지상국이 경로(`/planner/path`)와 로버 위치(`/localization/pose`)를 받습니다. 50 Hz마다 곡률 명령을 계산해서
로버에 **모터 rpm**과 **차량 유효조향각**을 보냅니다. 로버(`radxa.py`)는 이 명령으로 구동하고 조향하며,
휠 오도메트리와 휠베이스를 돌려줍니다.

- 사양서: `cuVSLAM_and_GUI/CLAUDE.md`
- 이번 변경 요약: `daron_run/CHANGES_261011.md`
- 이 문서의 기준 위치는 **`daron_run/rover/`**입니다. 예전 `DARON_auto/`는 더 이상 쓰지 않습니다(§10).

---

## 1. 무엇이 어디에 들어가는가

### 1.1 배치 그림

```
 ┌────────────────────────── 지상국 PC ──────────────────────────┐            ┌──────────── RADXA (로버) ────────────┐
 │                                                              │            │                                      │
 │  cuVSLAM → health_monitor → EKF-C ─ TF map→base_link         │            │  rover/radxa.py  (노드 /rover)        │
 │  pose_relay (daron-nav) ─► /localization/pose  [m, map]      │            │   ├─ 지령 중재  키보드 > 자율 > 0      │
 │  planner_node (daron-nav) ─► /planner/path     [m, map]      │            │   ├─ L12 x2 조향 (PWM8 / PWM9)         │
 │                                   │                          │            │   ├─ VESC x2 구동 (rpm 지령)           │
 │                                   ▼                          │            │   ├─ XM430 굴절·서스펜션·형상 모드     │
 │  rover/server/path_follower.py  (노드 /path_follower, 50 Hz) │            │   └─ /rover/odom, /rover/diagnostics,  │
 │   ├─ path_cte.py      경로 전처리 + CTE                      │  WiFi/DDS  │      /rover/battery                  │
 │   ├─ stanley_core.py  제어법칙·포화·적분·rate 제한            │ ─────────► │  /rover/auto/drive_rpm               │
 │   ├─ follower_core.py 한 주기 순서·상태 머신·속도             │ ─────────► │  /rover/auto/steer_cmd               │
 │   └─ path_tracking.py PoseEstimator, rpm 변환 상수            │ ◄───────── │  /rover/odom                         │
 │                                                              │ ◄───────── │  /rover/diagnostics (wheelbase_mm)   │
 │  rover/console.py (P 키) ─► /rover/auto_enable ─────────────────────────► │  자율 허용 / 일시정지                  │
 └──────────────────────────────────────────────────────────────┘            └──────────────────────────────────────┘
```

**추종기는 지상국에서 돕니다.** 경로와 pose가 둘 다 지상국에서 만들어지므로, WiFi를 건너는 것은
50 Hz 명령 두 개(Float64)와 odom·diagnostics뿐입니다.

> CLAUDE.md §2.3과 §10-5는 추종기가 로버에서 도는 것처럼 적혀 있습니다. 나중에 pose를 onboard에서 보간하기로 하면
> `server/` 폴더를 RADXA로 가져가면 됩니다. 코드는 어느 쪽에서도 같습니다(순수 Python + rclpy).

### 1.2 지상국에서 쓰는 것 (`daron_run/rover/` 기준)

| 파일 | 역할 |
|---|---|
| `server/path_follower.py` | ROS 노드. 토픽 입출력, 파라미터, 로그만 담당합니다 |
| `server/follower_core.py` | 제어 한 주기(§3), 상태 머신, 속도 rate 제한, 진단 배열. ROS 없음 |
| `server/stanley_core.py` | 제어법칙, 포화, 조건부 적분, rate 제한, κ→δ, 속도 명령(CLAUDE.md §6 참조 구현) |
| `server/path_cte.py` | 경로 전처리와 CTE 함수(window 탐색, 세그먼트 투영, preview 곡률). ROS 없음 |
| `server/path_tracking.py` | `PoseEstimator`(pose를 odom으로 보간), `TrackerConfig`(rpm 변환), `SimRover` |
| `server/config/follower.yaml` | 파라미터 |
| `motor_control/l12/driver.py` (`server/`의 상위 폴더) | `path_tracking.py`가 `MAX_STEER_DEG`와 `wheelbase_mm()`을 import합니다. 순수 계산 부분만 쓰고 하드웨어 접근은 없습니다 |
| `console.py` | 키보드 조종(선택). P 키로 `/rover/auto_enable` 전환, MODE/AUTO 배지 |
| 시험용: `server/sim_rover.py`, `server/send_path.py`, `server/sim_offline.py`, `server/tests/` | 시뮬레이션과 테스트 |
| `follow_test_paths_261011/send_path_csv.py` | 경로계획 CSV 4개를 발행 |
| (다른 저장소) `planner_node.py`, `pose_relay.py`(daron-nav), EKF-C | 경로와 pose를 만드는 쪽. `pose_relay`는 launch에 없어서 따로 띄워야 합니다 |

### 1.3 RADXA에서 쓰는 것

`launch/daron_rover.launch.py`가 `rover/radxa.py`를 띄웁니다.

| 파일 | 역할 |
|---|---|
| `radxa.py` | `/rover` 노드. **261010 판(배터리, 굴절각 부호, PWM8/PWM9) + 자율 수신부(261011)** |
| `motor_control/l12/driver.py`, `motor_control/vesc/xerun.py`, `motor_control/xm430/driver.py`, `motor_control/xm430/link.py`, `susp_*.py`, `motor_control/xm430/mock.py` | 하드웨어 드라이버와 서스펜션 |
| `config/front.json`, `config/back.json`, `profiles/rover.json` | VESC 설정, 캘리브레이션 프로파일 |
| `cyclonedds.xml` | DDS 설정(양쪽 공통) |

`server/` 폴더는 RADXA에 없어도 됩니다. 있어도 import되지 않습니다.

### 1.4 토픽 인터페이스

| 토픽 | 타입 | 발행 → 구독 | 내용 |
|---|---|---|---|
| `/planner/path` | `nav_msgs/Path` | planner → follower | `map` 좌표계, **미터**. 점 간격 약 5 cm, 축 가운데 기준, 전진만. 빈 경로 = 정지 |
| `/localization/pose` | `geometry_msgs/PoseStamped` | pose_relay → follower | `map` 좌표계, **미터**, 축 가운데 기준, 20 Hz |
| `/rover/odom` | `nav_msgs/Odometry` | radxa → follower | 휠 속도(`twist.linear.x`). pose 사이 보간에 씁니다. **`gear_ratio`가 있어야 나옵니다(§9-2)** |
| `/rover/diagnostics` | `DiagnosticArray` | radxa → follower | `wheelbase_mm`. 굴절각에 따라 317~342 mm |
| `/path_follower/enable` | `std_msgs/Bool` | 운용자 → follower | 추종 시작과 정지 |
| `/rover/auto_enable` | `std_msgs/Bool` | console(P 키) → radxa | 로버 쪽 자율 허용(기본 true) |
| `/rover/auto/drive_rpm` | `std_msgs/Float64` | follower → radxa | 모터 rpm, 0 이상(전진만). radxa가 VESC `max_speed`(400)로 자릅니다 |
| `/rover/auto/steer_cmd` | `std_msgs/Float64` | follower → radxa | 차량 유효조향각 [deg], +는 좌회전 |
| `/follower/debug` | `Float32MultiArray` | follower → 기록 | §5 순서 |
| `/path_follower/status` | `DiagnosticArray` | follower → GUI | 상태와 오차, 10 Hz |

자율 주행이 되려면 네 조건이 모두 필요합니다.

1. follower가 enable 상태
2. radxa의 `auto_enable`이 true
3. 키보드 입력이 없음
4. 형상 전환 중이나 수납 상태가 아님

### 1.5 radxa.py의 수동/자율 중재

| 조건 | 모드 | 구동 | 조향 |
|---|---|---|---|
| 0이 아닌 키 입력 후 `manual_override_hold_s`(1 s) 안 | `manual` | 키 방향 | 키 조향 |
| 그 밖에 `auto_enable`이고 자율 명령이 `auto_timeout`(0.5 s) 안에 들어옴 | `auto` | rpm → VESC `set_speed` | 자율 조향각 |
| 그 밖에 콘솔 링크가 살아 있음 | `manual` | 0 | 0 |
| 아무것도 없음 | `idle` | 0 | 중립 |

- 형상 전환 중이나 수납 상태(`drivable=False`)면 수동·자율 모두 구동 0, 조향 중립입니다.
- 전복 상태면 rpm 부호를 뒤집습니다.

---

## 2. 입력 좌표 — 픽셀이 아니라 map 미터

`follow_test_paths_261011/README.md` 기준입니다.

- **경로와 pose 모두 `map` 좌표계, 미터 단위입니다.** +x 앞, +y 왼쪽, yaw는 반시계가 +입니다. 기준점은 앞·뒤 축의 가운데(base_link 뒷축에서 0.17 m 앞)입니다.
- 추종기에서 픽셀 변환은 없앴습니다. `-p path_in_pixels:=false`를 넘겨도 선언되지 않은 파라미터라서 무시됩니다.
- 경로의 `frame_id`가 `map_frame`(기본 `map`)과 다르면 그 경로를 버리고 기존 경로를 유지합니다. 빈 경로는 frame과 상관없이 정지로 처리합니다.
- 경로의 orientation(yaw)은 쓰지 않습니다. 접선과 곡률은 x, y로 다시 계산합니다.
- `sim_rover.py`와 `send_path.py`도 `map` 미터로 발행합니다. `follow_test_paths_261011/README.md`의 "pose_in_pixels 기본값(true) 그대로"라는 안내는 이제 해당하지 않습니다.

---

## 3. 제어 한 주기 (50 Hz 타이머, `follower_core.Follower.step`)

콜백은 데이터를 저장만 합니다. 계산은 전부 타이머에서 합니다.

1. **경로 교체.** 콜백에서 검증하고 전처리한 `pending` 경로를 주기 시작 시점에 `active`로 바꿉니다. 이때 `path_id`를 1 올립니다.
   `kappa_prev`와 `I`는 **리셋하지 않습니다**(bumpless). 최근접점은 새 경로의 **앞쪽 0.5 m 안에서만** 찾습니다.
2. **enable, 경로 유무, pose staleness를 판정합니다.** pose staleness는 `now − header.stamp > pose_timeout_s`로 판단합니다.
3. **CTE를 계산합니다**(`path_cte.cte`). 결과는 `e_y`(왼쪽 +), `e_th = wrap(ψ − 접선)`, `κ_ref`(preview 0.05 m 앞), `remaining`입니다.
   - 최근접점: 직전 세그먼트 기준 뒤로 1개, 앞으로 8개 세그먼트에 투영합니다. 첫 세그먼트와 마지막 세그먼트는 바깥으로 연장해서 투영합니다.
   - 곡률: `k[i] = (th[i] − th[i−1]) / ((L[i−1]+L[i])/2)`에 3점 이동평균을 적용합니다.
   - 접선각: 꼭짓점에서는 양옆 세그먼트 각의 평균을 쓰고, 세그먼트 안에서는 선형 보간합니다.
4. **제어법칙을 적용합니다.**
   ```
   k_th  = k_th_ratio · k_e
   th_d  = −atan(k_e · e_y)
   κ_raw = κ_ref + k_th · wrap(th_d − e_th) − I
   ```
5. **포화 → 조건부 적분 → rate 제한 순서로 처리합니다.**
   - 포화: `|κ| ≤ kappa_max`
   - 적분: 포화 중이 아니고 `|e_y| < e_I_max`일 때만 `I += k_i·e_y·v·dt`
   - rate 제한: `|Δκ| ≤ kappa_rate·dt`
6. **κ를 δ로 변환합니다.** `δ = atan(L·κ)`이고, L은 `/rover/diagnostics`의 `wheelbase_mm` 값입니다(l12_driver의 tan Δv = L·κ 관계). 결과는 `|δ| ≤ 43°`로 clamp합니다.
7. **속도를 계산합니다.**
   ```
   v = v_max_mps · min(f_cte, f_curv)
   ```
   `a_max`로 rate 제한을 걸고, rpm은 `60·gear/(π·D)` 변환과 `min_rpm`/`max_rpm`으로 만듭니다.
8. **안전 로직을 적용합니다.** 정지 조건이면 v = 0을 **같은 주기에 바로** 내보냅니다(rate 제한 없음). 조향은 마지막 명령을 유지합니다.

### 상태 머신

| 코드 | 상태 | 진입 | 동작 | 해제 |
|---|---|---|---|---|
| 0 | `IDLE` | enable off, 또는 경로가 아직 없음 | v = 0 | enable on이고 경로가 있음 |
| 1 | `TRACKING` | 정상 | §3 | — |
| 2 | `STOP_GOAL` | 빈 경로 수신 | v = 0, I 리셋 | 점이 있는 새 경로 |
| 3 | `STOP_END` | `remaining < end_tol` | v = 0 | 새 경로 |
| 4 | `STOP_STALE` | pose 나이 > `pose_timeout_s` | v = 0, 적분 중단 | 새 pose(I는 유지) |
| 5 | `HOLD_DEVIATION` | `|e_y| > e_abort` | v = 0, 조향 유지 | 새 경로 |

- **I 리셋:** 빈 경로를 받았을 때, enable이 꺼졌을 때, 정지 상태(`IDLE`, `STOP_GOAL`, `STOP_END`, `HOLD_DEVIATION`)에서 다시 출발할 때.
- **`kappa_prev`를 0으로 돌리는 때:** enable off일 때만입니다. 이때 노드는 0을 한 번 보내고 발행을 멈추고, radxa는 0.5 s 뒤 조향을 중립으로 돌립니다.
- **정지 상태에서도 명령은 계속 50 Hz로 발행합니다**(v = 0, 조향 유지). 그래서 radxa의 `auto_timeout`이 걸리지 않고 조향이 유지됩니다.
- **경로 나이 기반 타임아웃은 없습니다.** 같은 경로는 다시 보내지 않기 때문입니다.

---

## 4. 파라미터 (`server/config/follower.yaml`)

### 4.1 조향과 속도 (CLAUDE.md §5)

| 이름 | 기본값 | 설명 |
|---|---|---|
| `k_e` | 1.5 | 주 게인 [1/m]. 수렴 길이는 약 1/(2·k_e) |
| `k_th_ratio` | 4.0 | 4이면 임계감쇠. 진동하면 5~6으로 올립니다 |
| `k_i`, `I_max`, `e_I_max` | 0.0, 0.5, 0.10 | 적분. 기본값은 꺼짐 |
| `gain_scale` | 1.0 | Fuzzy GS 자리. 1.0 고정 |
| `kappa_max`, `delta_max_deg` | 2.742, 43.0 | 조향 한계 |
| `kappa_rate` | 5.5 | **TODO 실측.** `2·kappa_max / T_full_swing` |
| `understeer_gain` | 1.0 | [261015] 언더스티어 보정 = 조향 지령 곡률 / 실제 곡률 = `1 / backup.turn_ratio`(planner status). 1.0이면 끔. 조향을 이 배수만큼 더 꺾고, 추종기의 곡률 한계·변화율은 그만큼 작게 봅니다 |
| `preview_m` | 0.05 | 곡률 feedforward 미리보기 거리 |
| `v_max_mps` | 0.15 | = v_nom |
| `v_min`, `e1`, `e2` | 0.05, 0.05, 0.15 | CTE 기반 감속 |
| `curv_slow_ratio`, `curv_slow_factor` | 0.8, 0.6 | 곡률 기반 감속 |
| `a_max` | 0.3 | 속도 rate 제한 [m/s²] |
| `end_tol`, `e_abort` | 0.02, 0.30 | 경로 끝 판정, 이탈 판정 [m] |
| `pose_timeout_s` | 0.3 | pose staleness 판정 시간 [s]. §9-1 확인 필요 |
| `new_path_search_m` | 0.5 | 새 경로 최근접점 탐색 범위 [m] |

### 4.2 노드

| 이름 | 기본값 | 설명 |
|---|---|---|
| `control_hz` | 50.0 | 제어 주기 [Hz] |
| `map_frame` | `map` | 경로 `frame_id` 검증값 |
| `pose_stamp_source` | `receive` | `header`는 chrony 시간 동기화가 전제입니다. 동기화가 안 돼 있으면 `receive` (261012 기본값 변경, §9-1) |
| `use_odom_propagation` | true | pose 사이를 odom으로 보간합니다 |
| `odom_timeout_s` | 0.5 | odom이 이보다 오래되면 속도 피드백과 보간을 쓰지 않습니다 |
| `start_enabled` | false | 기동하자마자 추종할지 |
| `allow_reverse` | false | 전진만 합니다 |
| `wheel_diameter_m`, `gear_ratio` | 0, 0 | **0이면 구동하지 않습니다.** VESC config와 같은 값을 넣습니다(D = 0.108, G = TODO) |
| `min_rpm`, `max_rpm` | 0, 500 | 모터 rpm 범위. `max_rpm`은 VESC `max_speed`(400) 이하로 둡니다 |
| `log_dir` | `logs` | CSV 로그 위치(`server/logs/`) |

모든 추종 파라미터는 런타임에 바꿀 수 있습니다(예: `ros2 param set /path_follower k_e 2.0`).

### 4.3 radxa.py (자율 관련)

| 이름 | 기본값 | 설명 |
|---|---|---|
| `auto_timeout` | 0.5 | 자율 명령이 이만큼 끊기면 정지 [s] |
| `manual_override_hold_s` | 1.0 | 키를 뗀 뒤 수동을 유지하는 시간 [s] |
| `auto_enable_default` | true | 콘솔이 `auto_enable`을 보내기 전의 값 |
| `odom_enable`, `odom_frame`, `base_frame` | true, `odom`, `base_link` | 휠 odom 발행 |

launch에서 넘길 때는 `rover_params:='auto_timeout:=0.5 manual_override_hold_s:=1.0'` 형식으로 줍니다.

---

## 5. 진단 `/follower/debug` (`Float32MultiArray`)

```
[0] t  [1] state  [2] path_id  [3] e_y  [4] e_th  [5] kappa_ref  [6] kappa_fb
[7] kappa_raw  [8] kappa_cmd  [9] delta_cmd  [10] saturated  [11] I  [12] v_cmd
[13] v_meas  [14] remaining  [15] pose_age
```

- `t`는 노드 시작 후 경과 시간 [s]입니다. float32로는 epoch 초를 담을 수 없어서입니다.
- `state`는 §3 상태 표의 코드입니다.
- `delta_cmd` 단위는 rad입니다.
- 비활성 상태에서도 발행합니다.
- 같은 내용이 `server/logs/follow_*.csv`에도 기록됩니다.

로버 쪽에서는 다음을 함께 보면 됩니다.
- `/rover/diagnostics`의 `drive_mode`, `drive_applied`(`rpm:250` 형식), `auto_rpm`, `auto_steer_deg`, `odom_*`
- console drive CSV의 맨 끝 8열

---

## 6. 실행

### 6.1 하드웨어 없이, ROS 없이

`daron_run/rover/server/`에서 실행합니다.

```bash
python3 -m pytest tests -q                       # 24개 (pytest가 없으면 python3 tests/test_*.py)
python3 sim_offline.py <follow_test_paths_261011>/path_*.csv --swap
python3 sim_offline.py <follow_test_paths_261011>/path_*.csv --noise 0.01 --latency 0.1
```

### 6.2 하드웨어 없이, ROS 시뮬레이션 (CLAUDE.md §7.2)

`daron_run/rover/`에서 실행합니다. `sim_rover`는 `radxa.py` 대신 띄우는 시뮬레이터입니다.

```bash
python3 server/sim_rover.py     --ros-args -p wheel_diameter_m:=0.1 -p gear_ratio:=10.0
python3 server/path_follower.py --ros-args --params-file server/config/follower.yaml \
        -p wheel_diameter_m:=0.1 -p gear_ratio:=10.0 -p min_rpm:=100.0 -p max_rpm:=2000.0 \
        -p start_enabled:=true
python3 <follow_test_paths_261011>/send_path_csv.py <follow_test_paths_261011>/path_detour.csv
ros2 bag record /follower/debug /planner/path /localization/pose
```

### 6.3 실차

**RADXA**에서 로버 노드를 띄웁니다.

```bash
ros2 launch <daron_run>/launch/daron_rover.launch.py ros_domain_id:=42 susp_profile:=<rover.json>
```

**지상국 (261012~)**: `daron_gui.launch.py`가 planner·pose_relay와 함께 추종기를 띄웁니다(`follower:=true` 기본, 꺼진 채 기동).
제어창 '로버 제어' 칸의 **웨이포인트** 버튼이 `/path_follower/enable`을 켜고 끕니다('자율' 버튼은 planner AUTO 모드 자리로 미연결). '추종기' 칸에 상태·횡오차·남은 거리·rpm,
'지령 출처' 칸에 로버가 실제로 따르는 지령(`drive_mode`)이 나옵니다. 창을 닫으면 추종기를 끕니다.

따로 띄울 때(`follower:=false`)는 EKF-C, `pose_relay`, `planner_node`가 떠 있는 상태에서 추종기를 띄웁니다.

```bash
python3 <daron_run>/rover/server/path_follower.py --ros-args \
        --params-file <daron_run>/rover/server/config/follower.yaml \
        -p wheel_diameter_m:=0.108 -p gear_ratio:=<실측> -p min_rpm:=<실측> -p max_rpm:=400.0
ros2 topic pub --once /path_follower/enable std_msgs/Bool "{data: true}"
```

- 정지: `ros2 topic pub --once /path_follower/enable std_msgs/Bool "{data: false}"`, 또는 콘솔 P 키, 또는 아무 키 입력(즉시 수동으로 넘어감).
- 지상국 GUI의 '수동' 제어창(`gui/rover_teleop.py`)은 `/rover/auto_enable`을 보내지 않습니다. 그래서 radxa 기본값(true)을 따르고, 키를 누르는 동안만 수동이 됩니다.

---

## 7. 검증 결과 (261011)

### 7.1 추종기 (`server/`, 새 위치에서 다시 실행)

**단위 테스트:** 24개 모두 통과했습니다.

| 직선 + 초기 오프셋 0.1 m (k_e 1.5, 0.15 m/s) | 1.0 m 지점 | 1.2 m 지점 | overshoot |
|---|---|---|---|
| 사양서 검증값 | 2.0 cm | 1.3 cm | 0 |
| 이 구현 | 2.0 cm | 1.3 cm | 0 |

**오프라인 폐루프 시뮬레이션:**
- 로버 모델: `SimRover`(조향 지연 0.15 s, 최대 120°/s)
- pose 10 Hz + odom 보간, 잡음 없음
- 계산 기준: 0.3 m 이후 최대 |e_y|

| 경로 | 결과 | 끝점 오차 | 최대 \|e_y\| | 포화 비율 |
|---|---|---|---|---|
| left | 끝점 도달 | 2.0 cm | 2.2 cm | 34 % |
| right | 끝점 도달 | 1.5 cm | 2.4 cm | 20 % |
| uturn | 끝점 도달 | 1.2 cm | 2.2 cm | 44 % |
| detour | 끝점 도달 | 1.2 cm | **3.0 cm** | 29 % |

- **pose 잡음 1 cm / 1°, 지연 0.1 s를 넣으면** 최대 3.6~4.9 cm, p95 3.1~3.6 cm입니다.
- **경로 교체:** 네 조합 모두 `path_id`가 2로 올라가고, 첫 투영점이 s = 2.7~4.1 cm(앞쪽)에서 잡히고, 교체 직후 |dκ/dt|가 5.5 이하입니다.
- **빈 경로:** `STOP_GOAL`로 정지합니다.
- **detour의 3.0 cm는 수용 기준(< 3 cm)의 경계입니다.** 원인은 `kappa_rate` 제한입니다.
  - 이상적 조건에서는 0.95~1.43 cm로, 사양서 참조 결과(1.2~1.6 cm)와 맞습니다.
  - `kappa_rate`를 11로 올리면 detour가 2.5 cm, `preview_m`을 0으로 두면 2.1 cm였습니다.

### 7.2 병합된 radxa.py (rclpy stub + `susp_mock` + 가짜 VESC 두 대)

| 시험 | 결과 |
|---|---|
| 기동 | 형상 NEUTRAL 이동(180° → 150°) 후 `drivable=True` |
| 자율 명령(250 rpm, −20°) | `auto`, `('rpm', 250.0)`, 조향 −20° |
| 1000 rpm 명령 | `max_speed` 400으로 clamp |
| 키보드 W 입력 | 즉시 `manual`, `('dir', 1)` |
| `auto_enable=false` | 자율 무시, `('dir', 0)` |
| 전복 + 자율 | `('rpm', -250.0)` |
| 자율 0.7 s 끊김 | `idle`, 구동 0, 조향 0 |
| `drivable=False` + 자율 | 구동 `rpm:0`, 조향 0 |
| odom / 배터리 / 진단 | `/rover/odom`(odom → base_link, v 0.15), `battery_pct` 80, `wheelbase_mm`, `articulation_deg`(180 + θ 부호) |

### 7.3 아직 하지 못한 것

- 실제 ROS 실행(§6.2)과 실차 시험은 하지 못했습니다. 이 Mac에 ROS가 없습니다.
- `console.py`는 import와 CSV 헤더까지만 확인했습니다. pygame 화면은 띄워 보지 못했습니다.

---

## 8. 사양서(CLAUDE.md)와 다르게 정한 것

| 항목 | 결정 | 이유 |
|---|---|---|
| 파라미터 이름 | `control_hz`, `pose_timeout_s`, `v_max_mps`(= v_nom)를 유지 | 인터페이스는 기존 코드를 따름(§0-5) |
| κ→δ 변환 | `atan(L·κ)`, L은 `wheelbase_mm` | 기존 kinematics. 굴절각 150°에서 L = 0.331 m(사양서 L_eff = 0.340 m). 이 경우 κ 2.742는 42.2°에 해당 |
| CTE 접선각 | 꼭짓점 평균 + 선형 보간 | 세그먼트 각을 그대로 쓰면 5 cm마다 최대 7.8°씩 계단이 생김 |
| 빈 경로 frame 검사 | 하지 않음 | 정지하는 쪽이 안전함 |
| 속도 PI(odom) | 뺌. rpm은 피드포워드만 | §4.6에 없음. VESC가 rpm 폐루프 제어를 함 |
| 픽셀 입력 | 없앰 | 입력은 map 미터 |
| debug `t` | 노드 시작 후 경과 시간 | float32 해상도 때문 |

---

## 9. 미결 사항과 확인할 것

1. **pose stamp와 `pose_timeout_s` 0.3 s — 261012 확인, `pose_stamp_source: receive`로 바꿈.**
   - pose_relay가 header에 넣는 stamp는 카메라 영상 시각이 아닙니다. `lookup(map, base_link)`에서 map→odom은 static TF라
     시각을 제한하지 않고, odom→base_link(EKF-C)의 stamp는 robot_localization의 **마지막 측정 시각 = IMU 시각**입니다.
     녹화 bag 실측(test_celab, tilt20_2): TF 나이 p50 7~8 ms, p95 12~14 ms (같은 bag의 VO 나이 p50 12~13 ms).
     원격(WiFi)에서도 IMU 지연(18~37 ms) 수준이라 **350~390 ms(영상 지연)가 아닙니다.** 0.3 s 로 충분합니다.
   - 그래도 `receive`로 둔 이유: RADXA·지상국 시계 동기(chrony)가 확인되지 않았습니다(DARON_261011 로그 §5).
     `header`는 시계 차이가 그대로 나이에 더해집니다. pose 보간(`PoseEstimator`)은 원래 수신 시각 기준이라 제어 결과는 같고,
     pose 자체가 오래된 경우는 pose_relay가 TF 나이 0.5 s 초과 때 발행을 멈춰서 막습니다.
   - chrony를 맞춘 뒤에는 `follower_pose_stamp:=header`(daron_gui.launch.py)로 바꿔도 됩니다.
2. **VESC `gear_ratio` 40 — 261016 입력, 261017 갱신** (`config/front.json`·`back.json`·`follower.yaml` 모두 40).
   - 필요 모터 rpm = 60·G·v/(π·D) ≈ 176.8·G·v → G 40 에서 7074·v (D 0.108 m).
   - `max_speed`·`max_rpm` 600 rpm 의 상한 속도는 약 0.085 m/s. 그래서 `v_max_mps` 를 0.15 → **0.08** 로 내렸다.
     0.15 로 두면 늘 rpm 포화라 곡률 감속(×0.6)·CTE 감속이 전혀 듣지 않는다.
   - 감속비·rpm 상한을 바꾸면 두 config 를 같이 바꾸고 `v_max_mps` 를 다시 계산할 것.
3. **chrony 시간 동기화** 여부. 이에 따라 `pose_stamp_source`를 정합니다.
4. **실측값:** L12 full swing 시간(`kappa_rate`), `min_rpm`(모터가 실제로 도는 최저 rpm), 실제 R_min(CLAUDE.md §7.3-2).
5. **운용 굴절각.** L과 κ_max 대조(CLAUDE.md §10-6)에 필요합니다.
6. **경로계획 팀 요청:** CLAUDE.md §10의 1~4.
7. **지상국 GUI 제어창(`gui/rover_teleop.py`)에는 자율 표시가 없습니다.** MODE와 자율 rpm은 `console.py`나 `/rover/diagnostics`로 확인합니다.

---

## 10. 예전 위치와 백업

- **`DARON_auto/`**: 9/30 자율판입니다. 그 `radxa.py`, `console.py`, `motor_control/l12/driver.py`, `config/`는 형상 모드·배터리·부호·PWM 수정 이전 판이라 **쓰지 않습니다.** `DARON_auto/server/`는 이 폴더 `server/`와 같은 코드입니다(앞으로는 이쪽만 고칩니다).
- **백업:** `daron_run/backup/*.bak_261011_*`
  - `radxa.py`, `console.py`, `rover_README.md`: 병합 전 261010 판
  - `path_follower.py`, `follower.yaml`: 이전 PID 추종기
- 그 밖의 백업(`sim_rover`·`send_path` 픽셀 판, 예전 `PATH_TRACKING.md`): `DARON_auto/server/backup/`
