# 통합 RC / ROS 제어 변경 사항

## 코드 위치

```text
rover/
├── radxa.py                 # 실행 진입점
├── rover_node/
│   ├── app.py               # ROS 초기화·종료
│   └── node.py              # RoverNode, 토픽·모드 중재·진단
├── motor_control/
│   ├── xm430/
│   │   ├── driver.py        # DYNAMIXEL 통신
│   │   ├── link.py          # 실제/mock 모터 초기화
│   │   ├── mock.py
│   │   ├── axis.py          # 굴절·형상·서스펜션 제어 스레드
│   │   ├── suspension.py    # 서스펜션 상태기계
│   │   ├── config.py
│   │   ├── log.py
│   │   └── metrics.py
│   ├── l12/
│   │   ├── driver.py        # 조향 기구학·L12 출력
│   │   └── pwm.py           # sysfs PWM 준비
│   └── vesc/
│       └── xerun.py         # VESC UART를 통한 XeRun 구동
├── rc_control/              # PPM 수신·RC 명령 계산
├── config/                  # 기존 front.json, back.json
├── profiles/                # 기존 XM430 프로파일
└── server/                  # 경로 추종
```

기존 `python3 radxa.py`, launch, 콘솔·캘리브레이션 실행 경로는 그대로입니다.
라이브러리를 사용하는 코드는 `from motor_control.xm430.link import open_motor`,
`from motor_control.l12.driver import L12Pair`처럼 import합니다.
드라이버 단독 명령은 rover 폴더에서 `python3 -m motor_control.vesc.xerun --dual --check-config`처럼 실행합니다.
패키지를 포함한 rover 폴더 전체를 배포하세요. `radxa.py` 한 파일만 복사하면 실행되지 않습니다.
별도 `daron_rc/install_board.sh`는 옛 단일 파일 구조용이며 현재 통합 노드에 적용하지 않습니다.

현재 `config/back.json`의 `direction_sign=-1`과 VESC `ROLES`의 `back=+1`이 불일치합니다.
설정 검사는 이를 거부합니다. 실제 배선·회전 방향 확인 후 코드와 설정을 함께 맞춰야 합니다.
구조 리팩토링에서는 이 물리 설정을 변경하지 않았습니다.

- `radxa.py`는 실행 진입점입니다. 구현은 `rover_node/node.py`에 있으며 중복 실행 파일 `radxa_rc.py`는 제거했습니다.
- 기존 ROS 토픽, 메시지 타입, 노드 이름과 주요 함수 입출력은 유지합니다.
- 내부 RC 상태는 `rover_node/node.py`의 `self._rc` (`RCCommand`) 한 곳에서 읽습니다. 설정과 PPM 해석은 `rc_control/rc_controller.py`, 실제 출력은 `rover_node/node.py`에서 확인하면 됩니다. 주기적인 RC 콘솔 로그는 제거했고, 모드 전환·신호 유실·오류만 기록합니다. 반복 입력 오류는 2초 간격으로 로그를 제한합니다.
- 조향 기하 계산은 굴절각 **150°** (`theta=-30°`)로 고정합니다. XM430 피드백 변화가 L12 조향 길이를 바꾸지 않습니다. XM430의 서스펜션, 자세 전환, CH4 굴절 제어는 별도로 유지합니다. 실제 관절을 항상 150°로 잠그는 설정은 아닙니다.
- 운용 조향 기본값은 **±20°**입니다. 콘솔·GUI·추종기도 같은 기본값을 사용합니다. `MAX_STEER_DEG`는 기존 기구학 한계 약 43°를 뜻하며 운용 기본값과 구분합니다.
- NaN/Inf 및 허용 범위 밖 조향 입력은 ERROR 로그와 중립 지령으로 처리합니다. 이전 지령 유지나 최대각 클램프를 하지 않습니다. 순수 계산 함수는 `ValueError`, 하드웨어 래퍼는 중립을 요청한 뒤 `L12Error`를 발생시킵니다. `clamp` 인자는 호출 호환성 때문에 남지만 초과 입력을 허용하지 않습니다.
- GPIO PPM: `gpiochip3:20`, 8채널. CH1 낮음(≤1200 µs)은 Wi-Fi, 높음(≥1800 µs)은 RC이며 서로 다른 유효 프레임 3개로 전환을 확인합니다. CH2는 구동 RPM, CH3는 조향, CH4는 굴절 각속도입니다.
- RC 진입·신호 복구 시 스로틀 중립 확인 후 구동을 허용합니다. 신호 유실/잘못된 프레임은 RC 정지·중립으로 처리하고 RC 권한을 유지합니다. Wi-Fi로 자동 전환하지 않습니다. CH1 전환 시 이전 소스의 명령을 지워 재실행을 막습니다.
- RC 선택 중 ROS 운동·전복 명령은 개입하지 않습니다. `stop`, `torque_off`는 허용합니다. Wi-Fi에서는 기존 수동 우선/자율 중재가 동작합니다. 진단에 `control_source`, `rc_connected`, `rc_armed`가 추가됩니다.

GPIO 없이 명시적으로 실행할 때는 `-p rc_enable:=false`를 추가합니다. 하드웨어 없는 실행은 기존 `susp_mock:=true l12_enable:=false vesc_enable:=false`와 함께 사용합니다. 실제 ROS2/Radxa 구동은 해당 환경에서 검증해야 합니다.

조향 한계 점검(로버 노드를 종료한 상태):

```sh
python3 check_steering_limits.py --angle 20         # 계산만, 하드웨어 쓰기 없음
python3 check_steering_limits.py --angle 5 --apply  # 준비된 PWM으로 단일 각도 시험
python3 -m unittest discover -s tests -v
```

`--apply`는 사용자가 Enter를 누르면 지정 각도로 이동하고, 다시 Enter/Ctrl+C를 누르면 중립 복귀 후 출력을 끕니다. 좌우를 작은 각도부터 따로 측정하세요. 자동 충돌 감지나 실제 각도 센서는 없으므로 출력된 기구학 한계는 실측 한계가 아닙니다. 실측 후 여유를 둔 작은 쪽 한계를 `max_steer_deg`와 송신 측 한계에 함께 반영하세요. 추종기 `delta_max_deg`와 `kappa_max=tan(delta_max)/0.3313`도 함께 맞춥니다.

아래는 기존 장치·실험 설명입니다. 조향 클램프와 동적 굴절 기하에 관한 이전 설명보다 위 규약이 우선합니다.

---

# DARON — 로버 통합 제어 (준최종)

굴절형 4륜 로버의 **굴절·서스펜션 / 조향 / 구동**을 한 노드에서 돌리는 통합본이다.
`XeRun_motor_test`(ROS 2 로버 노드)와 `xm430_suspension_test`(서스펜션 상태기계 +
캘리브레이션 도구)를 합쳤다.

261011: `DARON_auto` 의 **자율주행 수신부**(rpm/조향 지령 수신, 수동/자율 중재, `/rover/odom`)를
`radxa.py`·`console.py` 에 합쳤고, 지상국 경로 추종기(Stanley 곡률 제어)를 `server/` 에 넣었다.
→ **12절**, 상세는 [`PATH_TRACKING.md`](PATH_TRACKING.md)

운용 순서는 두 단계다.

```
① RADXA 에서 로컬 캘리브레이션 (최초 1회, ROS 없이)   →  profiles/rover.json
② 원할 때 메인 기동 (RADXA: radxa.py  +  지상국: console.py)
```

프로파일이 없으면 `radxa.py` 는 기동하지 않는다. 순서를 코드가 강제한다.

---

## 1. 무엇이 어디서 도는가

```
 ┌──────────────── RADXA (로버 본체) ─────────────────┐      ┌─── 지상국 ───┐
 │                                                    │      │              │
 │  radxa.py  (ROS 2 노드 /rover)                     │      │  console.py  │
 │   ├─ 서스펜션 스레드  100 Hz ── XM430 (FTDI/UART)  │ DDS  │   pygame 창  │
 │   │    HOLD/COMPLY/RECOVER/YIELD/CMD               │◄────►│   W A S D    │
 │   ├─ ROS 타이머 20 Hz ── L12 x2 (sysfs PWM)        │      │   Q E SPACE  │
 │   │                   └─ VESC x2 (UART, 각 스레드) │      │   + CSV 로그 │
 │   └─ 발행 10 Hz  /joint_states, /rover/diagnostics │      │              │
 │                  /rover/battery, /rover/odom       │      │ server/      │
 │   ◄── /rover/auto/{drive_rpm,steer_cmd} 50 Hz ─────┼──────┤ path_follower│
 └────────────────────────────────────────────────────┘      └──────────────┘
```

`server/` 폴더는 **지상국에서만** 쓴다. RADXA 에는 없어도 된다 (12절).

| 축 | 하드웨어 | 하는 일 |
|----|----------|---------|
| 굴절 + 서스펜션 | XM430-W210-R x1 | 굴절각 지령(조그)과 충격 흡수·복원을 같은 축이 겸한다 |
| 조향 | Actuonix L12 x2 (전/후) | 차량 유효조향각 → 전·후방 인출 길이 |
| 구동 | VESC x2 (front/back) | 전진/후진/정지. 전·후 반전은 `config/*.json` 의 `direction_sign` 이 한 번만 적용 |

### 왜 굴절축이 서스펜션인가

굴절 관절과 서스펜션은 요구가 정반대다. 정지 토크는 **낮아야** 하고(충격에
눌려줘야 한다), 외란 복귀는 **빨라야** 한다. 단일 전류 상한으로는 동시에 만족할
수 없다. 해법은 Goal Current(102)가 RAM 이라 매 주기 바꿀 수 있다는 점이다.
요구를 서로 다른 상태에 배치한다.

| 상태 | 언제 | Goal Current |
|------|------|--------------|
| `HOLD` · | 밴드 안. 물렁한 구간 | `current_holding` (낮음) |
| `COMPLY` ↓ | 외력이 축을 밀어내는 중 | `current_comply` (더 낮음) |
| `RECOVER` ↑ | 외력이 사라짐. 복귀 중 | `current_recover` (높음) + Goal Position 재기입 |
| `YIELD` ! | 계속 눌려 있음(복원 timeout) | `current_holding` — 모터 보호 |
| `CMD` → | 조그·절대 지령 이동 | `current_moving` |

상태기계 본체는 `motor_control/xm430/suspension.py`, 값은 캘리브레이션이 만든
`profiles/rover.json` 에서 온다. 자세한 근거는 `CALIBRATION.md`.

---

## 2. 설치

### RADXA (로버 본체)

```bash
pip3 install -r requirements.txt       # dynamixel-sdk, pyserial
sudo usermod -aG dialout $USER         # 시리얼 권한 (재로그인 필요)
sudo sh setup_latency.sh               # FTDI latency 16 ms -> 1 ms (처음 한 번)
```

`latency_timer` 를 안 낮추면 서스펜션 루프가 **60 Hz 근처에서 막힌다**.
외력 '해제' 검출이 한 박자 늦어 복원이 굼떠 보인다. 부팅하면 되돌아가므로
영구 적용은 `setup_latency.sh` 아래쪽의 udev 규칙을 참고한다.

ROS 2 (`rclpy`, `std_msgs`, `sensor_msgs`, `diagnostic_msgs`) 는 배포판으로
설치한다. 캘리브레이션 도구는 ROS 없이 돈다.

L12 의 sysfs PWM 은 `radxa.py` 가 시작할 때 자동으로 export 한다
(`l12_auto_export`).

**L12 PWM 배선 (261010 변경)**: 전방 **PWM8**, 후방 **PWM9** (이전 후방 PWM14). `rsetup` 의 Overlays 에서
PWM8·PWM9 를 켜고(쓰지 않는 PWM14 는 꺼도 됨) 재부팅한다. 경로는 `l12_front_pwm` / `l12_rear_pwm` 에
`pwm8` 처럼 **PWM 번호**로 주면 노드가 블록 주소(`fe6f0000`, `fe6f0010`)로 실제 `pwmchipN` 을 찾는다
(overlay 를 켜고 끌 때마다 pwmchip 번호가 밀리기 때문). 예전처럼 `/sys/class/pwm/pwmchip1/pwm0` 경로도 된다. `/sys` 쓰기 권한이 필요하므로 root 로 실행하거나,
컨테이너라면 `--privileged`, 일반 사용자라면 PWM udev 규칙이 있어야 한다.

### 지상국

```bash
pip3 install pygame                    # 창이 떠야 키 이벤트를 받는다
```

### 양쪽 공통 (DDS)

```bash
export ROS_DOMAIN_ID=<같은 번호>
# 멀티캐스트가 막힌 망이면:
export CYCLONEDDS_URI=file://$PWD/cyclonedds.xml    # 안의 Peer address 를 상대 IP 로
```

---

## 3. ① 최초 1회 — 로컬 캘리브레이션 (RADXA)

**로버를 주행 자세로 바닥에 내려놓고**, 차체 하중이 굴절축에 실린 상태에서 한다.
들어 올린 채로 재면 전부 무의미하다. 약 5 분 걸린다.

```bash
python3 calibrate.py --auto-zero --out profiles/rover.json
```

| 단계 | 하는 일 | 정하는 값 |
|------|---------|-----------|
| 0 굴절 원점 | 양쪽 기계 스토퍼를 저전류로 눌러 중점을 찾는다 | `home_deg` (= 로버의 굴절 원점) |
| 1 유지 전류 | 전류를 내려가며 차체가 주저앉는 지점을 찾고 1.8 배 여유 (`--hold-margin`, 261015 전 1.5) | `current_holding`, `current_comply`, `settled_band_deg`, `comply_band_deg` |
| 2 비대칭 측정 | 모터가 스스로 ±12° 변위를 만들고 복원시켜 `R` 측정 | — |
| 3 비대칭 보정 | `R` 이 5 % 밖일 때만. 전류/궤적 중 병목인 쪽을 알아서 고름 | `b_pos`/`b_neg` 또는 `profile_velocity_pos/neg_dps` |

`R` = `t90(+CCW) / t90(-CW)`. 양방향 복원 시간의 비이고 `1.0` 이면 대칭이다.

결착을 바꾸지 않아 원점을 이미 알고 있으면 `--auto-zero` 대신 `--zero 22.5`.

### 확인 (선택, 그러나 권장)

```bash
python3 run_suspension.py --profile profiles/rover.json
```

ROS 없이 서스펜션만 돌린다. 축을 눌렀다 놓으면 복원 이벤트가 한 줄씩 찍힌다.
양방향으로 5 회씩 눌러 `t90` 이 좌우 비슷하면 캘리브레이션이 맞은 것이다.
종료(Ctrl-C)해도 기본값은 **토크 유지**라 차체가 내려앉지 않는다.

자세한 절차·옵션·문제 해결은 **[`CALIBRATION.md`](CALIBRATION.md)**.

---

## 4. ② 메인 기동

### RADXA

```bash
python3 radxa.py
```

`profiles/rover.json` 을 읽고, XM430 을 잡고, L12 PWM 을 export·초기화하고,
VESC 두 채널을 연 뒤 `/rover` 노드로 뜬다. 기동 로그에 실제로 적용된 프로파일
값이 전부 찍히므로 한 번 훑어볼 것.

자주 쓰는 옵션:

```bash
python3 radxa.py --ros-args -p susp_profile:=profiles/firm.json   # 다른 프로파일
python3 radxa.py --ros-args -p vesc_enable:=false                 # 구동 없이
python3 radxa.py --ros-args -p l12_enable:=false                  # 조향 없이
python3 radxa.py --ros-args -p jog_limit_deg:=30.0                # 굴절 범위 좁히기
python3 radxa.py --ros-args -p susp_on_exit:=off                  # 종료할 때 토크 해제
python3 radxa.py --ros-args -p susp_set:="current_holding=45;d_gain=700"
python3 radxa.py --ros-args -p susp_mock:=true \
                            -p l12_enable:=false -p vesc_enable:=false  # 하드웨어 없이
```

### 지상국

```bash
python3 console.py
python3 console.py --ros-args -p max_steer_deg:=30.0 -p log_dir:=/data/logs
```

창이 뜨고 상단에 서스펜션 상태와 전복 여부가 보이면 연결된 것이다.
`NO LINK` 면 `ROS_DOMAIN_ID` 와 DDS 설정을 본다.

### 종료

콘솔은 `ESC`, 노드는 `Ctrl-C`. 노드는 구동을 0 으로 내리고 → VESC 를 끄고 →
L12 를 중립으로 보낸 뒤 → 서스펜션 축을 **토크를 유지한 채** 놓는다
(`susp_on_exit=hold`). 서보가 마지막 지령을 붙들고 있어 차체가 주저앉지 않는다.

---

## 5. 조작

| 키 | 동작 |
|----|------|
| `W` / `S` | 전진 / 후진 (누르는 동안, front+back 동시) |
| `A` / `D` | 조향 좌 / 우 (누르는 동안 최대 유효조향각, 떼면 중립) |
| `Q` / `E` | 굴절 CW / CCW (누르는 동안 회전) |
| `SPACE` | 전복 토글 (정상 ↔ 전복) |
| `1` | 중립 모드 — 굴절각 150° 로 펴고 조향 중립 |
| `2` | 수평 모드 — 굴절각 180° |
| `0` | 수납 모드 — 노드가 수납 상태로 켜졌을 때의 자세로 접기 |
| `P` | 자율 허용 토글 (AUTO ON ↔ PAUSED). 꺼지면 로버가 자율 지령을 무시 |
| `ESC` | 종료 |

자율 주행 중에도 키보드가 이긴다. W/S/A/D 를 누르는 순간 수동, 키를 뗀 뒤 1 s 가 지나면 자율로 돌아간다.

`A`+`D` 또는 `Q`+`E` 를 같이 누르면 그 축은 중립/정지한다.

### 부호 규약

* **굴절** `alpha = 180 + theta`. `theta` 는 굴절 원점 기준 XM430 각도.
  `theta=0 → 180°(평면 전개)`, `-30 → 150°(정상 굴절, 굴절부가 위로)`, `+30 → 210°(반대 굴절, 굴절부가 아래로)`.
  굴절각이 커질수록 굴절부가 내려오고, 더 키우면 앞바퀴가 들린다(수납 쪽).
  조그 `E`(CCW,+1)는 theta 증가 = 굴절각 증가, `Q`(CW,−1)는 theta 감소 = 굴절각 감소.
  (261010 부호 정정 — 이전 판은 `180 - theta` 로 실제와 반대였다. 변환은 `l12_driver.articulation_deg` 한 곳.)
* **조향** 보내는 값은 *차량* 유효조향각 Δv 다. 노드가 축 유효조향각으로
  환산한다 (`tan Δa = tan Δv / 2`). 기본 한계 43.003° 는 기계 한계인
  축 25° 와 같은 최소 회전반경이다.
* **전복** 굴절각의 부호와는 **별개의 독립 상태**다. 조종자가 스페이스바로
  알려주면 노드가 조향과 구동의 부호를 함께 뒤집어 조종 감각을 유지한다.
* **구동** 전진 = 위에서 봤을 때 시계방향. 전·후방 모터의 장착 방향 반전은
  `vesc_driver.ROLES` 와 `config/<role>.json` 의 `direction_sign` **두 곳(값이 같아야 함)** 이
  단 한 번 적용한다. 반대로 돌면 **둘을 함께** 뒤집는다 — 한쪽만 바꾸면 `load_role_config` 가
  거부해 구동 전체가 꺼진 채 뜬다 (261017 실제 발생: fc5c425 가 ROLES back 을 +1 로 바꿨는데 back.json 은 −1).
* **양쪽 ON 요구** 두 VESC 가 모두 ON 이 되기 전에는 어느 쪽에도 0 이 아닌
  지령을 주지 않는다 (한 축으로 끌리는 것을 막는다). 운전 중 한쪽이 죽으면
  즉시 양쪽을 0 으로 만든다.

### 조그와 서스펜션의 관계

조그를 누르는 동안은 `CMD` 상태(`current_moving`)로 한계까지 이동하고,
**키를 뗀 자리가 서스펜션의 새 기준 자세(home)** 가 된다. 즉 굴절각을 바꾼
뒤에도 그 자세에서 서스펜션이 그대로 동작한다.

다만 목표를 자르는 범위는 **굴절 원점 ± `jog_limit_deg`(기본 45°)** 에
고정되어 있다. home 이 조그를 따라 움직여도 이 창은 움직이지 않으므로, 조그를
반복해도 축이 기계 한계 밖으로 끌려가지 않는다.

### 형상 모드 (수납 / 중립 / 수평)

* **수납 상태로 시작** 노드가 켜질 때 굴절각이 `stow_detect_alpha_deg`(기본 250°)
  이상이면 수납 상태로 본다. 굴절축은 그 자리를 큰 전류로 붙들고, L12 초기화(중립
  이동)도 미룬다. 즉 **아무것도 움직이지 않는다.** `1`/`2` 를 눌러야 비로소 펴지고
  조향이 중립으로 돌아온다. 250° 미만이면 `1` 을 누른 것과 같이 **중립(150°)** 으로
  간다. 굴절 원점(`home_deg`, 굴절각 180°)은 각도의 기준일 뿐 시작 자세가 아니다.
* **판정 기준은 캘리브레이션 원점** 굴절각은 `profiles/rover.json` 의 `home_deg`
  기준이다(수납 = 원점 +70° 이상). XM430 은 전원을 켜면 엔코더를 0~360° 안으로 다시
  잡으므로, 시작할 때 원점을 읽힌 엔코더와 같은 회전으로 360° 단위만큼 옮겨
  `엔코더 − 원점` 이 [−180°, 180°) 에 들게 한다. 원점 22.5° 에서 중립(−7.5°)이
  352.5° 로 읽혀도 원점을 382.5° 로 옮겨 굴절각 150° 로 본다. 옮긴 값은 그 실행에만
  쓰고 프로파일에는 저장하지 않는다. 기동 로그에 `굴절 원점 +22.50 -> +382.50 deg` 로 찍힌다.
* **전환 방식** `xm430_angle_hold/hold_angle.py --hold-current 600` 과 같다.
  게인·궤적을 P800/I0/D0, 20°/s, 40°/s² 로 바꿔 600 LSB 로 천천히 옮긴다.
  도착하면 중립·수평은 서스펜션에 축을 돌려주고(그 자세가 home), 수납은 600 LSB
  로 계속 고정한다. Current Limit 은 600 이상으로 자동으로 올라간다.
* **잠금** 전환 중과 수납 상태에서는 구동·조향·조그를 막는다. 전환 중 `stop`
  (콘솔 종료 포함)이 오면 그 자리에서 고정한다.
* **수납 목표** 수납 상태로 켜졌을 때의 위치다. 그렇지 않게 켰으면 `0` 은
  거부된다. 이때는 `-p stow_alpha_deg:=<굴절각>` 으로 목표를 정해 준다.
* 관련 파라미터: `neutral_alpha_deg`, `flat_alpha_deg`, `stow_detect_alpha_deg`,
  `stow_alpha_deg`, `shape_hold_current`, `shape_move_current`,
  `shape_velocity_dps`, `shape_accel_dpss`, `shape_p_gain`/`i`/`d`,
  `shape_band_deg`, `shape_slip_deg`, `shape_timeout_margin_s` (런타임 변경 불가).

### 텔레옵 워치독

콘솔은 지령을 10 Hz 로 계속 보낸다. `teleop_timeout`(기본 0.5 s) 동안 아무것도
안 오면 노드가 스스로 조향을 중립, 조그를 정지, 구동을 0 으로 되돌린다.
링크가 끊겼을 때 액추에이터가 꺾인 채 남거나 로버가 계속 굴러가는 것을 막는다.

---

## 6. 로그

| 파일 | 어디서 | 주기 | 내용 |
|------|--------|------|------|
| `logs/rover_*.csv` + `.meta.json` | RADXA | **100 Hz** | 서스펜션 정본. 상태·목표·위치·속도·전류·Goal Current·이벤트 태그 |
| `logs/xm430_*.csv` | 지상국 | 10 Hz | XM430 + 서스펜션 요약 |
| `logs/l12_*.csv` | 지상국 | 10 Hz | 조향 해 (유효조향각, 곡률, 캐스터각, 휠 조향각, 인출 길이) |
| `logs/drive_*.csv` | 지상국 | 10 Hz | 구동 요약 (front/back 한 행에 나란히) |

복원 지표(`t90` 등)를 뽑으려면 **RADXA 쪽 100 Hz 로그**를 봐야 한다.
지상국 10 Hz 로는 복원 파형을 못 잡는다. 끄려면 `-p susp_local_log:=false`.

```bash
python3 analyze.py summary "logs/rover_*.csv"     # 복원 이벤트별 지표
python3 analyze.py asym    "logs/bidir_*.csv"     # 방향 대조 + R
python3 analyze.py compare "logs/sweep_*.csv"     # 런 간 비교
```

| 지표 | 뜻 | 이상 신호 |
|------|-----|-----------|
| `t90` | 복원량의 90 % 도달 시간 = 복원 속도의 본체 | — |
| `정착` | 최종 평형에 자리 잡은 시각 | `t90` 의 2 배 넘으면 진동 |
| `OS` | 오버슈트 [%] | 25 % 넘으면 `d_gain` ↑ |
| `진동` | 평형선 교차 횟수 | 2 이상이면 감쇠 부족 |
| `잔류` | 도달한 평형점의 오차 = 정적 처짐 | — |
| `Ipk` | 복원 중 최대 전류 | 상한의 80 % 미만이면 전류가 병목이 아님 |

`결과` 열: `settled`(밴드 안 완료) · `residual`(평형 도달, 처짐 남음 — 정상) ·
`timeout`(계속 눌려 있음) · `reimpact`(복원 중 새 충격).

---

## 7. 기동 중 튜닝

서스펜션 파라미터 이름은 프로파일과 ROS 파라미터가 같다. 그대로 바꾼다.

```bash
ros2 param set /rover current_holding 45
ros2 param set /rover current_recover 170
ros2 param set /rover p_gain 120
ros2 param set /rover d_gain 700
ros2 param set /rover dir_comp_mode bias
ros2 param set /rover b_pos 12.0
ros2 param list /rover                     # 바꿀 수 있는 값 전체
```

변경은 서스펜션 스레드로 요청이 들어가 다음 주기에 반영된다 (시리얼 포트는
그 스레드만 만지므로 ROS 쪽에서 레지스터를 직접 쓰지 않는다). 모순된 값
(`comply_band ≤ settled_band` 등)은 거부되고 이유가 돌아온다.

`current_limit`(EEPROM), `port`, `dxl_id`, `baudrate`, `susp_*` 는 런타임에
못 바꾼다. 노드를 다시 띄워야 한다.

**이 변경은 휘발성이다.** 마음에 들면 프로파일에도 남긴다.

```bash
python3 -m motor_control.xm430.config --profile profiles/rover.json \
        --set current_holding=45 --set d_gain=700 --save profiles/rover.json
```

더 파고들 때 (전부 ROS 없이, `radxa.py` 를 끄고 쓴다):

```bash
python3 susp_console.py --profile profiles/rover.json    # 키 하나로 실시간 튜닝
python3 experiments.py sweep --profile profiles/rover.json \
        --grid p_gain=80,120,160 --grid d_gain=400,600,800
python3 asymmetry.py breakaway --angles -15,0,15         # 비대칭의 '원인'
python3 find_zero.py check                               # 원점 육안 확인
```

---

## 8. 안전

* **가동 범위** `굴절 원점 ± jog_limit_deg` 는 *프로그램이 그 밖을 지령하지
  않는다*는 뜻이지 *그 밖으로 나가면 멈춘다*는 뜻이 **아니다**. 서스펜션은
  눌리는 것이 일이고, 큰 충격에 경계를 넘는 일은 정상이다. 그때 실행을 멈추면
  오히려 토크가 풀려 차체가 주저앉는다. 범위 밖 목표는 경계로 잘리고 경고만 찍힌다.
* **YIELD** 복원이 `recover_timeout_s`(기본 2 s)를 넘으면 유지 전류로 내려가
  대기한다. 계속 눌린 상태에서 모터를 밀어붙이면 발열만 쌓인다.
* **종료 시 토크 유지** 기본값이다. 힘을 빼려면 `-p susp_on_exit:=off`.
* **구동 양쪽 ON 요구** 위 5절 참고. 진단 목적이면 `-p vesc_require_both:=false`.
  운전 중 한쪽이 trip 하면 살아 있는 쪽도 OFF 로 내리고, `vesc_rearm_s`(기본 1 s) 가 지난 뒤에야
  다시 ON 을 시도한다 (0 = 즉시 재시도, 이전 동작).
* **서스펜션 축 장애** XM430 통신이 끊기면 굴절각을 모르게 되고 조향 해가
  틀어진다. 그 상태로 구르는 것이 가장 위험하므로 노드가 구동을 0 으로 내린다.
* **`current_limit`** 은 EEPROM 이라 Torque OFF 가 필요하다. 런타임에 못 바꾸므로
  처음부터 실험 범위를 덮도록 잡아 둔다.

---

## 9. 트러블슈팅

| 증상 | 원인 / 조치 |
|------|-------------|
| `[프로파일 없음]` 으로 기동 거부 | 캘리브레이션을 먼저 돌린다. 정말 기본값으로 띄우려면 `-p susp_require_profile:=false` |
| 콘솔의 `Susp Loop` 가 80 Hz 미만 | `sudo sh setup_latency.sh`. 그래도 낮으면 `-p susp_hz:=50.0` 으로 낮춰 맞춘다 |
| 복원이 `timeout` 으로만 끝남 | 축이 계속 눌려 있거나 `current_recover` 가 하중을 못 이김 |
| 상태가 `HOLD`↔`COMPLY` 로 채터링 | `comply_band_deg` 를 `settled_band_deg` 보다 충분히 크게 |
| 잔류(정적 처짐)가 큼 | `current_holding`/`p_gain` ↑, 또는 `i_gain` 소량 |
| 오버슈트·진동 | `d_gain` ↑ 또는 `current_recover` ↓ |
| 좌우 복원 속도가 다름 | 캘리브레이션 3단계(`R`)를 다시. `--amp 20` 으로 변위를 키워 본다 |
| 콘솔 `NO LINK` | `ROS_DOMAIN_ID`, `CYCLONEDDS_URI`(Peer address 는 *상대* IP), 방화벽 |
| `FTDI 장치를 찾지 못했습니다` | `ls -l /dev/serial/by-id/`, `lsusb \| grep -i ftdi`, `dialout` 그룹 |
| `dxl_driver import 실패` | `pip3 install dynamixel-sdk pyserial`, 또는 `-p susp_mock:=true` |
| `L12 ... simulate 모드` | PWM 채널 export 실패. 위의 PWM 준비 로그에 원인별 조치가 찍힌다 |
| `pwmchipN 가 없습니다` | export 로는 해결 안 됨. device tree overlay 문제 |
| 구동 출력이 계속 OFF | 두 VESC 가 모두 ON 이어야 한다. 콘솔의 `Drive ON F/B` 와 `Drive Error` 행을 본다. 기동 로그에 "VESC 설정 로드 실패" 가 있으면 `ROLES` ↔ json `direction_sign` 불일치 |
| 로버가 반대로 감 | `vesc_driver.ROLES` 와 `config/<role>.json` 의 `direction_sign` 을 **함께** 뒤집는다 (둘이 다르면 "VESC 설정 로드 실패 … 구동모터 없이 진행" 으로 뜬다) |
| 좌우 조향이 반대 | 굴절 원점이 틀렸을 가능성. `find_zero.py limits` 로 다시 |

---

## 10. 파일

### 운용

| 파일 | 역할 |
|------|------|
| `radxa.py` | **통합 ROS 2 노드.** 서스펜션 스레드(100 Hz) + L12/VESC/발행(20·10 Hz) |
| `console.py` | **지상국 조종 콘솔.** pygame 창, 키 입력, CSV 3종 |
| `cyclonedds.xml` | 멀티캐스트가 막힌 망에서 쓰는 DDS 유니캐스트 설정 |
| `config/front.json`, `config/back.json` | VESC 역할별 설정 (`direction_sign` 포함) |
| `server/` | **지상국 경로 추종기** (`path_follower.py` 등). RADXA 에는 필요 없다 — 12절 |
| `PATH_TRACKING.md` | 경로 추종 문서 (배치·인터페이스·제어·파라미터·검증) |

### 캘리브레이션 (ROS 없이, RADXA 로컬)

| 파일 | 역할 |
|------|------|
| `calibrate.py` | **캘리브레이션 한 번에.** `--auto-zero` 로 원점까지 |
| `find_zero.py` | 굴절 원점 (`limits` / `check` / `hand` / `goto`) |
| `run_suspension.py` | 순수 서스펜션 운용 루프. 캘리브레이션 확인용 |
| `susp_console.py` | 터미널 실시간 튜닝 콘솔 |
| `experiments.py` | `sag` / `step` / `release` / `sweep` |
| `asymmetry.py` | `breakaway` / `bidir` / `cal` — 방향 비대칭 측정·보정 |
| `analyze.py` | CSV → 지표. 지표 정의는 여기 한 곳에만 있다 |
| `profiles/` | `soft` / `medium` / `firm` 프리셋, 캘리브레이션 결과 `rover.json` |

### 드라이버 / 공통

| 파일 | 역할 |
|------|------|
| `motor_control/xm430/driver.py` | XM430 제어 계층 (포트·configure·sync read·레지스터) |
| `motor_control/xm430/link.py` | 드라이버 연결 + `--mock` 분기 |
| `motor_control/xm430/config.py` | 서스펜션 파라미터 묶음 + 방향 보정식 + JSON 저장/로드 |
| `motor_control/xm430/suspension.py` | **서스펜션 상태기계.** `radxa.py` 와 도구들이 같은 것을 쓴다 |
| `motor_control/xm430/log.py` | 100 Hz CSV + `.meta.json` 기록 |
| `motor_control/l12/driver.py` | 조향 기구학 + sysfs PWM 구동 |
| `motor_control/vesc/xerun.py` | VESC UART 드라이버 (역할별 스레드, 텔레메트리) |
| `motor_control/xm430/mock.py` | 하드웨어 없이 코드 경로 점검용 시뮬레이터 (값을 뽑아 쓰면 안 된다) |
| `setup_latency.sh` | FTDI latency 16 ms → 1 ms |

---

## 11. ROS 인터페이스 요약

구독 (`/rover/...`)

| 토픽 | 타입 | 뜻 |
|------|------|-----|
| `steer_cmd` | `Float64` | 차량 유효조향각 [deg]. + = 좌회전 |
| `articulation_cmd` | `Int8` | 굴절 조그. +1 CCW / −1 CW / 0 정지 |
| `drive_cmd` | `Int8` | +1 전진 / −1 후진 / 0 정지 |
| `inverted` | `Bool` | 전복 여부 |
| `goal_position` | `Float64` | XM430 절대 목표각 [rad] |
| `goal_relative` | `Float64` | XM430 상대각 [rad] |
| `command` | `String` | `home` / `home_here` / `stop` / `torque_on` / `torque_off` |
| `heartbeat` | `Empty` | 워치독 갱신 |
| `auto/drive_rpm` | `Float64` | [자율] 모터 rpm. + 전진 (조종자 기준). `max_speed` 로 자름 |
| `auto/steer_cmd` | `Float64` | [자율] 차량 유효조향각 [deg] |
| `auto_enable` | `Bool` | 자율 허용 (콘솔 P 키). 기본 `auto_enable_default=true` |

발행

| 토픽 | 타입 | 뜻 |
|------|------|-----|
| `/joint_states` | `JointState` | XM430 위치/속도/토크 추정치 |
| `/rover/diagnostics` | `DiagnosticArray` | 서스펜션 상태·조향 해·구동 상태·배터리 전부 (콘솔 CSV 의 원천) |
| `/rover/battery` | `BatteryState` | 배터리 잔량 `percentage`(0~1)·`voltage`. 모르면 NaN, `present=false` |
| `/rover/odom` | `Odometry` | 휠 오도메트리 (odom → base_link). `config/*.json` 에 `gear_ratio`·`wheel_diameter_m` 이 있어야 나온다 |

### 배터리 잔량 (261010)

VESC 펌웨어가 직접 계산하는 `battery_level` 을 쓴다. `vesc_driver` 가 이미 1 Hz 로 받는
`COMM_GET_VALUES_SETUP` 응답에 들어 있어 UART 요청이 늘지 않는다. VESC 는 입력 전압을
자기 Battery 설정(`si_battery_type` = Li-ion 3.0–4.2 V, `si_battery_cells` = 3, 2.2 Ah —
`vesc_front.xml`/`vesc_back.xml`)으로 셀 전압 → Li-ion 방전 곡선에 대입한다.
**팩을 바꾸면 VESC Tool 의 Battery 설정도 같이 바꿔야** 값이 맞는다.

* front/back 중 fresh 한 값의 **낮은 쪽**을 `battery_pct` 로 낸다 (같은 팩, 보수적).
* 진단 키: `battery_pct`, `battery_voltage_v`, `battery_wh`, `battery_source`,
  `drive_front_battery_pct`, `drive_back_battery_pct`.
* `battery_pct < battery_warn_pct`(기본 20) 이면 진단 레벨 WARN `battery low NN%`.
  `ros2 param set /rover battery_warn_pct 25` 로 바꿀 수 있다.
* 콘솔 맨 위 `Battery` 줄, drive CSV 끝 열(`battery_pct`, `battery_voltage_v`, `battery_wh`).

---

## 12. 자율주행 (261011)

`DARON_auto`(9/30, 자율 수신부) 와 이 폴더(261010, 배터리·부호·PWM) 의 `radxa.py` 두 판을 합친 결과다.
261010 수정은 전부 그대로이고, 자율 기능을 그 위에 얹었다.

### 지령 중재

| 조건 | 모드 | 구동 | 조향 |
|------|------|------|------|
| 0 이 아닌 키 입력 후 `manual_override_hold_s`(1 s) 안 | `manual` | `drive_cmd` 방향 | `steer_cmd` |
| 그 밖에 `auto_enable` 이고 자율 지령이 `auto_timeout`(0.5 s) 안에 옴 | `auto` | `auto/drive_rpm` → VESC `set_speed` | `auto/steer_cmd` |
| 그 밖에 콘솔 링크 살아 있음 | `manual` | 0 (키 안 누름) | 0 |
| 아무것도 없음 | `idle` | 0 | 중립 |

* 형상 전환 중·수납(`drivable=False`)이면 **수동·자율 모두** 구동 0, 조향 중립 (261010 규칙 유지).
* 전복이면 rpm 부호를 뒤집는다 (조향은 l12_driver 가 뒤집는다).
* 진단 키: `drive_mode`, `drive_applied`(`dir:1` / `rpm:250`), `auto_enable`, `auto_rpm`, `auto_steer_deg`,
  `odom_x`, `odom_y`, `odom_yaw_deg`, `odom_v`. 콘솔에 `MODE`/`AUTO ON` 배지와 세 줄, drive CSV 맨 끝 8 열.
* 파라미터: `auto_timeout`, `manual_override_hold_s`, `auto_enable_default`, `odom_enable`, `odom_frame`, `base_frame`.

### 오도메트리·속도 (261017 갱신)

`config/front.json`·`back.json` 의 `gear_ratio` 는 **40** 이다 (261016 입력). 지상국 `server/config/follower.yaml` 의
`gear_ratio`(40.0)·`wheel_diameter_m`(0.108) 과 **같은 값**을 유지한다. 감속비 40 · `max_speed` 600 rpm 에서 상한 속도는
약 0.085 m/s (모터 rpm ≈ 7074·v) 라 추종기 `v_max_mps` 는 0.08 이다.

### 기동

```bash
# RADXA — 지금처럼
ros2 launch <daron_run>/launch/daron_rover.launch.py ros_domain_id:=42 susp_profile:=<rover.json>
# 지상국 — 추종기·pose_relay 는 daron_gui / daron_3dmap launch 가 띄운다 (따로 띄우면 두 벌이 된다)
ros2 topic pub --once /path_follower/enable std_msgs/Bool "{data: true}"
```

직접 띄워서 실험할 때만 (`daron_gui follower:=false pose_relay:=false` 로 끈 뒤):

```bash
python3 <daron_run>/rover/server/path_follower.py --ros-args \
        --params-file <daron_run>/rover/server/config/follower.yaml
```

배치·인터페이스·제어 법칙·검증 결과는 [`PATH_TRACKING.md`](PATH_TRACKING.md).
