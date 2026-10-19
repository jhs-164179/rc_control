# daron_rc — RC8X / R8FM PPM 4채널 통합 (261016)

RC 송신기(RadioLink RC8X → 수신기 R8FM PPM)를 로버 노드 `radxa.py`와 지상국 콘솔 `console.py`에 붙인 **시험판**입니다.
10/15 첨부 묶음(`DARON_RC_4CH_20261015.zip`, README_KO)의 설계를 따르되, 바탕은 저장소의 최신
`daron_run/rover/radxa.py`(261015 작업본)이고, 첨부 코드에서 찾은 안전 버그는 고쳤습니다(맨 아래 표).

**실제 하드웨어 미검증.** PC에서 단위 테스트, 그리고 가짜 PPM → PPM 노드 → mock 로버 → 콘솔까지 종단 시험만 했습니다.
보드에서는 아래 **보드 시험 순서**를 1단계부터 지켜 주세요.

---

## 1. 구성

```
daron_rc/
├── README.md                 이 문서
├── install_board.sh          보드: rover/* 를 daron_run/rover 에 설치 (기존 radxa.py 백업)
├── console.py                지상국: 원본 콘솔 + RC 패널(오른쪽) + X/R 키 + drive CSV rc_* 열
├── ground_rc_monitor.py      지상국: 채널·상태 한 줄 모니터 (기존 파일, 그대로)
├── rover/                    → 보드 /workspace/daron_run/rover/ 로 들어감
│   ├── radxa.py              daron_run/rover/radxa.py (261015) + RC 중재. rc_enable:=false(기본)면 원본과 같다
│   ├── rc_core.py            PPM 디코더 + RC/AUTO 중재 상태기계 (ROS·하드웨어 의존 없음)
│   ├── rc_ppm_node.py        GPIO PPM(gpiomon) → /rover/rc/channels. 모터 안 건드림
│   ├── rc_channel_monitor.py 터미널 진단 (채널 + 상태 변화)
│   ├── rc_env.sh             아래 run 스크립트 공통 (도메인·rclpy·중복 실행·안전 인자 검사)
│   ├── run_board_ppm.sh      PPM 노드
│   ├── run_board_dry.sh      RC 판단만, 출력 0 (XM430 mock, L12·VESC 꺼짐)
│   └── run_board_hardware.sh RC 실구동, 단계별 (l12 → xm430 → drive)
└── tests/test_rc_core.py     PC 단위 테스트 26개
```

`radxa.py`를 기준판에서 바꾼 범위: RC 파라미터·구독·발행, 중재 호출 한 곳(`_select_mode` → `_rc_step`),
진단 `rc_*` 키, 그리고 RC와 무관한 **기존 종료 버그 수정** 하나(6절).

## 2. 채널과 송신기(RC8X) 설정

| 채널 | 기능 | 코드 해석 | RC8X 설정 |
|---|---|---|---|
| CH1 | RC/AUTO 선택 | ≤1300 µs = **RC**, ≥1700 µs = **AUTO**, 그 사이 = **STOP**. 0.3 s 그대로여야 전환 | 공장값은 핸들 → **2단 스위치로 바꿔야 함** |
| CH2 | 구동 | 1500±100 µs 밖이면 +1 / -1 (방향 지령. 속도는 VESC `max_speed`·가감속이 정함) | 공장값(트리거). 실제 중립값 확인 |
| CH3 | 조향 | 1500±70 µs 밖을 다시 펴서 -1~+1 × 43.003° (경계에서 값이 튀지 않음) | 공장값(VR 다이얼) |
| CH4 | 굴절 조그 | `rc_ch4_enable` 일 때만. 1500±150 µs 밖이면 +1(CCW, 굴절각↑) / -1(CW) | PS1 단독은 중립·양방향이 안 됨 → **3단 입력 권장** |

- 채널 순서는 PPM 1-based 그대로입니다. 1단계에서 실제 조작 ↔ 채널을 먼저 확인하세요.
- 방향이 반대면 송신기 Reverse를 쓰거나 `-p rc_chN_reverse:=true`(N=1~4)를 주면 됩니다.
- **Failsafe가 제일 중요합니다.** R8FM이 RF를 잃었을 때 PPM을 끊으면 코드가 0.25 s 안에 STOP합니다.
  반대로 값을 붙들고 PPM을 계속 내면 소프트웨어로는 알 수 없습니다. 이때 RC8X Failsafe로 **CH1을 가운데(1500 µs 부근)**에
  둘 수 있으면 신호를 잃는 순간 STOP 구간에 들어갑니다. 어느 쪽인지는 1단계에서 송신기를 꺼 보고 확인합니다.

## 3. 중재 규칙 (rc_enable:=true)

- **RC**: CH1 RC → CH2를 중립(±90 µs)에 **0.5 s** 두면 arm → CH2/CH3/CH4로 조종.
  콘솔이나 제어창의 W/S/A/D/Q/E(Wi-Fi)는 모터로 가지 않습니다. 형상(1/2/0), 전복(SPACE), 자율 허용(P)은 그대로 받습니다.
- **AUTO**: CH1 AUTO → CH2 중립 0.5 s → `auto_enable=true` + 경로 추종기 rpm·조향 지령이 **둘 다** 0.5 s 안에 와야 주행합니다.
  AUTO 중에 CH2를 ±130 µs 넘게 0.15 s 당기면 정지하고, 다시 중립을 0.5 s 두면 재개합니다. AUTO로 바꾸는 권한은 CH1에만 있습니다.
- **STOP**: PPM 없음/0.25 s 끊김, CH1 가운데, 콘솔 X(래치) 중 하나면 STOP입니다. 출력은 0(조향 중립, 구동 0, 조그 0)이 됩니다.
- CH1 위치가 바뀌거나 링크가 끊기면 **arm이 바로 풀립니다**. 0.3 s 안에 RC↔AUTO가 튀어도 재arm 없이 다시 달리지 않습니다.
- 콘솔 **X** = SW STOP 래치입니다. CH1을 움직여도 풀리지 않고 **R**로만 풉니다. 풀린 뒤에도 CH2 중립 0.5 s로 다시 arm해야 합니다.
- `rc_actuation_enable:=false`(dry)이면 위 판단은 그대로 하지만 출력은 늘 0입니다. 대신 `would_*`(켜져 있었다면 낼 지령)를 보여 줍니다.
- 형상 전환 중이거나 수납 상태, XM430 fault는 원래대로 구동을 막습니다(RC 판단보다 뒤에 걸림).

## 4. 보드에 설치

RADXA 도커 안에서(저장소를 받아 둔 위치 기준):

```bash
rsync -a Project_DARON/Daron/daron_rc/ /workspace/daron_rc/
bash /workspace/daron_rc/install_board.sh --check     # 비교만 (아무것도 안 바꿈)
bash /workspace/daron_rc/install_board.sh             # 설치. 기본 대상 /workspace/daron_run/rover
#   다른 위치:  bash install_board.sh /경로/daron_run/rover   (또는 DARON_ROVER_DIR=...)
```

- 보드의 `radxa.py`가 이 묶음의 기준판(저장소 261015 작업본 또는 커밋 62b824f)이 아니면 **멈춥니다**.
  보드에서 직접 고친 내용이 덮이지 않도록 하는 장치입니다. 표시되는 `diff -u`로 확인하고, 필요한 변경을 옮겨 붙인 뒤
  `--force`로 설치하세요. 예전에 설치한 daron_rc 판은 갱신으로 보고 그대로 진행합니다.
- `radxa.py` / `rc_ppm_node.py`가 실행 중이면 설치하지 않습니다.
- 되돌리기: `cp /workspace/daron_run/rover/backup_rc_4ch_<시각>/radxa.py /workspace/daron_run/rover/`
  (`rc_*` 파일은 남아 있어도 원본 `radxa.py`가 쓰지 않으므로 무해합니다).
- 설치해도 `rc_enable` 기본값이 false라서, `daron_rover.launch.py`처럼 RC 인자 없이 띄우면 **지금과 똑같이** 키보드 중재로 돕니다.

보드에 필요한 것: `gpiomon`(libgpiod 1.6.x, `apt install gpiod`), 컨테이너에서 `/dev/gpiochip3` 접근(`--privileged` 등),
ROS 2 Humble. 서스펜션 프로파일이 `rover/profiles/rover.json`에 없으면 `SUSP_PROFILE=/workspace/DARON_keyboard_ctrl/profiles/rover.json`처럼 지정합니다.
도메인은 기본 `ROS_DOMAIN_ID=42`, `RMW=rmw_cyclonedds_cpp`입니다(셸에 이미 있으면 그 값을 씁니다).

## 5. 보드 시험 순서

각 단계를 통과해야 다음 단계로 넘어갑니다. **비상정지 수단(배터리 커넥터·전원 스위치)은 항상 손에 두세요.**
모든 명령은 `cd /workspace/daron_run/rover` 후 실행합니다. 터미널마다 ROS 환경이 잡혀 있어야 합니다.

### 1단계 — PPM만 (모터 노드 없음)

```bash
./run_board_ppm.sh                 # 터미널 A
python3 rc_channel_monitor.py      # 터미널 B
```

- [ ] A의 로그에 `프레임 45~50/s  버림 0  간격 개수 8 (기대 8)`이 찍힘 (프레임 주기에 따라).
      간격 개수가 8이 아니면 `PPM_CHANNELS=<그 수> ./run_board_ppm.sh`
- [ ] CH1 스위치 → CH1만, 트리거 → CH2만, 다이얼 → CH3만, CH4 입력 → CH4만 움직임
- [ ] CH1 양 끝이 ≤1300 / ≥1700, CH2·CH3 중립이 1500±(70~90) 안에 있음
- [ ] **송신기 OFF**: B가 `PPM 없음 / timeout`(좋음)인지, 값이 그대로 남는지(Failsafe hold → 2절대로 설정 변경) 확인.
      다시 켰을 때 정상 복귀하는지도 확인
- GPIO가 다르면 `./run_board_ppm.sh -p gpiochip:=gpiochip3 -p offset:=20`

### 2단계 — Dry 중재 (출력 0, XM430도 mock)

```bash
./run_board_ppm.sh                 # 터미널 A (1단계 그대로)
./run_board_dry.sh                 # 터미널 B — rc_ch4_enable 도 켜짐 (출력이 없으니 극성 확인용)
python3 rc_channel_monitor.py      # 터미널 C (또는 지상국 콘솔, 6절)
```

| 조작 | 기대 (`[STATUS]` 줄 / 콘솔 RC 패널) |
|---|---|
| CH1 RC, 트리거 놓음 | `source RC` → 0.5 s 뒤 `armed True`, `would manual`, `active idle` |
| 트리거 당김 / 밂 | `would_drive` +1 / -1 (전진이 +1인지 확인. 반대면 `-p rc_ch2_reverse:=true`) |
| 다이얼 좌/우 | `would_steer_deg` ±43 쪽으로 (좌회전이 +인지 확인) |
| CH4 | `would_jog` +1 = 굴절각 증가(굴절부 내려감) 쪽인지 확인 |
| 트리거 당긴 채 CH1 RC로 | arm 안 됨 (`hold CH2 neutral ...`) |
| CH1 AUTO | `source AUTO` → 중립 0.5 s 뒤 `auto_ready True`. 추종기가 없으면 `would idle` |
| 콘솔 X → R | `latched True` / `source STOP` → R 뒤 래치 풀림, 중립 0.5 s 뒤 재arm |
| 송신기 OFF | 0.3 s 안에 `link False`, `source STOP` |
| 터미널 A 종료(Ctrl+C) | B가 0.25 s 뒤 `link False` |

극성을 바꿔 볼 때는 `./run_board_dry.sh -p rc_ch3_reverse:=true`처럼 줍니다.
dry 스크립트는 `rc_actuation_enable`, `l12_enable` 같은 **안전 관련 인자를 덮는 것을 거부**합니다.

### 3단계 — L12 조향만

```bash
./run_board_hardware.sh --hardware-ready l12      # XM430 실기(시작하면 중립 150°로 이동 후 서스펜션 유지) + L12. VESC·CH4 꺼짐
```
- [ ] arm 전에는 다이얼을 돌려도 L12가 움직이지 않음 → arm 뒤 다이얼 방향 = 바퀴 조향 방향
- [ ] CH1 가운데 / 송신기 OFF / 콘솔 X 각각에서 L12가 중립으로 돌아감

### 4단계 — + XM430 굴절 조그 (CH4)

```bash
./run_board_hardware.sh --hardware-ready xm430
```
- [ ] CH4 +/- 가 2단계에서 확인한 방향으로 움직이고, 놓으면 그 자리를 새 기준 자세로 유지(`hold_here`)
- [ ] 조그 한계(원점 ±45°)에서 멈춤. 이전 `Incorrect status packet` / 형상 전환 timeout 이 다시 나오지 않는지 관찰

### 5단계 — + VESC 구동 (**바퀴를 공중에 띄우고**)

```bash
./run_board_hardware.sh --hardware-ready drive                 # CH4 꺼짐
RC_CH4=1 ./run_board_hardware.sh --hardware-ready drive        # CH4 도 켬
```
- [ ] 트리거 살짝 = 설정 속도(`config/*.json` `max_speed` 600 rpm, 가감속 600 rpm/s)로 도는 것을 이해하고 시작.
      비례 제어가 아니라 방향 지령입니다
- [ ] 전진/후진 방향, front/back 동시 ON(`Drive ON F/B` True/True)
- [ ] 회전 중 CH1 가운데 / 송신기 OFF / 콘솔 X / 노드 Ctrl+C → 감속 정지

### 6단계 — 지면 저속 → AUTO

AUTO는 지상국 경로 추종기가 떠 있고, 콘솔에서 **P(자율 허용)**를 켜고, CH1을 AUTO에 둔 다음 CH2 중립 0.5 s가 지나야 움직입니다.
제어창(ground_station)은 `/rover/auto_enable`을 보내지 않습니다. 콘솔 없이 쓰려면
`ros2 topic pub -r 5 /rover/auto_enable std_msgs/msg/Bool "data: true"`를 쓰세요.

### launch로 띄울 때

`daron_rover.launch.py`는 설치된 `daron_run/rover/radxa.py`를 그대로 쓰므로 인자만 더하면 됩니다(PPM 노드는 따로):
```bash
ros2 launch /workspace/daron_run/launch/daron_rover.launch.py ros_domain_id:=42 susp_profile:=... \
  rover_params:='rc_enable:=true rc_actuation_enable:=false rc_ch4_enable:=false auto_enable_default:=false'
```
launch 경로에는 run 스크립트의 안전 인자 검사가 없습니다. 실구동(`rc_actuation_enable:=true`)은 3~5단계를 마친 뒤에만 쓰세요.

## 6. 지상국 콘솔

```bash
source ~/slam_env.sh          # ROS_DOMAIN_ID=42, cyclonedds
/usr/bin/python3 Project_DARON/Daron/daron_rc/console.py      # pygame 필요: pip3 install pygame
```

- 왼쪽은 원본 `daron_run/rover/console.py`와 같고, 오른쪽에 **RC 4CH 패널**이 붙습니다.
  SOURCE(STOP/RC/AUTO), DRY/LIVE, 링크, arm, CH1~CH4 막대, `would` 지령, 그리고 지금 왜 안 움직이는지(reason)를 보여 줍니다.
- **X** = SW STOP 래치, **R** = 해제와 재arm 요청. 소프트웨어 정지이므로 Wi-Fi가 끊기면 전달되지 않습니다(물리 E-stop 아님).
- 자율 허용은 **꺼진 채(AUTO PAUSED)로 시작**합니다(원본은 켜진 채). 원본처럼 하려면 `-p auto_enable_start:=true`.
- `logs/drive_*.csv` 끝에 `rc_*` 열 15개가 붙습니다(앞 열은 원본과 같음). 로그 폴더는 실행한 위치의 `logs/`입니다.
- RC를 안 켠 로버에 붙이면 패널에 `disabled`만 뜨고, 나머지는 원본과 똑같이 동작합니다.
- 제어창(ground_station)의 로버 패널·수동 조종은 수정하지 않았습니다. RC 모드에서는 그쪽 키보드 조종도 로버가 무시합니다.

## 7. 토픽·파라미터

| 토픽 | 형 | 방향 |
|---|---|---|
| `/rover/rc/channels` | `std_msgs/Int16MultiArray` | rc_ppm_node → radxa. CH1~CH8 µs, **빈 배열 = 신호 없음** |
| `/rover/rc/status` | `std_msgs/String`(JSON, 10 Hz) | radxa →. `source, switch_request, active_mode, would_mode, rc_link, rc_armed, auto_ready, latched, actuation, ch1~ch4, steer_deg, drive, posture_jog, would_*, reason` |
| `/rover/control_mode/set` | `std_msgs/String` | 콘솔 → radxa. `STOP` / `RC`(=`RESUME`). `AUTO`는 거부 |
| `/rover/diagnostics` | 기존 | `rc_*` 키 추가 (`rc_source, rc_link, rc_armed, rc_latched, rc_would_*, rc_ch1~4, rc_reason …`) |

radxa.py 파라미터(기본값). `rc_enable`·`rc_actuation_enable`·`rc_ch4_enable`은 **재시작해야 바뀝니다**.
나머지는 `ros2 param set /rover <이름> <값>`으로 바로 바뀌고, 조합이 틀리면 거부됩니다.

| 이름 | 기본 | 뜻 |
|---|---|---|
| `rc_enable` | false | RC 중재 사용 |
| `rc_actuation_enable` | false | false = dry |
| `rc_ch4_enable` | false | CH4 굴절 조그 |
| `rc_timeout_s` | 0.25 | 채널 갱신이 이만큼 없으면 링크 없음 |
| `rc_center_us` / `rc_span_us` | 1500 / 500 | 중립 / 끝까지 민 값까지의 거리 |
| `rc_mode_threshold_low_us` / `_high_us` | 1300 / 1700 | CH1 RC 이하 / AUTO 이상 |
| `rc_mode_stable_s` | 0.3 | CH1 안정 시간 |
| `rc_neutral_arm_us` / `rc_manual_neutral_hold_s` | 90 / 0.5 | arm(=auto_ready) 중립 폭 / 유지 시간 |
| `rc_drive_deadzone_us` | 100 | CH2 구동 데드존 (**arm 폭 이상이어야 함** — 검사함) |
| `rc_steer_deadzone_us` | 70 | CH3 |
| `rc_ch4_deadzone_us` | 150 | CH4 |
| `rc_override_us` / `rc_override_hold_s` | 130 / 0.15 | AUTO 중 CH2 정지 조건 |
| `rc_ch1_reverse` … `rc_ch4_reverse` | false | 채널 부호 반전 (2·center − 값) |

rc_ppm_node 파라미터: `gpiochip`(gpiochip3), `offset`(20), `channels`(8), `sync_gap_us`(3000),
`min_us`/`max_us`(800/2200), `max_gap_us`(50000), `timeout_s`(0.25), `publish_hz`(20), `gpiomon`(실행 파일), `restart_s`, `stats_s`.

## 8. PC에서 테스트

```bash
cd Project_DARON
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 /usr/bin/python3 -m pytest -q Daron/daron_rc/tests     # 26 passed
```
(`PYTEST_DISABLE_PLUGIN_AUTOLOAD`는 `~/.local`의 anyio 플러그인이 시스템 pytest 6.2와 충돌해서 넣었습니다.)

종단 시험 도구(가짜 gpiomon, 시나리오 러너)는 `Project_DARON/local_test/rc_4ch/`(git 제외)에 있습니다.
반드시 **격리 도메인**에서 돌리세요: `ROS_DOMAIN_ID=77 ROS_LOCALHOST_ONLY=1 ./e2e_rc.py dry|live`.

## 9. 첨부판(10/15 zip)과 다른 점

| 첨부판 | 이 판 | 이유 |
|---|---|---|
| PPM 프레임을 간격 2~N개면 받음 | 동기 사이에 **정확히 N개**인 프레임만 받음. 시작 직후 조각이나 노이즈가 낀 프레임은 버림 | 부분 프레임을 받으면 채널이 밀려 **CH2(구동) 자리에 다른 채널 값**이 들어감 |
| 끊김 뒤 복귀 시 끊기기 전 프레임이 나감 | 50 ms 넘게 무신호면 대기 중이던 프레임을 버림 | 몇 초 묵은 CH1 값이 새 값처럼 나감 (종단 시험에서 발견) |
| arm 중립 90 µs > 구동 데드존 70 µs | 구동 데드존 100, 검사로 강제 | 70~90 µs 위치에서 arm하면 **arm 즉시 바퀴가 돎** |
| RC↔AUTO가 0.3 s 안에 튀면 arm 유지 | CH1이 바뀌면 즉시 disarm | 재arm 없이 다시 달림 |
| `/control_mode/set STOP`이 다음 주기에 풀림 | 진짜 래치(R로만 해제) | 'latched'라는 로그와 실제 동작이 달랐음 |
| `rc_override_*` 선언만 하고 안 씀 | AUTO 중 CH2 조작 → 정지 | 조종자가 트리거로 AUTO를 멈출 수 있게 |
| dry에서 판단 결과가 안 보임 | `would_*` 보고 | dry로 극성·arm을 확인할 수 있게 |
| 조향 데드존 경계에서 0 → 6° 점프 | 데드존 뒤를 다시 펴서 연속 | 다이얼 작은 조작이 튀지 않게 |
| PPM 노드: 끊김 로그 한 줄에서 info/warn 전환 | 호출 위치 분리 | rclpy "Logger severity cannot be changed" → **첫 끊김에 노드가 죽음** (종단 시험에서 발견) |
| PPM 노드: gpiomon이 죽으면 그대로 | 1 s 뒤 재시작, 5 s마다 프레임 통계 | |
| run 스크립트: 경로 고정, 인자로 dry → 실구동 가능 | 스크립트 위치 기준, 안전 인자 덮기 거부, 단계별 실구동 | |
| 기존 `radxa.py` 종료 시 `'Event' object is not callable` | `SuspensionAxis._stop` → `_stop_evt` | `threading.Thread._stop`을 덮어써 `join()`이 TypeError. 그 결과 유지 전류 설정·로그 닫기·포트 닫기가 **매번 건너뛰어짐**. 원본 `daron_run/rover/radxa.py`에도 같은 버그가 있었고 **261017 리뷰에서 원본도 수정됨** |

## 10. 남은 한계

- ROS 토픽 STOP은 물리 E-stop이 아닙니다. 프로세스 정지, R8FM Failsafe hold, Linux 지터, VESC 설정에 따라 예상과 다르게 움직일 수 있습니다.
- R8FM Failsafe가 값을 붙드는 방식이면 송신기를 꺼도 소프트웨어가 모릅니다(2절, 1단계 확인).
- AUTO 중 지상국 형상 명령(`/rover/command` neutral/flat/stow)은 막지 않습니다. 형상 전환 중에는 구동이 원래대로 막힙니다.
- CH2는 방향 지령이라 트리거를 살짝 당겨도 VESC 설정 속도까지 가속합니다.
- 원본 콘솔과 같은 폰트를 써서, 한글 안내 줄은 시스템에 따라 □로 보일 수 있습니다. RC 패널은 영문만 씁니다.
