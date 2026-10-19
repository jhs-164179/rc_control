# 캘리브레이션

로버를 기동하기 전에 **RADXA 보드에서 로컬로** 한 번 돌린다. ROS 도 DDS 도
지상국도 필요 없다. 결과물은 `profiles/rover.json` 하나이고, `radxa.py` 가
그 파일을 그대로 읽는다 (옮겨 적을 값은 없다).

## 전부 두 줄이다

```bash
sudo sh setup_latency.sh                                  # 처음 한 번만
python3 calibrate.py --auto-zero --out profiles/rover.json
```

`--auto-zero` 는 굴절 원점(스토퍼 중점)을 먼저 재서 `home_deg` 로 넣는다.
원점을 이미 알고 있고 결착을 바꾸지 않았으면 `--zero 22.5` 로 대신한다.

확인은 한 줄 더:

```bash
python3 run_suspension.py --profile profiles/rover.json   # 눌렀다 놓아 본다
```

**준비**: 로버를 주행 자세로 바닥에 내려놓고 차체 하중이 굴절축에 실린 상태.
들어 올린 채로 재면 전부 무의미하다. 측정 중에는 축을 건드리지 않는다
(`run_suspension.py` 에서는 눌러 봐야 한다).

`setup_latency.sh` 를 안 돌리면 FTDI latency 16 ms 때문에 제어 루프가 60 Hz
근처에서 막히고, 외력 '해제' 검출이 한 박자 늦어 복원이 굼떠 보인다.

---

## 0) 굴절 원점 (`--auto-zero`, 또는 `find_zero.py`)

엔코더 각도 중 **굴절각 180° (평면 전개)** 에 해당하는 값을 찾는다.
`calibrate.py --auto-zero` 가 부르는 것과 같은 루틴이다. 따로 확인하고
싶을 때만 직접 쓴다.

```bash
python3 find_zero.py limits          # 양쪽 스토퍼 중점 (--auto-zero 와 동일)
python3 find_zero.py check           # 후보로 이동해 눈으로 확인
python3 find_zero.py hand            # 토크 끄고 손으로 맞춘 뒤 읽기
```

`limits` 출력의 **편측 행정**이 45°와 3° 안쪽이면 믿는다. `⚠ 불일치`가 뜨면
한쪽 스토퍼에 못 닿은 것이니 `--probe-current 100` 으로 올려 다시 잰다.

이 값은 프로파일의 `home_deg` 가 되고, `radxa.py` 는 그것을 굴절 원점
(`articulation_zero_deg`) 으로 그대로 쓴다. 두 이름은 같은 값이다.

## 1~3) `calibrate.py` — 본 캘리브레이션 (약 5 분)

```bash
python3 calibrate.py --auto-zero --out profiles/rover.json
```

세 가지를 자동으로 한다.

| 단계 | 하는 일 | 정하는 값 |
|------|---------|-----------|
| 1 유지 전류 | 전류를 내려가며 차체가 주저앉는 지점을 찾고 1.8 배 여유 (`--hold-margin`, 261015 전 1.5) | `current_holding`, `current_comply`, `settled_band_deg`, `comply_band_deg` |
| 2 비대칭 측정 | 모터가 스스로 ±12° 변위를 만들고 복원시켜 `R` 측정 | — |
| 3 비대칭 보정 | `R` 이 5 % 밖일 때만. 전류/궤적 중 병목인 쪽을 알아서 고름 | `b_pos`/`b_neg` 또는 `profile_velocity_pos/neg_dps` |

끝나면 바뀐 값 목록과 함께 프로파일이 저장된다.

**`R` 이 무엇인가** — `t90(+CCW) / t90(-CW)`. 양방향 복원 시간의 비다.
`1.0` 이면 좌우 대칭, `1.2` 면 +CCW 쪽이 20 % 느리다는 뜻.

자주 쓰는 옵션:

```bash
--zero 22.5           # 원점을 이미 알 때 (--auto-zero 대신)
--skip-holding        # 유지 전류를 이미 알 때 1단계 건너뛰기
--amp 20              # 시험 변위를 키운다 (작은 변위로 R 이 안 잡힐 때)
--reps 6              # 반복을 늘려 잡음을 줄인다
--home-here           # 원점을 모를 때, 지금 자세를 기준으로
--yes                 # 확인 질문 건너뛰기
```

## 4) `run_suspension.py` — 실제로 써 보기

```bash
python3 run_suspension.py --profile profiles/rover.json
```

실험 장치가 전혀 없는 순수 운용 루프다. 축을 눌렀다 놓으면 복원 이벤트가
한 줄씩 찍힌다. 양방향을 비슷하게 5 회씩 눌러 보고, `t90` 이 좌우 비슷하면
캘리브레이션이 맞은 것이다.

종료할 때 기본값은 **토크 유지**다. Ctrl-C 로 빠져나가도 차체가 내려앉지
않는다. 정말로 힘을 빼려면 `--on-exit off`.

---

## 결과 읽는 법

| 지표 | 뜻 | 이상 신호 |
|------|-----|-----------|
| `t90` | 복원량의 90 % 도달 시간 = 복원 속도 | — |
| `정착` | 최종 평형에 자리 잡은 시각 | `t90` 의 2 배 넘으면 진동 |
| `OS` | 오버슈트 [%] | 25 % 넘으면 `d_gain`↑ |
| `진동` | 평형선 교차 횟수 | 2 이상이면 감쇠 부족 |
| `잔류` | 도달한 평형점의 오차 = 정적 처짐 | — |
| `Ipk` | 복원 중 최대 전류 | 상한의 80 % 미만이면 전류가 병목이 아님 |

`결과` 열: `settled`(밴드 안 완료) · `residual`(평형 도달, 처짐 남음 — 정상) ·
`timeout`(계속 눌려 있음) · `reimpact`(복원 중 새 충격).

## 가동 범위에 대해

`travel_limit_deg`(기본 35°)는 **프로그램이 그 밖을 지령하지 않는다**는 뜻이지
**그 밖으로 나가면 멈춘다**는 뜻이 아니다. 서스펜션은 눌리는 것이 일이므로
큰 충격에 경계를 넘는 일은 정상이고, 그때 실행을 멈추면 오히려 토크가 풀려
차체가 주저앉는다. 범위 밖 목표는 경계로 잘리고 경고만 찍힌다.

## 문제 생겼을 때

| 증상 | 조치 |
|------|------|
| `실제 루프 NN Hz < 설정` | `sudo sh setup_latency.sh` |
| 1단계에서 바로 처짐 | `--sag-start 120` |
| 1단계에서 끝까지 안 무너짐 | `--sag-end 1` |
| `t90 을 못 구했습니다` | `--amp 20` 으로 변위를 키운다 |
| `x 를 바꿔도 R 이 거의 안 변합니다` | 정상이다. 다음 모드로 자동 전환된다 |
| 복원이 `timeout` 만 남김 | 축이 계속 눌려 있거나 `current_recover` 가 하중을 못 이김 |
| `⚠ 설정이 모순됩니다` | 출력된 내용대로 `motor_control/xm430/config.py --set` 으로 고친다 |

## 더 파고들 때 (선택)

```bash
# 비대칭의 '원인'을 알고 싶다 — 중력인가 마찰인가 기구인가
python3 asymmetry.py breakaway --angles -15,0,15 --i-fall <1단계 붕괴 전류>

# 게인을 더 조이고 싶다
python3 experiments.py sweep --profile profiles/rover.json \
    --grid p_gain=80,120,160 --grid d_gain=400,600,800

# 손으로 만지며 실시간 튜닝
python3 susp_console.py --profile profiles/rover.json

# 지난 로그 다시 보기
python3 analyze.py summary "logs/run_*.csv"
```

개별 도구의 전체 옵션은 각 파일의 `-h` 또는 `README.md` 5절 참고.

## 로버에 반영

**옮겨 적을 것이 없다.** `radxa.py` 가 `profiles/rover.json` 을 그대로 읽고,
같은 `susp_controller.SuspensionController` 를 100 Hz 전용 스레드에서 돌린다.
방향 보정(`b_pos`/`b_neg`)과 `COMPLY`/`YIELD` 도 그대로 살아 있다.

```bash
python3 radxa.py                                          # 기본 프로파일
python3 radxa.py --ros-args -p susp_profile:=profiles/firm.json
```

기동 중에 값을 다시 만지고 싶으면 ROS 파라미터로 바꾼다 (이름이 같다):

```bash
ros2 param set /rover current_holding 60
ros2 param set /rover p_gain 120
ros2 param set /rover d_gain 600
```

단, 이 변경은 **휘발성**이다. 마음에 들면 프로파일에도 반영해 둔다:

```bash
python3 -m motor_control.xm430.config --profile profiles/rover.json \
        --set current_holding=60 --save profiles/rover.json
```

예외는 `travel_limit_deg` 하나다. 로버에서 굴절 범위를 정하는 것은 기계
한계(`jog_limit_deg`, 기본 45°)이므로 `radxa.py` 는 프로파일의
`travel_limit_deg` 를 쓰지 않는다. 좁히려면 `-p jog_limit_deg:=30`.
