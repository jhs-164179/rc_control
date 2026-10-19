# 261011 변경 요약 — 루프 클로저 때 그리드 피쳐맵 재동기화

배경: 교수님 질문(루프클로징 시 월드맵·위치·웨이포인트가 같이 보정되나). 지금까지는 로버 위치(TF map→odom)만 보정되고
피쳐맵 칸은 쌓을 당시 위치에 남아 둘이 어긋났다. 이 변경으로 루프 클로저가 나면 피쳐맵 전체를 cuVSLAM 이 보정한
과거 궤적으로 다시 쌓는다. 판정 격자·플래너는 피쳐맵을 다시 읽으므로 따라온다.

백업: `backup/pre_lcsync_261011/` (변경 전 launch·nodes·config·rviz·tools·rover 전체, `MD5SUMS.txt`),
      `backup/pre_lcsync2_261011/` (1단계 후·2단계 전 launch·nodes·config).

**2단계(같은 날 밤, 사용자 결정)**: 로버 위치(TF)는 그대로 두고, **지도에 맞춰 크레이터 핀·이번 세션 웨이포인트도 같이 옮긴다** (아래 '2단계' 절).
되돌리기: 실행 인자 `lc_resync:=false`(daron_viz·daron_3dmap 둘 다) = 옛 동작. 파일째 되돌리려면 백업 폴더에서 복사.

## 확인한 사실 (trav_obstacle bag, 오프라인 `tools/slam_offline.py --rig cv`)
- 루프 클로저 8회 동안 실시간 포즈(`Slam.track`) 보정 = 0 → TF map→odom 도 안 바뀜. cuVSLAM 이 1 % 미만 보정은 실시간 포즈에 안 넣는다.
- `Slam.get_all_slam_poses()` 는 2762/2762 전 프레임의 PGO 반영 포즈를 준다(`max_map_size` 300 과 무관). 실시간 포즈와 최대 2.1 cm·0.16°.

## 동작
1. cuVSLAM 노드(`nodes/d405_cuvslam_viz.py`, `lc_history:=true`)
   - 프레임마다 (stamp, VO odom←base, 그때 낸 map←odom) 기록. 재추정 확정 전(PENDING/CONFIRMING) 프레임은 기록 안 함, REJECTED 면 비움.
   - 새 루프 클로저가 보이면(1 s 마다 확인, 최소 간격 2 s) `get_all_slam_poses()` 호출(트래킹 스레드, 실측 1 ms) →
     별도 스레드에서 프레임마다 `M_corr(t) = Slam_all(t) · rig_from_base · VO(t)⁻¹` 계산 → `/slam/map_odom_history` 발행.
   - 그때 낸 map←odom 과 차이가 0.2 cm·0.05° 미만이면 안 냄. **TF map→odom·/slam/odometry·EKF 입력은 그대로.**
2. 피쳐맵 노드(`nodes/3dmap/live_tsdf_node.py`, `lc_resync:=true`, grid + map 좌표일 때만)
   - 키프레임마다 원재료 보관: 솎은 원본 depth(stride 2, uint16, zlib) + 적분 포즈 + 그때 map←odom. 상한 `lc_store_max_mb` 1500.
   - 이력을 받으면 백그라운드에서 `T_wc_new = M_corr(t) · M_used⁻¹ · T_wc_used` 로 처음부터(원래 적분 순서) 다시 쌓고,
     재적분 중 새로 들어온 키프레임은 잠금 안에서 이어 쌓은 뒤 통째로 교체. 이전 세션에서 불러온 피쳐맵은 원재료가 없어 바탕으로 둠.
   - 저장되는 `*_traj_tum.txt`·`*_featuregrid.npz` 도 보정된 값.
   - 계산은 `nodes/3dmap/kf_resync.py` (ROS 없음).

## 기존 동작과 달라진 것

| # | 무엇 | 이전 | 이후 |
|---|------|------|------|
| 1 | 피쳐맵 칸 (값·위치) | 쌓을 당시 위치 그대로 | 루프 클로저마다 보정 궤적으로 다시 쌓음 |
| 2 | 판정 격자·플래너 지도 | 피쳐맵과 같음 (옛 칸) | 피쳐맵과 같음 (보정 칸, 다음 판정 주기 2 s 안) |
| 3 | 새 토픽 | — | `/slam/map_odom_history` (nav_msgs/Path, TRANSIENT_LOCAL, 루프 클로저 때만) |
| 4 | live_tsdf 메모리 | 칸 배열만 | + 키프레임 원재료 (실측 약 21 KB/장) |
| 5 | live_tsdf 로그 | — | `[LC 재동기화 #n] 키프레임 … 최대 이동 … 재적분 … ms` / 5 s 통계에 `LC 재동기화 N회 원재료 …` |
| 6 | cuVSLAM 로그 | — | `[lc_history #n] 루프 클로저 … 실시간 대비 최대 … get_all_slam_poses … ms` |
| 7 | 녹화 목록·재생 제외 | — | `/slam/map_odom_history` 추가 |

**바뀌지 않는 것**: EKF(odom). (로버 TF 는 3단계에서 보정 궤적을 따르게 바뀜 — 아래 3단계)
(웨이포인트·핀은 2단계에서 지도와 같이 옮기게 바뀜)

## 2단계 — 크레이터 핀 · 웨이포인트도 지도에 맞춤

같은 `/slam/map_odom_history`, 같은 식(`p_new = M_corr(t) · M_ref⁻¹ · p`, `kf_resync.correct_point`). 기준 시각 t 와 그때 지도가 쓰던 map←odom(M_ref)을 항목마다 들고 있다가 이력이 오면 옮기고 기준을 갱신한다.

| 대상 | 기준 시각 t | 무엇이 바뀌나 |
|---|---|---|
| 크레이터 핀 (`cda/pin_node.py`, `lib/pins.py`) | 검출 stamp (검출마다) | 핀을 **map 좌표**로 낸다(`map_frame`, 이전엔 odom). 검출마다 위치·기준을 보관(`PinSet._recs`), 이력이 오면 검출을 옮기고 핀 중심 다시 계산(`PinSet.reposition`). 핀 id·소속·반지름·횟수·top 그대로 → 지상국 이미지·크레이터 계산 안 깨짐. 지난 핀·지금 쌓이는 핀 모두 |
| 웨이포인트 — 지도 클릭(rviz)·이번 세션 다시 넣기(session) | 그 자리 지도 칸의 마지막 관측 시각 (`/live_tsdf/map_cells` 새 필드 `age`) | 이력이 오면 위치·방위를 옮기고 `waypoints.jsonl` 에 `moved` 사건(from/to/dpos/dyaw). `reconstruct` 가 반영. **플래너에 보낸 지금 목표가 옮겨지면 CLEAR 후 새 좌표로 다시 보냄** (`WaypointSeq.relocate`) |
| 웨이포인트 — 영상 클릭(image) | 클릭한 영상 stamp | 위와 같음 |
| 웨이포인트 — 이전 세션 불러온 것(loaded:) | — | 안 옮김 (불러온 바탕 지도도 원재료가 없어 안 옮겨지므로 같이 그대로) |
| 지상국 로버 위치 (거리 열·마커·핀 정보 창 거리) | — | `/odometry/filtered_health`(odom) 대신 **TF waypoint_frame←base_link**(map) — 핀·웨이포인트와 같은 좌표. TF 없으면 옛 동작 |

- `added` 기록에 `anchor {t_ns, src: cell|image|now}`. 칸을 못 찾으면 지금 시각.
- `/live_tsdf/map_cells` 끝 필드 `age` = 칸 마지막 관측 stamp − header.stamp [s]. 판정 노드·mount_check 는 필드 이름으로 읽어 영향 없음.
- 인자: `daron_gui.launch.py lc_resync:=true`(기본) → 지상국 파라미터. pin_node 는 daron_3dmap 의 `map_frame`·`lc_resync` 를 받는다.
- ⚠ 동작 변경: `map_frame:=map`(기본)이면 `lc_resync:=false` 여도 **핀 좌표계가 map** (이전 odom). Slam 없을 때 map→odom 은 항등이라 숫자는 같다.

## 남은 차이 (알고 넣은 것)
- 로버 위치는 `Slam.track` 기반 그대로(사용자 결정). 1 % 미만 보정만큼(이 bag 실측 최대 1.4 cm·0.35°) 재동기화된 지도·핀·웨이포인트와 다를 수 있다.
- 웨이포인트 기준 시각은 칸의 '마지막' 관측 시각 근사 — 여러 시점에 본 칸은 보정량이 시점마다 달라 정확히 한 값이 아니다.
- HOLD 해제 프레임(health ANCHOR)은 원래 코드처럼 map←odom 을 곱하지 않고 쌓인다(M_used = I 로 보관 → 재동기화도 같은 기준).

## 검증
- 단위: `tests/test_lc_resync.py` 6개 — 보정 항등이면 실시간 피쳐맵과 **칸 배열이 비트 단위로 같음**, 평행이동·회전 보정 시 처음부터 보정 포즈로 쌓은 맵과 같음,
  이력 일부 누락, 두 번 연속 보정(덮어쓰기), 불러온 바탕 맵 유지, 저장 상한. 기존 시험 전부 통과(feature_grid 8, health 13, image_click 8, rim 6, rover_teleop 5, waypoint_log 23, waypoint_seq 9).
- 재생 (trav_obstacle 98 s, 도메인 91, sim time, `rebuild:=true`):
  - 켬: `lc_history` 10회(get_all_slam_poses 1 ms + 변환 72~125 ms, 별도 스레드), 재동기화 9회 + 건너뜀 1회,
    키프레임 최대 이동 0.09 → 1.13 cm / 0.07 → 0.35°, 재적분 233~397 ms(키프레임 78~137장) + 교체 0~3 ms, 원재료 137장 2.9 MB.
  - 끔: 이력 발행 0, 옛 동작.
  - 같은 stamp 키프레임 100개 위치 차(켬 − 끔): 중앙 0.68 cm, 최대 0.94 cm. 칸 프레임간 높이 표준편차 p90 1.86 → 1.81 cm (두 실행 타이밍이 달라 참고값).
- [계산] 20분 주행(키프레임 1~2.4 장/s ≈ 2400장): 원재료 약 50 MB, 재적분 약 7 s(백그라운드), 이력 메시지 3.6만 프레임 변환 약 2 s(별도 스레드).
- **미검증**: 실물 실시간(원격 WiFi), 큰 보정(수 cm 이상)이 나는 주행, 긴 주행에서의 재적분 시간·메모리 실측, RViz 화면 확인.

## 2단계 검증
- 단위 `tests/test_lc_resync.py` 10개(+4: 이력 메시지 왕복·점 보정, 핀 reposition = id 유지·중심 이동·같은 이력 반복 시 안 움직임, 웨이포인트 relocate → CLEAR → 새 좌표 재전송, reconstruct 의 moved). 기존 시험·GUI 스모크(ALL PASS) 통과.
- 재생 (trav_obstacle + 합성 검출 CSV 277장 → fake 검출기, 지상국 offscreen, 웨이포인트 2개 지도 클릭, 도메인 92):
  이력 8회 → 피쳐맵 재동기화 6회(최대 0.87 cm)·건너뜀 2, **핀 16개 8회 다시 계산(핀 중심 최대 0.63 cm, 좌표계 map)**,
  웨이포인트 `anchor src=cell` 2개, `moved` 1회(0.03 cm — 출발 직후 본 칸이라 보정 작음). 끔(`lc_resync:=false`): 이력 0, anchor 없음, 핀 map 좌표 정상, 오류 없음.

## 파일·md5 (2단계 이후 최종)
```
42228edd241d4a755c53c2c23dfecefc  nodes/d405_cuvslam_viz.py
372f74ec5057be1d33998dfe51eaf87a  nodes/3dmap/live_tsdf_node.py
6e78403b36fc7fd9a0a69635ea291e5d  nodes/3dmap/kf_resync.py
347a908211d665caf80593d462a2eac6  nodes/cda/pin_node.py
f3f7c2dcc383d0c9ce7f4186111171af  nodes/lib/pins.py
46b756787e28debe000d6b5e990f8857  nodes/gui/ground_station.py
76cdc03f3156247ef9693d6c2510ada7  nodes/gui/waypoint_seq.py
372d857ca374388a6055a111d0718a53  nodes/tests/test_lc_resync.py
c8348a8fa9c9856ec61ec25f09a860e9  launch/daron_viz.launch.py
874b8af82142a3edd24dff058de0479c  launch/daron_3dmap.launch.py
80d258955a9a58fc05b2d7e4ef1a5e53  launch/daron_gui.launch.py
```

## 파일·md5 (1단계 직후)
```
42228edd241d4a755c53c2c23dfecefc  nodes/d405_cuvslam_viz.py          (이전 b0a80f4f…)
cb0cdfcefa415e484334aab1d5889b56  nodes/3dmap/live_tsdf_node.py      (이전 b1b6f91b…)
ae28772d07eef11e63a7f92baf6ce3f6  nodes/3dmap/kf_resync.py           (새)
0752ed8d773895cb5d676ee7f60414e0  nodes/tests/test_lc_resync.py      (새)
c8348a8fa9c9856ec61ec25f09a860e9  launch/daron_viz.launch.py         (이전 14a6293e…)
cfc3a71fad434f60212e9a606e92b11b  launch/daron_3dmap.launch.py       (이전 17ebe2aa…)
80d94728a472f96ae77bdb7199282460  config/record_topics_viz.txt
7e84337a21276daae97f104aa2bf4615  config/record_topics_3dmap.txt
```

## 3단계 (같은 밤, 사용자 결정) — 로버 위치도 보정 궤적을 따름 + 이전 세션 크레이터 핀 불러오기

백업: `backup/pre_lcsync3_261011/` (2단계 후·3단계 전 launch·nodes·config).
결정 배경: '현재 로버 기준'(로버는 그대로, 과거 지도를 옮김)은 이전 세션 지도·저장 맵과 원점이 어긋나서, **출발점(이전 지도) 기준을 유지하고 로버 위치를 옮기는 안**으로 확정.

| 항목 | 내용 | 파일 |
|---|---|---|
| 로버 TF (`lc_robot:=true` 기본, daron_viz) | 이력을 낼 때 마지막 프레임 오프셋 `Off = M_corr(t_last)·M_raw(t_last)⁻¹` → 이후 TF map→odom = `Off·M_raw`. 루프 클로저 순간 로버가 보정량만큼 옮겨져 지도·핀·웨이포인트와 맞음. Slam.track 이 PGO 를 직접 반영해 M_raw 가 2 mm·0.1° 넘게 바뀌면 Off = I + 이력 다시(이중 적용 방지). 계산 사이 track 이 바뀌면 그 Off 버림 | `nodes/d405_cuvslam_viz.py` |
| 재추정 대기 (pin_node) | map 좌표 핀은 재추정 확정 전 검출을 안 씀(`skip_reloc_wait`) — live_tsdf 와 같은 규칙 | `nodes/cda/pin_node.py` |
| 이전 세션 핀 (`pins_load:=auto` 기본, daron_3dmap) | ACCEPTED 면 이전 세션 폴더 최신 `pins_*.json`(frame map·map_id 일치)을 `PinSet.seed` 로 미리 넣음 → 새 검출이 같은 판정으로 합쳐짐. 불러온 핀은 top·cands 비움(이번 세션 첫 검출이 대표 이미지), 출처는 `loaded`. 저장 json 에 `map_id`·`loaded_from` | `nodes/cda/pin_node.py`, `nodes/lib/pins.py`, `launch/daron_3dmap.launch.py` |

검증
- 단위 `test_lc_resync.py` 11개(+1: seed 한 핀에 재관측이 합쳐짐·먼 곳은 새 핀·seed 는 reposition 대상 아님). 기존 시험·GUI 스모크 ALL PASS.
- 비교 실험 보고서: `~/Daron/루프클로징_동기화_실험보고_261011.html` (https://.ai/artifact/V95FGW7uFEcxAjk9yLTHgD, 비공개).
  - 실제 bag 한 세션(기존 = `backup/pre_lcsync_261011` 코드): 로버 TF 보정 기존 0회 / 현재 9회(한 번 최대 0.73 cm), 키프레임 위치 차 중앙 0.20 · 최대 0.94 cm. 지도 높이 흔들림 차이 없음(보정 < 1 cm).
  - 두 세션(같은 bag 두 번, 재추정): 기존 2회차 확정 핀 11개 새로 생김(11개 모두 1회차와 10 cm 안 = 같은 크레이터 중복) / 현재 불러온 16개 모두 재관측으로 합쳐짐, 새 핀 0.
  - 합성 큰 드리프트(13.4 cm, 이상적 보정 가정): 장애물 칸 IoU 0.69 → 1.00, 가짜 장애물 칸 27 → 0, 귀환 로버 오차 13.4 → 0 cm, 웨이포인트 도착 오차 8.9 → 0 cm.
- md5 (3단계 이후): `d405_cuvslam_viz.py dc460f09…`, `cda/pin_node.py ebc9dbd3…`, `lib/pins.py cf509528…`, `tests/test_lc_resync.py 1c6c2152…`, `launch/daron_viz.launch.py 9c697358…`, `launch/daron_3dmap.launch.py fa6fd100…`
