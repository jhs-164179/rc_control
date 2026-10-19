#!/usr/bin/env python3
"""DARON 3D 매핑 + 주행가능성 격자 + RViz.

daron_bringup.launch.py 와 **따로** 띄운다. bringup(카메라/cuVSLAM/EKF)이 이미 돌고
있는 상태에서 이 launch 를 추가로 실행하는 구조다. bringup 을 건드리지 않으므로
매핑이 죽어도 SLAM 녹화는 영향을 받지 않는다.

    터미널 A:  source ~/slam_env.sh && ros2 launch daron_bringup.launch.py ...
    터미널 B:  source ~/slam_env.sh && ros2 launch ~/daron_ros/launch/daron_3dmap.launch.py

⚠ 반드시 `source ~/slam_env.sh` 후에 실행할 것. ROS_DOMAIN_ID=42 와 PYTHONPATH 가
  여기서 온다. 도메인이 다르면 토픽이 조용히 안 잡힌다(에러가 안 난다).

인터프리터: /home/pakr/venv/daron3d/bin/python (open3d 0.19.0 + scipy).
  cuvslam venv 와 분리돼 있다 — pycuvslam 쪽에 영향을 주지 않기 위해서다.
  Node() 대신 ExecuteProcess 를 쓰는 이유는 인터프리터를 고를 수단이 없기 때문
  (daron_bringup.launch.py 의 cuVSLAM 과 같은 이유).
"""
import math
import os
import subprocess
import time

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

HOME = os.path.expanduser('~')
VENV_PY = os.path.join(HOME, 'venv/daron3d/bin/python')
# [daron_run 260929] 이 launch 파일 기준 상대경로 (~/daron_run/{nodes/3dmap,config,rviz}).
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
MAP_DIR = os.path.join(ROOT, 'nodes', '3dmap')
LIVE_TSDF = os.path.join(MAP_DIR, 'live_tsdf_node.py')
TRAV = os.path.join(MAP_DIR, 'traversability_node.py')
DEPTH_VIZ = os.path.join(MAP_DIR, 'depth_colorize.py')
# [261001] CDA 핀 (검출 bbox -> 맵 핀). 시스템 python (open3d 불필요) — nodes/lib/pins.py
CDA_DIR = os.path.join(ROOT, 'nodes', 'cda')
PIN_NODE = os.path.join(CDA_DIR, 'pin_node.py')
FAKE_DET = os.path.join(CDA_DIR, 'fake_detector.py')
CDA_OVERLAY = os.path.join(CDA_DIR, 'cda_overlay.py')   # [261005] 검출 bbox 를 infra1 위에 그려 /cda/overlay (보기 전용)
SYS_PY = '/usr/bin/python3'
TRAV_PARAMS = os.path.join(ROOT, 'config', 'traversability.yaml')
RVIZ_CFG = os.path.join(ROOT, 'rviz', 'daron_3dmap.rviz')
# [260929] bag/맵 폴더는 daron_run 옆 (~/Daron/{daron_bags,maps})
DATA_ROOT = os.path.dirname(ROOT)
# [261006] 경로계획 노드 (daron-nav 레포, 경로계획 채팅 담당). 기본 꺼짐(planner:=false).
#   판정 격자(/traversability/grid)·health·TF odom->base_link 를 받아 /planner/path 등 발행. 같은 daron3d venv.
#   경로계획만 다시 띄우고 싶으면 planner:=false 로 두고 터미널을 따로 열어 planner_node.py 를 직접 실행.
#   [261012] 이 launch 파일 기준 상대경로 (Daron/ 옆의 daron-nav/). 저장소에서는 <저장소>/daron-nav,
#   옛 배치(~/Daron/daron_run)에서는 그대로 ~/daron-nav 가 된다.
PLANNER_DIR = os.path.join(os.path.dirname(DATA_ROOT), 'daron-nav', 'daron_planner', 'ros')
PLANNER = os.path.join(PLANNER_DIR, 'planner_node.py')
# [261011] 로버 기준점 중계 (daron-nav): TF <map|odom>->base_link -> /localization/pose (두 축 가운데 점, 20 Hz).
#   현성님 추종기·제어창 도착 기록이 구독. planner:=true 일 때 같은 좌표·같은 시계로 같이 띄운다 (pose_relay:=false 면 끔).
POSE_RELAY = os.path.join(PLANNER_DIR, 'pose_relay.py')


# camera_link -> camera_depth_optical_frame. 260928 tf2_echo 실측값:
#   Translation [0, 0, 0] / Quaternion (xyzw) [-0.500, 0.500, -0.500, 0.500]
# D405 는 depth 를 좌측 IR 위에서 계산하므로 camera_link(= 좌 이미저)와 원점이 같고,
# 회전은 REP-103 광학 프레임 변환뿐이다 (librealsense extrinsics 로도 t=0, R=I 확인).
CAM_DEPTH_TF = ['--x', '0', '--y', '0', '--z', '0',
                '--qx', '-0.5', '--qy', '0.5', '--qz', '-0.5', '--qw', '0.5',
                '--frame-id', 'camera_link',
                '--child-frame-id', 'camera_depth_optical_frame']


def _double_arg(name, text):
    """[261017] DOUBLE 로 정적 선언된 노드 파라미터(live_tsdf dmax/voxel…, pin_node dmin/dmax, depth_colorize)에
    'dmax:=1' 같은 정수 모양을 넘기면 INTEGER 가 돼 InvalidParameterTypeException 으로 기동 즉시 죽는다.
    daron_viz._double_arg 와 같은 처리 — 항상 소수점이 든 문자열로 돌려준다."""
    try:
        v = float(text)
    except (TypeError, ValueError):
        raise RuntimeError(f'{name} 인자는 숫자여야 합니다 (받은 값: {text!r})')
    if not math.isfinite(v):
        raise RuntimeError(f'{name} 인자가 유한한 숫자가 아닙니다: {text!r}')
    out = repr(v)                       # 0.07 / 1.0 / 1e-05
    if 'e' in out or 'E' in out:        # YAML 1.1 은 '1e-05' 를 float 로 안 읽는다 — 자릿수를 펼쳐 쓴다
        out = f'{v:.12f}'.rstrip('0')
        if out.endswith('.'):
            out += '0'
    return out


def _int_arg(name, text):
    """[261017] INTEGER 로 선언된 파라미터(every, min_count)에 '3.0' 을 넘기면 반대로 DOUBLE 이 돼 죽는다."""
    try:
        v = float(text)
    except (TypeError, ValueError):
        raise RuntimeError(f'{name} 인자는 정수여야 합니다 (받은 값: {text!r})')
    if not v.is_integer():
        raise RuntimeError(f'{name} 인자는 정수여야 합니다 (받은 값: {text!r})')
    return str(int(v))


def _camera_node_running():
    """realsense2_camera 노드가 떠 있는지. bag 재생 모드면 없다."""
    try:
        return subprocess.run(['pgrep', '-f', 'realsense2_camera_node'],
                              capture_output=True).returncode == 0
    except Exception:
        return False


def _preflight():
    if not os.path.exists(VENV_PY):
        raise RuntimeError(
            f'매핑용 인터프리터가 없습니다: {VENV_PY}\n'
            f'  /usr/bin/python3 -m venv ~/venv/daron3d && '
            f'~/venv/daron3d/bin/pip install "open3d==0.19.0" rosbags scipy')
    for f in (LIVE_TSDF, TRAV, DEPTH_VIZ, TRAV_PARAMS, RVIZ_CFG, PIN_NODE, FAKE_DET, CDA_OVERLAY):
        if not os.path.exists(f):
            raise RuntimeError(f'파일이 없습니다: {f}')
    if 'ros' not in os.environ.get('PYTHONPATH', ''):
        raise RuntimeError('PYTHONPATH 에 ROS 가 없습니다. `source ~/slam_env.sh` 후 다시 실행하세요.')


def _resolve_out_dir(cfg):
    """맵 파일 저장 위치.
    session_name 을 주면 daron_viz 녹화 폴더(bag_dir/session_name)에 bag 과 같이 저장한다.
    폴더는 **여기서 만들지 않는다** — daron_viz 는 세션 폴더가 이미 있으면 녹화를 거부하므로,
    3dmap 이 먼저 만들면 녹화가 막힌다. 저장은 종료 시점이라 그때는 녹화 폴더가 있다.
    녹화 폴더가 없으면(record:=false 등) live_tsdf 가 저장 시 만든다 — 그 이름으론 다시 녹화 못 한다.
    """
    session = cfg('session_name').strip()
    if not session:
        return os.path.expanduser(cfg('out_dir')), None
    d = os.path.join(os.path.abspath(os.path.expanduser(cfg('bag_dir'))), session)
    if os.path.exists(os.path.join(d, 'metadata.yaml')):
        note = f'⚠ {d} 는 이미 끝난 녹화 폴더입니다. 맵 파일이 거기에 추가됩니다.'
    elif os.path.exists(d):
        note = f'녹화 중인 세션 폴더 {d} 에 저장합니다 (종료 시).'
    else:
        note = (f'{d} 가 아직 없습니다. daron_viz record:=true session_name:={session} 가 만들 것으로 보고 '
                f'종료 시 거기에 저장합니다. 녹화를 안 하면 이 폴더가 새로 생기고 같은 이름으로는 녹화가 막힙니다.')
    return d, note


def _setup(context, *_, **__):
    _preflight()
    cfg = lambda n: LaunchConfiguration(n).perform(context)  # noqa: E731
    sim = cfg('use_sim_time').lower() in ('true', '1')
    # [261017] health:=false — daron_viz 를 health:=false 로 띄우면 /health/state·/slam/odometry_h·/odometry/filtered_health 가
    #   전부 없다. 전에는 odom_topic 만 바꾸라고 안내했지만 live_tsdf 는 slam_topic(/slam/odometry_h) 미수신으로 모든 프레임을
    #   skip_slam_range 로 버리고, traversability 는 require_health(기본 true) 게이트(health_none)에 걸려 격자를 아예 안 냈다.
    #   여기서 odom_topic / health_topic / slam_topic / require_health 를 한 번에 맞춘다.
    health_on = cfg('health').lower() in ('true', '1')
    odom_topic = cfg('odom_topic').strip()
    health_topic = cfg('health_topic').strip() if health_on else ''
    slam_topic = cfg('slam_topic').strip()
    if not health_on:
        if odom_topic == '/odometry/filtered_health':   # health 꺼진 daron_viz 에는 EKF-C 가 없다
            odom_topic = '/slam/odometry'
        if slam_topic == '/slam/odometry_h':            # health_monitor 출력 → 원시 SLAM 으로
            slam_topic = odom_topic
    dbl = _double_arg
    out_dir, note = _resolve_out_dir(cfg)
    save_on_exit = cfg('save_on_exit').lower() in ('true', '1')
    acts = [LogInfo(msg=f'[3dmap] interpreter={VENV_PY}  use_sim_time={sim}  '
                        f'DOMAIN={os.environ.get("ROS_DOMAIN_ID", "0")}  '
                        f'RMW={os.environ.get("RMW_IMPLEMENTATION", "(default)")}'),
            LogInfo(msg=f'[3dmap] 맵 파일: {out_dir if save_on_exit else "저장 안 함 (save_on_exit:=false)"}')]
    if note and save_on_exit:
        acts.append(LogInfo(msg=f'[3dmap] {note}'))

    # ── 카메라 내부 static TF ──────────────────────────────────────
    # bag 재생 모드에서는 realsense2_camera 가 안 떠서
    # camera_link -> camera_depth_optical_frame 이 통째로 빈다. /tf_static 도
    # replay_exclude 로 재생에서 빠지므로 live_tsdf 가 base_link -> depth 를
    # 못 풀고 drop_tf 만 올라간다(맵 0건). 그때만 우리가 대신 발행한다.
    # ⚠ 실시간 모드에서 같이 발행하면 camera_depth_optical_frame 의 부모가
    #   camera_link 와 camera_depth_frame 둘이 되어 TF 트리가 깨진다.
    mode = cfg('camera_tf').lower()
    if mode == 'auto':
        need_tf = not _camera_node_running()
        why = ('realsense2_camera 노드가 안 보임 -> bag 재생 모드로 판단'
               if need_tf else 'realsense2_camera 노드 실행 중 -> 카메라가 직접 발행')
    else:
        need_tf = mode in ('true', '1')
        why = f'camera_tf:={mode} 명시'
    acts.append(LogInfo(msg=f'[3dmap] camera static TF {"발행" if need_tf else "생략"} ({why})'))
    if need_tf:
        acts.append(Node(package='tf2_ros', executable='static_transform_publisher',
                         name='camera_depth_optical_tf', arguments=CAM_DEPTH_TF,
                         parameters=[{'use_sim_time': sim}], output='screen'))

    # [261008] 맵 저장 방식: grid(그리드 피쳐맵, 기본 — 교수님 10/8 지시) | tsdf(옛 동작, A/B·오프라인 비교용)
    backend = cfg('map_backend').strip().lower()
    if backend not in ('grid', 'tsdf'):
        raise RuntimeError(f"map_backend 는 grid|tsdf: {backend!r}")
    # [261008 밤] map 좌표계 · 맵 전체 판정 · 피쳐맵 불러오기
    map_frame = cfg('map_frame').strip()
    extent = cfg('judge_extent').strip().lower()
    if extent not in ('map', 'window'):
        raise RuntimeError(f"judge_extent 는 map|window: {extent!r}")
    if backend == 'tsdf' and extent == 'map':
        extent = 'window'                       # TSDF 는 반경 crop 점군만 보내므로 맵 전체 판정 불가
        acts.append(LogInfo(msg='[3dmap] map_backend:=tsdf 라 judge_extent 를 window 로 (플래너는 옛 누적)'))
    fm_load = cfg('feature_map_load').strip()
    # [261011] 루프 클로저 때 피쳐맵을 보정 궤적으로 다시 쌓기 — grid + map 좌표일 때만 (daron_viz lc_resync 와 같이)
    lc_resync = cfg('lc_resync').strip().lower() in ('true', '1', 'yes') and backend == 'grid' and bool(map_frame)
    if not health_on:
        acts.append(LogInfo(msg=f'[3dmap] health:=false → odom_topic={odom_topic} slam_topic={slam_topic} '
                                f'health 게이트 끔 (live_tsdf health_topic 빈 값, traversability require_health:=false)'))
    tsdf = ExecuteProcess(
        name='live_tsdf',
        cmd=[VENV_PY, LIVE_TSDF, '--ros-args',
             '-p', f'odom_topic:={odom_topic}',
             '-p', f'depth_scale:={dbl("depth_scale", cfg("depth_scale"))}',     # [261017] DOUBLE 고정 (dmax:=1 이면 죽던 문제)
             '-p', f'voxel:={dbl("voxel", cfg("voxel"))}',
             '-p', f'dmax:={dbl("dmax", cfg("dmax"))}',
             '-p', f'dmin:={dbl("dmin", cfg("dmin"))}',
             '-p', f'every:={_int_arg("every", cfg("every"))}',
             '-p', f'publish_period:={dbl("publish_period", cfg("publish_period"))}',
             '-p', f'out_dir:={out_dir}',
             '-p', f'save_on_exit:={"true" if save_on_exit else "false"}',
             '-p', f'ground_filter:={cfg("ground_filter")}',
             '-p', f'speckle_min_area:={cfg("speckle_min_area")}',
             '-p', f'max_height:={cfg("max_height")}',
             '-p', f'range_color:={cfg("range_color")}',
             # [health 260929] 포즈 게이트 + 기동 무시 + 반경 발행 (live_tsdf 전용 — 주행가능성 노드엔 안 넘김)
             # 빈 값은 '' 로 — '-p health_topic:=' 그대로면 rcl 이 'Failed to parse global arguments' 로 죽는다(260929 실측)
             '-p', 'health_topic:=' + (health_topic or "''"),
             '-p', f'slam_topic:={slam_topic}',
             '-p', f'warmup_s:={cfg("warmup_s")}',
             '-p', f'publish_radius:={cfg("publish_radius")}',
             '-p', f'full_publish_period:={cfg("full_publish_period")}',
             '-p', f'backend:={backend}',                                    # [261008]
             '-p', 'map_frame:=' + (map_frame or "''"),
             '-p', f'cells_radius:={"0.0" if extent == "map" else "-1.0"}',
             '-p', 'feature_map_load:=' + (fm_load or "''"),
             '-p', f'lc_resync:={"true" if lc_resync else "false"}',            # [261011]
             '-p', f'use_sim_time:={"true" if sim else "false"}'],
        output='screen', emulate_tty=True)
    acts.append(tsdf)
    acts.append(LogInfo(msg=f'[3dmap] 좌표 {map_frame or "odom"} | 판정 범위 {extent}'
                            + (' (피쳐맵 전체 판정 → 플래너 grid_mode:=replace)' if extent == 'map' else '')
                            + f' | 피쳐맵 불러오기 {fm_load or "안 함"} (재추정 ACCEPTED 일 때)'
                            + f' | 루프 클로저 재동기화 {"켬" if lc_resync else "끔"}'))
    acts.append(LogInfo(msg=f'[3dmap] 맵 저장 방식 map_backend:={backend} → 판정 입력 '
                            + ('/live_tsdf/map_cells (그리드 피쳐맵 칸 채널, dense 전체 누적 없음)'
                               if backend == 'grid' else '/live_tsdf/map_cloud (TSDF 전체 누적 후 반경 crop — 옛 동작)')))

    if cfg('trav').lower() in ('true', '1'):
        acts.append(ExecuteProcess(
            name='traversability',
            cmd=[VENV_PY, TRAV, '--ros-args',
                 '--params-file', TRAV_PARAMS,
                 '-p', f'odom_topic:={odom_topic}',
                 # [261017] health:=false 면 /health/state 발행자가 없어 require_health(기본 true) 가 영원히 health_none 이었다
                 '-p', f'require_health:={"true" if health_on else "false"}']
                + (['-p', f'health_topic:={health_topic}'] if health_topic else [])
                + ['-p', f'gravity_mode:={cfg("gravity_mode")}',
                 '-p', f'judge:={cfg("judge")}',
                 '-p', f'input_mode:={"cells" if backend == "grid" else "cloud"}',   # [261008]
                 '-p', 'map_frame:=' + (map_frame or "''"),
                 '-p', f'judge_extent:={extent}',
                 '-p', f'use_sim_time:={"true" if sim else "false"}'],
            output='screen', emulate_tty=True))
    else:
        acts.append(LogInfo(msg='[3dmap] traversability 비활성 (trav:=false)'))

    if cfg('depth_viz').lower() in ('true', '1'):
        acts.append(ExecuteProcess(
            name='depth_colorize',
            cmd=[VENV_PY, DEPTH_VIZ, '--ros-args',
                 '-p', f'depth_scale:={dbl("depth_scale", cfg("depth_scale"))}',
                 '-p', f'dmin:={dbl("dmin", cfg("dmin"))}',
                 '-p', f'dmax:={dbl("dmax", cfg("dmax"))}',
                 '-p', f'use_sim_time:={"true" if sim else "false"}'],
            output='screen', emulate_tty=True))

    # ── [261001] CDA 핀 ────────────────────────────────────────────
    # cda:=fake  fake_detector(CSV 를 영상 stamp 에 맞춰 발행) + pin_node  — YOLO 없이 배관 시험
    # cda:=pins  pin_node 만 — YOLO 노드가 /cda/detections 를 따로 낸다
    cda = cfg('cda').lower()
    if cda in ('fake', 'pins'):
        acts.append(ExecuteProcess(
            name='pin_node',
            cmd=[SYS_PY, PIN_NODE, '--ros-args',
                 '-p', f'odom_topic:={odom_topic}',      # 맵과 같은 포즈 소스
                 '-p', 'map_frame:=' + (map_frame or "''"),          # [261011] 핀도 피쳐맵과 같은 map 좌표
                 '-p', f'lc_resync:={"true" if lc_resync else "false"}',
                 '-p', 'pins_load:=' + (cfg('pins_load').strip() or "''"),         # [261011 밤] 이전 세션 핀
                 '-p', 'health_topic:=' + (health_topic or "''"),
                 '-p', f'dmin:={dbl("dmin", cfg("dmin"))}', '-p', f'dmax:={dbl("dmax", cfg("dmax"))}',
                 '-p', f'depth_scale:={dbl("depth_scale", cfg("depth_scale"))}',
                 '-p', f'min_count:={_int_arg("cda_min_count", cfg("cda_min_count"))}',
                 # 빈 값을 그대로 넘기면 rcl 이 'Couldn't parse parameter override rule' 로 노드를 죽인다 (261001 실측)
                 '-p', 'out_dir:=' + ((out_dir if save_on_exit else '') or "''"),
                 '-p', f'use_sim_time:={"true" if sim else "false"}'],
            output='screen', emulate_tty=True))
        if cda == 'fake':
            csv_path = os.path.expanduser(cfg('cda_csv'))
            if not os.path.isfile(csv_path):
                raise RuntimeError(f'cda:=fake 인데 cda_csv 가 없습니다: {csv_path!r}')
            acts.append(ExecuteProcess(
                name='fake_detector',
                cmd=[SYS_PY, FAKE_DET, '--ros-args', '-p', f'csv:={csv_path}',
                     '-p', f'latency_s:={dbl("cda_latency", cfg("cda_latency"))}',
                     '-p', f'use_sim_time:={"true" if sim else "false"}'],
                output='screen', emulate_tty=True))
        if cfg('cda_overlay').lower() in ('true', '1'):
            acts.append(ExecuteProcess(
                name='cda_overlay',
                cmd=[SYS_PY, CDA_OVERLAY, '--ros-args',
                     '-p', f'use_sim_time:={"true" if sim else "false"}'],
                output='screen', emulate_tty=True))
        acts.append(LogInfo(msg=f'[3dmap] CDA 핀 켬 (cda:={cda}) -> /pin_node/pins (RViz "CDA Pins")'
                                + (', /cda/overlay (RViz "CDA Overlay")' if cfg('cda_overlay').lower() in ('true', '1') else '')))
    elif cda not in ('false', '0', ''):
        raise RuntimeError(f'cda 는 false|fake|pins 중 하나: {cda!r}')

    # ── [261006] 경로계획 (기본 꺼짐) ───────────────────────────────
    # 기록 폴더는 세션 폴더(bag_dir/session_name) 밖에 둔다 — 노드가 기동하며 폴더를 만들기 때문에
    # 세션 폴더 안이면 daron_viz 녹화가 '폴더 있음'으로 막힌다 (_resolve_out_dir 참고).
    if cfg('planner').lower() in ('true', '1'):
        params = os.path.expanduser(cfg('planner_params'))
        for f in (PLANNER, params):
            if not os.path.exists(f):
                raise RuntimeError(f'planner:=true 인데 파일이 없습니다: {f}')
        log_dir = os.path.expanduser(cfg('planner_log_dir').strip())
        if not log_dir:
            tag = cfg('session_name').strip() or 'live'
            log_dir = os.path.join(HOME, 'daron_nav_runs', time.strftime(f'{tag}_%Y%m%d_%H%M%S'))
        acts.append(ExecuteProcess(
            name='daron_planner',
            cmd=[VENV_PY, PLANNER, '--ros-args',
                 '--params-file', params,
                 '-p', f'log_dir:={log_dir}',
                 # [261008 밤] 판정이 맵 전체를 보내면 플래너는 쌓지 않고 그대로, 좌표는 map
                 '-p', f'grid_mode:={"replace" if extent == "map" else "accumulate"}',
                 '-p', f'odom_frame:={map_frame or "odom"}',
                 '-p', f'use_sim_time:={"true" if sim else "false"}'],
            # [261016] 프로세스가 죽으면(segfault·OOM·kill) 2 s 뒤 다시. 파이썬 예외는 노드 안에서 정지(ERROR)로 처리한다.
            # 다시 뜬 플래너는 목표가 없어 경로를 안 냄(로버 정지) — 제어창 웨이포인트는 일시정지→재개로 다시 보낸다.
            respawn=True, respawn_delay=2.0,
            output='screen', emulate_tty=True))
        acts.append(LogInfo(msg=f'[3dmap] 경로계획 켬 (planner:=true) params={params}  기록={log_dir}'))
        # [261011] 기준점 중계 — 재생(use_sim_time)이면 노드 시계도 sim 이어야 TF 나이 검사(0.5 s)를 통과한다
        if cfg('pose_relay').lower() in ('true', '1'):
            if not os.path.exists(POSE_RELAY):
                raise RuntimeError(f'pose_relay:=true 인데 파일이 없습니다: {POSE_RELAY}')
            acts.append(ExecuteProcess(
                name='pose_relay',
                cmd=[VENV_PY, POSE_RELAY, '--ros-args',
                     '-p', f'odom_frame:={map_frame or "odom"}',
                     '-p', f'use_sim_time:={"true" if sim else "false"}'],
                output='screen', emulate_tty=True))
            acts.append(LogInfo(msg=f'[3dmap] 기준점 중계 켬 (pose_relay:=true) TF {map_frame or "odom"}->base_link -> /localization/pose'))

    if cfg('rviz').lower() in ('true', '1'):
        acts.append(Node(package='rviz2', executable='rviz2', name='rviz2_3dmap',
                         arguments=['-d', RVIZ_CFG],
                         parameters=[{'use_sim_time': sim}], output='screen'))
    return acts


def generate_launch_description():
    A = DeclareLaunchArgument
    return LaunchDescription([
        A('odom_topic', default_value='/odometry/filtered_health',
          description='포즈 소스. 맵과 격자가 같은 값을 쓴다. README §8 — 한 맵에 두 소스를 섞지 말 것. '
                      '260929 기본값 /slam/odometry -> /odometry/filtered_health (최종 pose = EKF-C, 사용자 결정). '
                      'daron_viz 를 health:=false 로 띄웠으면 여기도 health:=false (odom_topic 이 기본값이면 /slam/odometry 로 바뀐다)'),
        A('health', default_value='true',
          description='[261017] daron_viz 의 health 와 같게. false 면 health_topic 빈 값·slam_topic=odom_topic·'
                      'traversability require_health:=false 를 한 번에 적용한다 (안 그러면 맵이 안 쌓이고 격자가 안 나온다)'),
        A('health_topic', default_value='/health/state',
          description='[health] 맵 포즈 게이트 입력 (health_monitor). 빈 문자열이면 게이트 끔(기존 동작). '
                      '토픽이 안 오면 OK 로 간주한다'),
        A('slam_topic', default_value='/slam/odometry_h',
          description='[health] ANCHOR/HOLD 포즈 계산용 SLAM 오도메트리. 기본은 health_monitor 가 점프를 지운 '
                      '연속 포즈(/slam/odometry_h) — 원시 /slam/odometry 는 cuVSLAM 이 LOST 없이 튀는 점프를 그대로 담는다'),
        A('warmup_s', default_value='2.0',
          description='[health] 첫 odom 뒤 이 시간은 적분 안 함 (EKF 출력 첫 ~2 s 는 최대 0.9 m 떠 있음)'),
        A('publish_radius', default_value='1.25',
          description='발행(map_cloud, RViz·주행가능성 입력) 반경 [m]. 판정 1 m + 판정 주변 참조 25 cm. '
                      '0 = 전체. 저장은 항상 전체 맵'),
        A('map_frame', default_value='map',
          description='[261008] 피쳐맵·판정·플래너 좌표계. map = Slam 보정(루프 클로저·재추정) 반영, map->odom TF 필요 '
                      '(daron_viz 가 Slam 이나 항등 static TF 로 늘 낸다). 빈 값 = 옛 동작(odom).'),
        A('judge_extent', default_value='map',
          description='[261008] map = 피쳐맵 전체를 2 s 마다 판정(플래너는 쌓지 않고 그대로 씀) / window = 로버 주변 1.25 m(옛 동작, 플래너가 비용 누적)'),
        A('feature_map_load', default_value='auto',
          description='[261008] 재추정 ACCEPTED 시 이전 피쳐맵 불러오기. auto = 재추정한 Slam 맵 폴더 상위 세션의 최신 '
                      'map_*_featuregrid.npz / 파일 경로 / 빈 값 = 안 함'),
        A('pins_load', default_value='auto',
          description='[261011 밤] 재추정 ACCEPTED 시 이전 세션 크레이터 핀 불러오기 (새 검출이 같은 핀으로 합쳐짐). '
                      'auto = 재추정한 Slam 맵 폴더 상위 세션의 최신 pins_*.json / 파일 경로 / 빈 값 = 안 함'),
        A('lc_resync', default_value='true',
          description='[261011] 루프 클로저 때 피쳐맵을 cuVSLAM 보정 궤적(/slam/map_odom_history)으로 다시 쌓는다. '
                      'grid·map_frame:=map 일 때만. daron_viz 의 lc_resync 도 true 여야 이력이 나온다. false = 옛 동작'),
        A('map_backend', default_value='grid',
          description='[261008] grid = 그리드 피쳐맵(5 cm 칸마다 높이 평균·분산·하/상단·점 수만 누적, '
                      'dense 전체 맵 안 쌓음 — 판정 노드 input_mode:=cells) / tsdf = 옛 TSDF 전체 누적(비교용)'),
        A('full_publish_period', default_value='30.0',
          description='[260930] 전체 맵(map_cloud_full, RViz 표시 전용 회색) 발행 주기 [s]. 0 = 끔. '
                      '반경 밖은 로버가 돌아오기 전엔 안 변하고, 반경 안은 map_cloud_color 가 2 s 마다 덮어 그린다'),
        # depth_scale: realsense2_camera 노드가 기동하며 장치의 depth_units 를 0.001 로
        # 덮어쓴다(260928 실측: 노드 종료 후 재조회해도 0.001 로 남음). 장치 공장값은
        # 1e-4 이지만 **ROS 로 발행/녹화되는 값은 1e-3 기준**이다.
        A('cda', default_value='false',
          description='[261001] CDA 핀. false | fake (fake_detector + pin_node, cda_csv 필요) | '
                      'pins (pin_node 만 — YOLO 노드가 /cda/detections 발행). ~/Daron/yolo/README.md'),
        A('cda_csv', default_value='',
          description='cda:=fake 용 검출 CSV (stamp_ns,camera,x1,y1,x2,y2,score,class). '
                      '예: ~/Daron/yolo/data/rock_data/annotations.csv'),
        A('cda_latency', default_value='0.15', description='cda:=fake 가 흉내 내는 추론 지연 [s]'),
        A('cda_min_count', default_value='3', description='핀 확정에 필요한 검출 횟수 (미확정 = 반투명 주황)'),
        A('cda_overlay', default_value='true',
          description='[261005] cda:=fake|pins 일 때 검출 bbox 를 infra1 위에 그린 /cda/overlay 발행 (RViz "CDA Overlay")'),
        A('depth_scale', default_value='0.001', description='m per LSB (ROS 경로는 0.001)'),
        A('voxel', default_value='0.01', description='TSDF 복셀 [m]'),
        A('dmin', default_value='0.07'),
        A('dmax', default_value='1.0',
          description='이보다 먼 depth 는 맵에 넣지 않는다. 원거리 오측정이 공중에 뿌려지는 것 방지. '
                      '260928 1.2 -> 1.0 (README §11, §12-12)'),
        A('every', default_value='2', description='depth 프레임 간격 (30Hz -> 15Hz)'),
        A('publish_period', default_value='2.0'),
        A('out_dir', default_value=os.path.join(DATA_ROOT, 'maps'),
          description='맵 파일 저장 폴더 (session_name 을 안 줬을 때만 쓰임)'),
        A('session_name', default_value='',
          description='daron_viz 의 session_name 과 같은 값을 주면 맵 파일을 bag 폴더 '
                      '(bag_dir/session_name) 안에 같이 저장한다. 비우면 out_dir'),
        A('bag_dir', default_value=os.path.join(DATA_ROOT, 'daron_bags'),
          description='daron_viz 의 bag_dir 와 같아야 한다'),
        A('save_on_exit', default_value='true',
          description='종료 시 맵 파일 저장. false 면 파일 없음(bag 에 맵 토픽만 남김). '
                      '실행 중 수동 저장: ros2 service call /live_tsdf/save_map std_srvs/srv/Trigger'),
        A('trav', default_value='true', description='주행가능성 격자 노드 실행'),
        A('ground_filter', default_value='true',
          description='지면과 이어지지 않은(공중에 뜬) 덩어리 제거. 3dmap/ground_filter.py'),
        A('speckle_min_area', default_value='0.0025',
          description='depth 고립 조각(스테레오 오매칭) 제거 면적 [m²]. 적분 전에 적용. 0.0 = 끔. '
                      '3dmap/depth_filter.py (260928: 25 cm²)'),
        A('max_height', default_value='0.0',
          description='지면 위 이 높이[m] 초과 점을 발행/저장(_ground)에서 제거. 0.0 = 끔. '
                      '실제 구조물 윗부분도 잘린다. 로버 높이 0.1155 m (README §12-14)'),
        A('range_color', default_value='true',
          description='3D 맵 거리색 (depth 영상과 같은 turbo·같은 dmin~dmax). 켜진 채 save_map 하면 '
                      '*_range.ply 저장. 실행 중 토글: /live_tsdf/set_range_color (SetBool)'),
        A('gravity_mode', default_value='imu', description='imu | fixed'),
        A('judge', default_value='judge_core',
          description='주행가능성 판정기. judge_core(261008 기본) | builtin '
                      '(3dmap/judge_adapters.py 의 JUDGES 에 등록된 이름)'),
        A('depth_viz', default_value='true',
          description='depth 16UC1 -> 컬러맵 릴레이. RViz 에서 depth 를 눈으로 보려면 필요하다'),
        A('camera_tf', default_value='auto',
          description="camera_link->camera_depth_optical_frame static TF 발행. "
                      "auto = realsense 노드가 없을 때만(=bag 재생) 발행. true|false 로 강제 가능"),
        A('use_sim_time', default_value='false',
          description='bag 재생 시 true. daron_viz 는 bag_path 지정 시 전 노드를 '
                      'use_sim_time:=true 로 강제하므로 여기도 맞춰야 stamp 가 일치한다'),
        A('rviz', default_value='true'),
        A('planner', default_value='false',
          description='[261006] 경로계획 노드(<Daron 옆>/daron-nav/daron_planner/ros/planner_node.py) 같이 실행. '
                      '목표는 RViz 2D Goal Pose. 경로계획 작업 중엔 false 로 두고 터미널을 따로 띄우는 편이 편하다'),
        A('planner_params', default_value=os.path.join(PLANNER_DIR, 'live_lab.yaml'),
          description='경로계획 파라미터 파일 (기본: 연구실 실시간 시험용 live_lab.yaml)'),
        A('planner_log_dir', default_value='',
          description='경로계획 기록 폴더 (plans.jsonl, arena_*.npz). 비우면 ~/daron_nav_runs/<session_name|live>_<시각>'),
        A('pose_relay', default_value='true',
          description='[261011] planner:=true 일 때 pose_relay(<Daron 옆>/daron-nav/daron_planner/ros/pose_relay.py)도 같이 실행 '
                      '-> /localization/pose. 따로 띄우는 경우(예: videos_1011 run_scene.sh) false'),
        OpaqueFunction(function=_setup),
    ])
