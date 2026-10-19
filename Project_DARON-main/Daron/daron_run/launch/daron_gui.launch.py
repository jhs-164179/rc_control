#!/usr/bin/env python3
"""DARON 지상국 — 매핑 + 3D RViz + 탑뷰 RViz + 제어창 + CDA 핀 갤러리를 한 번에. 작성 261006.
설계: ~/Daron/지상국 GUI 기능 계획.md

daron_viz(센서·SLAM·EKF·녹화)만 따로 띄우고, 나머지는 이 launch 하나로 띄운다.

    터미널 A:  ros2 launch ~/Daron/daron_run/launch/daron_viz.launch.py ...
    터미널 B:  ros2 launch ~/Daron/daron_run/launch/daron_gui.launch.py [session_name:=<A 와 같게>]

⚠ `source ~/Daron/slam_env.sh` 후 실행 (ROS_DOMAIN_ID=42, PYTHONPATH).

띄우는 것 (끄려면 각 인자 :=false)
  map         daron_3dmap.launch.py 를 포함 (rviz:=false 고정 — 3D 창은 아래 viz3d).
              cda(기본 pins)·planner(기본 true) 와 3dmap 의 나머지 인자(session_name, trav, ...)는 그대로 넘어간다.
              ⚠ 3dmap 을 따로 띄우고 있으면 map:=false (같은 노드가 두 벌 뜬다)
  viz3d       rviz2 -d rviz/daron_viz3d.rviz — 3D 화면. Views 패널의 저장 뷰 '탑뷰'로 전환하면 이 창에서도
              웨이포인트를 찍을 수 있다 (창 하나만 쓰려면 topview:=false)
  topview     rviz2 -d rviz/daron_gui_topview.rviz — 탑뷰 전용 창 (Fixed Frame odom, TopDownOrtho)
  gui         제어창 (nodes/gui/ground_station.py)
  follower    [261012] 경로 추종기 (rover/server/path_follower.py, Stanley 곡률 제어, rover/PATH_TRACKING.md).
              /planner/path + /localization/pose(pose_relay) → /rover/auto/drive_rpm·steer_cmd. 기동 때는 꺼져 있고
              제어창 '웨이포인트' 버튼이 /path_follower/enable 로 켠다. 파라미터 rover/server/config/follower.yaml (follower_params).
              로버 쪽은 RADXA 의 radxa.py(자율 수신부 있는 판)가 떠 있어야 실제로 움직인다.
  pin_gallery CDA 핀 → 클릭 가능한 InteractiveMarker + 핀별 대표 이미지 (nodes/gui/pin_gallery.py)
              rim:=true 면 대표 박스로 림 선명도도 계산해 정보 창에 표시 (nodes/gui/rim/, 261007)
              crater:=true 면 대표 프레임 depth 로 크레이터 경사·깊이·메시 계산, 정보 창 표시 + '3D 보기' (nodes/gui/crater/, 261009)
                관측이 끝난 뒤(crater_finalize_timeout 초 동안 새 검출 없음) 한 번만 계산
              degradation:=true 면 관측이 끝난 크레이터의 열화도 순번 (nodes/gui/degradation_bridge.py, 261013)
  lc_viz      [261015] 루프 클로저 보정 표시 (nodes/gui/lc_viz_node.py) — 3D RViz 그룹 '루프 클로저 보정'.
              보정 직후 10 s: 보정 전 칸 자리(빨강), 옮겨진 크레이터 핀의 전 자리(보라), 로버 위치 전→후(빨강→초록 화살표),
              실시간 궤적(빨강)·보정 궤적(초록). 구독만 하고 /lc_viz/* 만 낸다 — 지도·핀·웨이포인트·로버 보정 자체는 이것과 무관

웨이포인트: 두 RViz 창의 2D Goal Pose(G) 모두 /gui/waypoint_goal → 제어창 목록 → 플래너(/goal_pose)에 하나씩.
  [261011] 누른 채 드래그한 방향 = 도착 방위. 마커 = 원판(wp_pos_tol_m) + 화살표 + 부채꼴(±wp_yaw_tol_deg) — 표시·기록만,
    플래너는 아직 위치만 쓴다 (방위 도착 판정은 나중).
  정밀하게는 탑뷰(정사영)에서. 원근 3D 뷰의 Goal Pose 는 z=0 평면 투영이라 바닥 높이만큼 x, y 가 어긋난다.
  [261011] 제어창 '영상에서 찍기' → D405 infra1 창에서 누르고 드래그 (바닥 0.07~1.8 m 만, image_click:=false 로 끔).
RViz 는 한 창 안에서 3D 뷰를 둘로 나누지 못한다 — 그래서 창 두 개를 한 명령으로 띄운다.
  ⚠ [261008] 제어창은 waypoint_frame(기본 map — 261008 밤 변경) 이 아닌 좌표계의 클릭을 거부한다 (TF 변환 안 함).
    탑뷰·3D RViz 고정 좌표계가 둘 다 map 이라 두 창 모두 2D Goal Pose 가 받아진다.

[261008] 웨이포인트 기록·불러오기 (제어창, 형식은 ground_station.py 독스트링)
  session_dir          waypoints.jsonl 을 쓸 폴더. 비우면 3dmap 맵 파일과 같은 세션 폴더:
                         session_name 을 주면 <bag_dir>/<session_name> (daron_viz 녹화 폴더),
                         안 주면 gui_out_dir (~/daron_gui_runs/<시각>, 핀 이미지와 같은 곳).
                       폴더가 없으면 제어창이 첫 웨이포인트 때 만든다 (daron_viz 녹화를 먼저 띄우는 게 원칙).
  waypoint_frame       저장·불러오기 좌표계 (기본 map: Slam 재추정·루프 클로저 반영 좌표)
  map_id               이 세션의 지도 이름(선택). 불러온 파일과 map_id·세션이 다르면 경고
  load_waypoints_file  기동 시 지도에 표시할 waypoints.jsonl (표시만, 목록엔 안 넣음)

[261010] 로버 키보드 조종 (제어창 '수동' 버튼, 제어팀 console.py 와 같은 규약 — ground_station.py 독스트링)
  rover_ns             로버 노드 이름 (기본 /rover = radxa.py). 이 아래 토픽으로 송신, <rover_ns>/diagnostics 표시
  max_steer_deg        차량 유효조향각 한계 (빈 값 = 20°, 실측 전 보수적 운용값. 초과 입력은 중립 복귀)
  ⚠ console.py 와 '수동'을 동시에 켜지 말 것 (수동이 꺼져 있으면 제어창은 아무것도 안 보냄)
"""
import os
import time

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, LogInfo, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

HOME = os.path.expanduser('~')
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DATA_ROOT = os.path.dirname(ROOT)                          # [261008] daron_3dmap 과 같은 bag_dir 기본값 (~/Daron/daron_bags)
GUI_DIR = os.path.join(ROOT, 'nodes', 'gui')
GROUND = os.path.join(GUI_DIR, 'ground_station.py')
GALLERY = os.path.join(GUI_DIR, 'pin_gallery.py')
MAP_LAUNCH = os.path.join(ROOT, 'launch', 'daron_3dmap.launch.py')
VIZ3D_CFG = os.path.join(ROOT, 'rviz', 'daron_viz3d.rviz')
TOPVIEW_CFG = os.path.join(ROOT, 'rviz', 'daron_gui_topview.rviz')
FOLLOWER = os.path.join(ROOT, 'rover', 'server', 'path_follower.py')                  # [261012]
FOLLOWER_PARAMS = os.path.join(ROOT, 'rover', 'server', 'config', 'follower.yaml')
LC_VIZ = os.path.join(GUI_DIR, 'lc_viz_node.py')                                   # [261015]
SYS_PY = '/usr/bin/python3'


def _setup(context):
    def cfg(name):
        return LaunchConfiguration(name).perform(context)

    if 'ros' not in os.environ.get('PYTHONPATH', ''):
        raise RuntimeError('PYTHONPATH 에 ROS 가 없습니다. `source ~/Daron/slam_env.sh` 후 다시 실행하세요.')
    viz3d_cfg = os.path.expanduser(cfg('viz3d_cfg').strip() or VIZ3D_CFG)        # [261008] 녹화용 창 배치 사본 등
    topview_cfg = os.path.expanduser(cfg('topview_cfg').strip() or TOPVIEW_CFG)
    for f in (GROUND, GALLERY, MAP_LAUNCH, viz3d_cfg, topview_cfg):
        if not os.path.exists(f):
            raise RuntimeError(f'파일이 없습니다: {f}')
    on = lambda k: cfg(k).lower() in ('true', '1')  # noqa: E731
    sim = on('use_sim_time')
    sim_s = 'true' if sim else 'false'
    sim_arg = ['-p', f'use_sim_time:={sim_s}']
    out_dir = os.path.expanduser(cfg('gui_out_dir').strip()) or os.path.join(
        HOME, 'daron_gui_runs', time.strftime('%Y%m%d_%H%M%S'))
    # [261008] 웨이포인트 기록 폴더 = 3dmap 맵 파일과 같은 세션 폴더 (daron_3dmap._resolve_out_dir 와 같은 규칙).
    #   세션 없이 띄우면 3dmap 은 maps/ 에 평평하게 쓰므로 세션별 폴더가 없다 → 이 실행의 gui_out_dir.
    session = cfg('session_name').strip()
    session_dir = os.path.expanduser(cfg('session_dir').strip())
    if not session_dir:
        session_dir = (os.path.join(os.path.abspath(os.path.expanduser(cfg('bag_dir'))), session) if session
                       else out_dir)
    session_dir = os.path.abspath(session_dir)
    wp_note = ''
    if not os.path.isdir(session_dir):
        wp_note = (f' (아직 없음 — 첫 웨이포인트 때 제어창이 만든다. 녹화할 거면 daron_viz record:=true '
                   f'session_name:={session} 를 먼저)' if session else ' (첫 웨이포인트 때 만듦)')

    def q(v):           # -p 값을 YAML 문자열로 고정: 숫자만인 이름(0930)·빈 값이 INTEGER·오류가 되지 않게
        return "'" + v.replace("'", "''") + "'"
    acts = []
    if on('map'):
        # 3dmap 의 다른 인자(session_name, trav, cda_csv ...)는 같은 launch 컨텍스트라 명령줄 값이 그대로 보인다
        acts.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(MAP_LAUNCH),
            launch_arguments={'rviz': 'false', 'cda': cfg('cda'), 'planner': cfg('planner'),
                              'use_sim_time': sim_s, 'odom_topic': cfg('odom_topic')}.items()))
    if on('pin_gallery'):
        acts.append(ExecuteProcess(
            name='pin_gallery',
            cmd=[SYS_PY, GALLERY, '--ros-args', '-p', f'out_dir:={os.path.join(out_dir, "pins")}',
                 '-p', f'rim:={"true" if on("rim") else "false"}',
                 '-p', f'crater:={"true" if on("crater") else "false"}',
                 '-p', f'crater_finalize_timeout_s:={float(cfg("crater_finalize_timeout"))}',
                 '-p', f'crater_z_exaggeration:={float(cfg("crater_z_exaggeration"))}',
                 '-p', f'crater_diameter_source:={cfg("crater_diameter_source")}',
                 '-p', f'degradation:={"true" if on("degradation") else "false"}',
                 '-p', f'degradation_reference:={q(os.path.expanduser(cfg("degradation_reference").strip()))}'] + sim_arg,
            output='screen', emulate_tty=True))
    if on('gui'):
        acts.append(ExecuteProcess(
            name='ground_station',
            cmd=[SYS_PY, GROUND, '--ros-args', '-p', f'odom_topic:={cfg("odom_topic")}',
                 '-p', f'session_dir:={q(session_dir)}',
                 '-p', f'session_name:={q(session or os.path.basename(session_dir))}',
                 '-p', f'waypoint_frame:={q(cfg("waypoint_frame").strip())}',
                 '-p', f'map_id:={q(cfg("map_id").strip())}',
                 '-p', f'load_waypoints_file:={q(os.path.expanduser(cfg("load_waypoints_file").strip()))}',
                 '-p', f'window_geometry:={q(cfg("ctl_geometry").strip())}',
                 '-p', f'rover_ns:={q(cfg("rover_ns").strip() or "/rover")}',
                 '-p', f'image_click:={"true" if on("image_click") else "false"}',
                 '-p', f'wp_pos_tol_m:={float(cfg("wp_pos_tol_m"))}',
                 '-p', f'wp_yaw_tol_deg:={float(cfg("wp_yaw_tol_deg"))}',
                 '-p', f'lc_resync:={"true" if on("lc_resync") else "false"}']   # [261011]
            + (['-p', f'max_steer_deg:={float(cfg("max_steer_deg"))}'] if cfg('max_steer_deg').strip() else [])
            + sim_arg,
            output='screen', emulate_tty=True))
    # [261012] 경로 추종기. 기동 때 꺼짐(start_enabled false) — 꺼진 동안은 /rover/auto/* 를 안 보낸다.
    #   노드 이름 path_follower 고정 (제어창 follower_ns 기본 /path_follower 와 맞춤). 로그 follow_*.csv → rover/server/logs/
    if on('follower'):
        f_params = os.path.expanduser(cfg('follower_params').strip() or FOLLOWER_PARAMS)
        for f in (FOLLOWER, f_params):
            if not os.path.exists(f):
                raise RuntimeError(f'follower:=true 인데 파일이 없습니다: {f}')
        stamp_src = cfg('follower_pose_stamp').strip()
        acts.append(ExecuteProcess(
            name='path_follower',
            cmd=[SYS_PY, FOLLOWER, '--ros-args', '--params-file', f_params,
                 '-p', 'start_enabled:=false',
                 '-p', f'rover:={q(cfg("rover_ns").strip() or "/rover")}']
            + (['-p', f'pose_stamp_source:={stamp_src}'] if stamp_src else [])
            + sim_arg,
            output='screen', emulate_tty=True))
    # [261015] 루프 클로저 보정 표시 — 표시 전용 (구독만, /lc_viz/* 만 발행). viz3d 를 따로 띄워도 보이게 viz3d 와 무관하게 기동
    if on('lc_viz'):
        acts.append(ExecuteProcess(
            name='lc_viz',
            cmd=[SYS_PY, LC_VIZ, '--ros-args', '-p', f'map_frame:={q(cfg("waypoint_frame").strip() or "map")}'] + sim_arg,
            output='screen', emulate_tty=True))
    if on('viz3d'):
        acts.append(Node(package='rviz2', executable='rviz2', name='rviz2_viz3d',
                         arguments=['-d', viz3d_cfg], parameters=[{'use_sim_time': sim}], output='screen'))
    if on('topview'):
        acts.append(Node(package='rviz2', executable='rviz2', name='rviz2_gui_topview',
                         arguments=['-d', topview_cfg], parameters=[{'use_sim_time': sim}], output='screen'))
    acts.append(LogInfo(msg=f'[gui] 매핑={on("map")}(cda={cfg("cda")}, planner={cfg("planner")}) 3D창={on("viz3d")} '
                            f'탑뷰={on("topview")} 제어창={on("gui")} 핀갤러리={on("pin_gallery")} 추종기={on("follower")} LC표시={on("lc_viz")} '
                            f'기록={out_dir}'))
    if on('gui'):
        acts.append(LogInfo(msg=f'[gui] 웨이포인트 기록 {os.path.join(session_dir, "waypoints.jsonl")}{wp_note} '
                                f'좌표계={cfg("waypoint_frame")} map_id={cfg("map_id")!r}'
                                + (f' 불러오기={cfg("load_waypoints_file")}' if cfg('load_waypoints_file').strip() else '')))
    return acts


def generate_launch_description():
    A = DeclareLaunchArgument
    return LaunchDescription([
        A('map', default_value='true', description='daron_3dmap.launch.py 포함 (3dmap 을 따로 띄우면 false)'),
        A('viz3d', default_value='true', description='3D RViz (daron_viz3d.rviz)'),
        A('viz3d_cfg', default_value='', description='[261008] 3D RViz 설정 파일 (빈 값 = rviz/daron_viz3d.rviz). 녹화용 창 배치 사본 등'),
        A('topview_cfg', default_value='', description='[261008] 탑뷰 RViz 설정 파일 (빈 값 = rviz/daron_gui_topview.rviz)'),
        A('ctl_geometry', default_value='', description='[261008] 제어창 위치·크기 "WxH+X+Y" (빈 값 = 창 관리자에 맡김). 녹화 때 창 겹침 방지'),
        A('topview', default_value='true', description='탑뷰 전용 RViz (웨이포인트 찍는 창)'),
        A('gui', default_value='true', description='지상국 제어창'),
        A('follower', default_value='true',
          description='[261012] 경로 추종기 (rover/server/path_follower.py). 기동 때 꺼짐, 제어창 "웨이포인트" 버튼으로 켬'),
        A('follower_params', default_value='',
          description='[261012] 추종기 파라미터 파일 (빈 값 = rover/server/config/follower.yaml)'),
        A('follower_pose_stamp', default_value='',
          description='[261012] 추종기 pose_stamp_source 덮어쓰기 (빈 값 = follower.yaml 값). '
                      'receive = 수신 간격으로 끊김 판정 (시계 동기 무관) | header = pose stamp 나이 (RADXA 시계 동기 필요)'),
        A('pin_gallery', default_value='true', description='CDA 핀 InteractiveMarker + 대표 이미지 저장'),
        A('rim', default_value='true', description='pin_gallery 가 대표 박스로 림 선명도 계산 (정보 창 표시)'),
        A('crater', default_value='true', description='[261009] pin_gallery 가 대표 프레임 depth 로 크레이터 경사·깊이·메시 계산 (정보 창 + 3D 보기)'),
        A('crater_finalize_timeout', default_value='2.5',
          description='[261009] 마지막 검출 뒤 이 초만큼 새 검출이 없으면 관측 종료 → 대표 이미지 확정 → geometry 1회 계산'),
        A('crater_diameter_source', default_value='pin',
          description='[261009] 직경: pin = 기존 bbox 가로폭×depth(요청서, pin_node 와 같음) | code = 전달본 bbox 접원. 둘 다 결과에 기록'),
        A('degradation', default_value='true',
          description='[261013] 열화도 순번 (경사 P95 0.5 + 림 S 0.5, gui/degradation). crater:=true 필요. 정보 창 순번 → 전체 순서'),
        A('degradation_reference', default_value='',
          description='[261013] 고정 기준 reference.json (전달본 python -m crater_rank calibrate). 빈 값 = 이번 미션 범위로 임시 기준'),
        A('crater_z_exaggeration', default_value='5.0',
          description='[261009] 시험장 Z 증폭 배율(실제 크레이터 대비). 기록만 — 측정값·메시는 측정 높이 그대로'),
        # 아래 셋은 3dmap 인자와 이름·뜻이 같다 (여기 기본값이 우선)
        A('cda', default_value='pins',
          description='3dmap 의 cda. pins = pin_node 만 (YOLO 노드는 따로) | fake | false'),
        A('planner', default_value='true', description='3dmap 의 planner (경로계획 노드)'),
        A('odom_topic', default_value='/odometry/filtered_health',
          description='포즈 소스 — 맵·판정·핀·제어창 거리 표시가 같은 값을 쓴다'),
        # 3dmap 의 out_dir(맵 저장 폴더)과 겹치지 않게 다른 이름
        A('gui_out_dir', default_value='',
          description='핀 이미지·pins_info.json 저장 폴더. 비우면 ~/daron_gui_runs/<시각> (이미지는 그 아래 pins/)'),
        A('use_sim_time', default_value='false', description='bag 재생 시 true (daron_viz bag_path 와 맞춤)'),
        # [261008] 웨이포인트 기록·불러오기. session_name·bag_dir 은 3dmap 인자와 이름·뜻·기본값이 같다
        A('session_name', default_value='',
          description='daron_viz·3dmap 의 session_name 과 같게. 웨이포인트 기록이 <bag_dir>/<session_name> 에 간다'),
        A('bag_dir', default_value=os.path.join(DATA_ROOT, 'daron_bags'), description='3dmap 의 bag_dir (세션 폴더의 부모)'),
        A('session_dir', default_value='',
          description='waypoints.jsonl 폴더. 비우면 session_name 이 있으면 <bag_dir>/<session_name>, 없으면 gui_out_dir'),
        A('waypoint_frame', default_value='map',
          description='웨이포인트 저장·불러오기·플래너 목표 좌표계 (기본 map). 다른 좌표계 클릭은 거부'),
        A('map_id', default_value='', description='이 세션 지도 이름(선택). 불러온 파일과 다르면 경고'),
        A('load_waypoints_file', default_value='',
          description='기동 시 지도에 표시할 waypoints.jsonl (표시만, 목록에 자동으로 안 넣음)'),
        A('rover_ns', default_value='/rover', description='[261010] 로버 노드 이름 (radxa.py 기본 /rover). 수동 키보드 조종 송신 대상'),
        A('max_steer_deg', default_value='',
          description='[261010] 수동 조종 차량 유효조향각 한계 [deg] (빈 값 = 43.003, 로버도 자체 한계로 다시 자름)'),
        A('image_click', default_value='true',
          description='[261011] 제어창 "영상에서 찍기" (D405 infra1 창, 누르고 드래그 = 위치 + 도착 방향). false 면 버튼·구독 없음'),
        A('wp_pos_tol_m', default_value='0.10', description='[261011] 웨이포인트 원판 반경 = 도착 위치 허용 [m] (표시용)'),
        A('wp_yaw_tol_deg', default_value='10.0', description='[261011] 웨이포인트 부채꼴 = 도착 방위 허용 ±[deg] (표시용)'),
        A('lc_resync', default_value='true',
          description='[261011] 루프 클로저 재동기화: 피쳐맵(3dmap)·핀·이번 세션 웨이포인트를 보정 궤적에 맞춰 옮김. '
                      'daron_viz 의 lc_resync 도 true 여야 이력이 나온다. false = 옛 동작'),
        A('lc_viz', default_value='true',
          description='[261015] 루프 클로저 보정 표시 노드(nodes/gui/lc_viz_node.py, 3D RViz "루프 클로저 보정" 그룹). '
                      '표시 전용 — 끄면 보정은 그대로 되고 빨강 표시만 없음'),
        OpaqueFunction(function=_setup),
    ])
