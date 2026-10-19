#!/usr/bin/env python3
"""DARON 로버(RADXA 도커) 쪽 한 번에 — 로버 제어 노드 + 카메라 + CV7. 작성 261010.

지금까지 RADXA 에서 터미널 셋으로 따로 띄우던 것을 launch 하나로 묶었다.

    R1 카메라   ros2 launch realsense2_camera rs_launch.py camera_namespace:=radxa ...   (README '원격 실행')
    R2 CV7      ros2 launch microstrain_inertial_driver microstrain_launch.py params_file:=/workspace/daron_cv7.yml ...
    R3 로버     python3 radxa.py      (제어팀 DARON_keyboard_ctrl → 이 폴더 rover/ 로 합침)

    RADXA:   ros2 launch <daron_run>/launch/daron_rover.launch.py [susp_profile:=/workspace/DARON_keyboard_ctrl/profiles/rover.json]
    지상국:  daron_viz.launch.py source:=remote ...   +   daron_gui.launch.py ...   (제어창 '수동' 버튼 = 키보드 조종)
             제어창 대신 pygame 콘솔을 쓰려면  python3 <daron_run>/rover/console.py  (둘을 동시에 켜지 말 것)

띄우는 것 (끄려면 각 인자 :=false)
  rover    rover/radxa.py — /rover 노드. 굴절·서스펜션(XM430) + 조향(L12) + 구동(VESC).
           /rover/diagnostics (배터리 포함) · /rover/battery (BatteryState) · /joint_states 발행.
           cwd 는 rover_dir. 프로파일·config/front.json·back.json·logs 는 radxa.py 가 자기 파일 위치 기준으로 찾는다.
  camera   realsense2_camera rs_launch.py, camera_namespace:=radxa (지상국 image_relay 가 /radxa/camera/* 를 받는다)
  imu      microstrain_inertial_driver microstrain_launch.py, params_file:=cv7_params (namespace /)

⚠ 서스펜션 프로파일(캘리브레이션 결과 profiles/rover.json)은 RADXA 마다 다르고 이 폴더에 없다.
  RADXA 의 기존 파일을 susp_profile:=<절대경로> 로 주거나 rover/profiles/ 에 복사한다. 없으면 기동 전에 멈춘다.
  처음이면 RADXA 에서:  cd <daron_run>/rover && python3 calibrate.py --auto-zero --out profiles/rover.json
⚠ ROS_DOMAIN_ID 는 지상국(slam_env.sh, 42)과 같아야 한다. ros_domain_id:=42 로 이 launch 의 모든 프로세스에 넣을 수 있다.

로버 노드 파라미터
  susp_profile     프로파일 경로 (빈 값 = rover/profiles/rover.json). 상대경로는 rover_dir 기준
  vesc_enable / l12_enable / susp_mock   radxa.py 같은 이름 파라미터 그대로
  rover_params     그 밖의 radxa.py 파라미터, 'a:=1 b:=2' 처럼 공백으로 구분 (예: rover_params:='jog_limit_deg:=30.0 battery_warn_pct:=25.0')
"""
import os
import shlex

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, LogInfo, OpaqueFunction,
                            SetEnvironmentVariable)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
ROVER_DIR = os.path.join(ROOT, 'rover')
DEFAULT_PROFILE = os.path.join('profiles', 'rover.json')
# radxa.py 종료 순서(구동 0 → VESC off → L12 중립 대기 최대 5 s → 서스펜션 토크 유지)가 끝나기 전에
# launch 가 SIGTERM/SIGKILL 로 넘어가면 L12 가 꺾인 채 남는다. launch 기본 5 s 보다 길게.
ROVER_SIGTERM_S = '15'


def _setup(context):
    def cfg(name):
        return LaunchConfiguration(name).perform(context).strip()

    on = lambda k: cfg(k).lower() in ('true', '1', 'yes')  # noqa: E731
    acts = []

    dom = cfg('ros_domain_id')
    if dom:
        acts.append(SetEnvironmentVariable('ROS_DOMAIN_ID', dom))
    elif not os.environ.get('ROS_DOMAIN_ID'):
        acts.append(LogInfo(msg='⚠ ROS_DOMAIN_ID 가 비어 있습니다 (도메인 0). 지상국(42)과 다르면 토픽이 안 오갑니다 — '
                                'ros_domain_id:=42'))

    if on('rover'):
        rover_dir = os.path.abspath(os.path.expanduser(cfg('rover_dir') or ROVER_DIR))
        script = os.path.join(rover_dir, 'radxa.py')
        if not os.path.exists(script):
            raise RuntimeError(f'radxa.py 가 없습니다: {script} (rover_dir 확인)')
        profile = os.path.expanduser(cfg('susp_profile') or DEFAULT_PROFILE)
        full = profile if os.path.isabs(profile) else os.path.join(rover_dir, profile)
        mock = on('susp_mock')
        if not os.path.exists(full) and not mock:
            raise RuntimeError(
                f'서스펜션 프로파일이 없습니다: {full}\n'
                f'  RADXA 에 있는 기존 파일을 쓰려면: susp_profile:=/workspace/DARON_keyboard_ctrl/profiles/rover.json\n'
                f'  처음이면: cd {rover_dir} && python3 calibrate.py --auto-zero --out profiles/rover.json\n'
                f'  하드웨어 없이 점검: susp_mock:=true vesc_enable:=false l12_enable:=false')
        # -p 값은 YAML 로 읽힌다. 경로를 따옴표로 고정 (공백·':' 가 든 경로, daron_gui.launch.py q() 와 같은 이유)
        params = ["susp_profile:='" + full.replace("'", "''") + "'",
                  f'susp_mock:={"true" if mock else "false"}',
                  f'vesc_enable:={"true" if on("vesc_enable") else "false"}',
                  f'l12_enable:={"true" if on("l12_enable") else "false"}']
        if mock and not os.path.exists(full):
            params.append('susp_require_profile:=false')
        for item in shlex.split(cfg('rover_params')):
            if ':=' not in item:
                raise RuntimeError(f"rover_params 항목은 이름:=값 형식이어야 합니다: {item!r}")
            params.append(item)
        cmd = [cfg('python') or 'python3', script, '--ros-args']
        for p in params:
            cmd += ['-p', p]
        acts.append(ExecuteProcess(name='rover', cmd=cmd, cwd=rover_dir, output='screen', emulate_tty=True,
                                   sigterm_timeout=ROVER_SIGTERM_S, sigkill_timeout='5'))
        acts.append(LogInfo(msg=f'[rover] {script}  프로파일={full}  ' + ' '.join(params[1:])))

    if on('camera'):
        acts.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(_pkg_launch('realsense2_camera', 'rs_launch.py')),
            launch_arguments={
                'camera_namespace': cfg('camera_namespace'),
                'enable_infra1': 'true', 'enable_infra2': 'true', 'enable_depth': 'true', 'enable_color': 'false',
                'depth_module.infra_profile': cfg('camera_profile'),
                'depth_module.depth_profile': cfg('camera_profile'),
                'depth_module.enable_auto_exposure': 'false',
                'depth_module.exposure': cfg('camera_exposure'),
                'depth_module.gain': cfg('camera_gain'),
            }.items()))

    if on('imu'):
        params_file = os.path.expanduser(cfg('cv7_params'))
        if not os.path.exists(params_file):
            raise RuntimeError(f'CV7 설정 파일이 없습니다: {params_file} '
                               f'(묶음의 radxa_docker/daron_cv7.yml = daron_run/config/daron_cv7.yml 에서 port 만 /dev/ttyACM0)')
        acts.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(_pkg_launch('microstrain_inertial_driver', 'microstrain_launch.py')),
            launch_arguments={'params_file': params_file, 'namespace': '/',
                              'configure': 'true', 'activate': 'true'}.items()))

    acts.append(LogInfo(msg=f'[daron_rover] 로버={on("rover")} 카메라={on("camera")} CV7={on("imu")} '
                            f'ROS_DOMAIN_ID={dom or os.environ.get("ROS_DOMAIN_ID", "(없음)")}'))
    return acts


def _pkg_launch(pkg, name):
    # 쓰는 경우에만 찾는다 — camera:=false 면 realsense 패키지가 없는 도커에서도 떠야 한다
    from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
    try:
        return os.path.join(get_package_share_directory(pkg), 'launch', name)
    except PackageNotFoundError as e:
        raise RuntimeError(f'ROS 패키지 {pkg} 가 없습니다. 끄려면 해당 인자 :=false') from e


def generate_launch_description():
    A = DeclareLaunchArgument
    return LaunchDescription([
        A('rover', default_value='true', description='로버 제어 노드 rover/radxa.py (/rover)'),
        A('camera', default_value='true', description='RealSense (camera_namespace:=radxa)'),
        A('imu', default_value='true', description='CV7 microstrain 드라이버'),
        A('ros_domain_id', default_value='', description='이 launch 의 모든 프로세스 ROS_DOMAIN_ID (빈 값 = 셸 값). 지상국은 42'),
        # ---- 로버
        A('rover_dir', default_value='', description='radxa.py 폴더 (빈 값 = daron_run/rover)'),
        A('python', default_value='python3', description='radxa.py 를 돌릴 인터프리터 (rclpy·dynamixel_sdk·pyserial 필요)'),
        A('susp_profile', default_value='',
          description='서스펜션 프로파일 (빈 값 = <rover_dir>/profiles/rover.json). 상대경로는 rover_dir 기준'),
        A('vesc_enable', default_value='true', description='구동 VESC (false = 구동 없이, 배터리 잔량도 안 나옴)'),
        A('l12_enable', default_value='true', description='조향 L12'),
        A('susp_mock', default_value='false', description='XM430 없이 코드 경로만 점검'),
        A('rover_params', default_value='', description="그 밖의 radxa.py 파라미터 'a:=1 b:=2'"),
        # ---- 카메라 (README '원격 실행' R1 과 같은 값)
        A('camera_namespace', default_value='radxa', description='지상국 daron_viz remote_cam_prefix(/radxa/camera)와 맞출 것'),
        A('camera_profile', default_value='848x480x30', description='infra·depth 해상도x주기'),
        A('camera_exposure', default_value='8500'),
        A('camera_gain', default_value='16'),
        # ---- CV7 (README '원격 실행' R2)
        A('cv7_params', default_value='/workspace/daron_cv7.yml',
          description='CV7 설정 (daron_cv7.yml 사본, port=/dev/ttyACM0)'),
        OpaqueFunction(function=_setup),
    ])
