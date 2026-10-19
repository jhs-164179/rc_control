"""Linux sysfs PWM channel preparation."""
import errno
import os
import re
import time

PWM_EXPORT_TIMEOUT_S = 2.0            # export 후 채널 디렉터리/권한이 준비되기를 기다리는 시간


_PWM_ATTRS = ("period", "duty_cycle", "enable")


class PwmSetupError(RuntimeError):
    """PWM 채널을 사용 가능한 상태로 만들지 못했을 때. 메시지에 원인별 조치를 담는다."""


def _pwm_channel_ready(pwm_path: str) -> bool:
    """채널 디렉터리가 있고 필수 속성 파일을 모두 쓸 수 있으면 True."""
    return os.path.isdir(pwm_path) and all(
        os.access(os.path.join(pwm_path, a), os.W_OK) for a in _PWM_ATTRS)


def _sysfs_write(path: str, value: str) -> None:
    """sysfs 속성에 한 번에 쓴다. 버퍼링 없이 써야 errno 가 바로 올라온다."""
    fd = os.open(path, os.O_WRONLY)
    try:
        os.write(fd, value.encode())
    finally:
        os.close(fd)


def ensure_pwm_channel(pwm_path: str,
                       timeout_s: float = PWM_EXPORT_TIMEOUT_S) -> str:
    """
    sysfs PWM 채널이 export 되어 있는지 확인하고, 없으면 export 한다.

        pwm_path = /sys/class/pwm/pwmchipN/pwmM

        1) pwmM 이 있고 속성 파일을 쓸 수 있으면  -> 'present'
        2) pwmchipN 이 없으면                     -> 실패 (overlay 미적용. export 로 해결 불가)
        3) M >= npwm 이면                         -> 실패 (없는 채널)
        4) pwmchipN/export 에 M 을 쓴다
        5) pwmM 과 속성 파일이 쓸 수 있게 될 때까지 대기 (udev 권한 적용 지연 대비)
                                                  -> 'exported'

    export 는 채널을 sysfs 에 노출하는 것까지만 한다. period / polarity /
    duty_cycle / enable 은 L12Pair.initialize() 가 이어서 설정한다.
    실패 원인마다 조치가 다르므로 PwmSetupError 메시지를 구분해서 낸다.
    """
    pwm_path = os.path.normpath(pwm_path)
    chip_dir, chan = os.path.split(pwm_path)
    m = re.fullmatch(r"pwm(\d+)", chan)
    if m is None or re.fullmatch(r"pwmchip\d+", os.path.basename(chip_dir)) is None:
        raise PwmSetupError(
            f"PWM 경로 형식이 아닙니다: {pwm_path} (기대: .../pwmchipN/pwmM)")
    idx = int(m.group(1))

    # 1) 이미 준비됨
    if _pwm_channel_ready(pwm_path):
        return "present"

    # 2) 컨트롤러 자체가 없음
    if not os.path.isdir(chip_dir):
        class_dir = os.path.dirname(chip_dir)
        try:
            chips = sorted(c for c in os.listdir(class_dir) if c.startswith("pwmchip"))
        except OSError:
            chips = []
        raise PwmSetupError(
            f"{chip_dir} 가 없습니다. PWM 컨트롤러가 활성화되지 않은 상태라 "
            f"export 로는 해결되지 않습니다. device tree overlay 설정을 확인하십시오. "
            f"(현재 {class_dir}: {chips or '없음'})")

    # 3) 채널 번호 범위
    npwm = None
    try:
        with open(os.path.join(chip_dir, "npwm")) as f:
            npwm = int(f.read().strip())
    except (OSError, ValueError):
        pass                                  # 못 읽어도 export 는 시도한다
    if npwm is not None and idx >= npwm:
        raise PwmSetupError(
            f"{chip_dir} 의 채널 수는 {npwm} 개인데 pwm{idx} 를 요청했습니다.")

    # 4) export. 디렉터리가 이미 있으면(권한만 미적용) 건너뛰고 대기로 간다.
    did_export = False
    if not os.path.isdir(pwm_path):
        export = os.path.join(chip_dir, "export")
        try:
            _sysfs_write(export, str(idx))
            did_export = True
        except OSError as e:
            if e.errno == errno.EBUSY and os.path.isdir(pwm_path):
                pass                          # 확인과 export 사이에 다른 프로세스가 export 함
            elif e.errno == errno.EBUSY:
                raise PwmSetupError(
                    f"{chip_dir} 의 채널 {idx} 를 다른 커널 드라이버가 점유하고 있습니다 "
                    f"(EBUSY). device tree 에서 이 PWM 을 쓰는 노드"
                    f"(pwm-fan, backlight 등)가 있는지 확인하십시오.") from e
            elif e.errno in (errno.EACCES, errno.EPERM):
                raise PwmSetupError(
                    f"{export} 에 쓸 권한이 없습니다. root 로 실행하거나 "
                    f"PWM 용 udev 규칙을 추가하십시오.") from e
            elif e.errno == errno.EROFS:
                raise PwmSetupError(
                    "/sys 가 읽기 전용으로 마운트되어 있습니다. 컨테이너라면 "
                    "--privileged 로 띄우거나 /sys 를 rw 로 마운트하십시오.") from e
            else:
                raise PwmSetupError(
                    f"export 실패: {export} <- {idx} ({e})") from e

    # 5) 채널 디렉터리와 속성 파일 권한이 준비될 때까지 대기
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        if _pwm_channel_ready(pwm_path):
            return "exported" if did_export else "present"
        time.sleep(0.02)

    if os.path.isdir(pwm_path):
        blocked = [a for a in _PWM_ATTRS
                   if not os.access(os.path.join(pwm_path, a), os.W_OK)]
        raise PwmSetupError(
            f"{pwm_path} 는 있지만 {', '.join(blocked)} 에 쓸 수 없습니다 "
            f"({timeout_s:.1f} s 대기). root 로 실행하거나 udev 규칙을 확인하십시오.")
    raise PwmSetupError(
        f"export 후 {timeout_s:.1f} s 안에 {pwm_path} 가 나타나지 않았습니다.")

