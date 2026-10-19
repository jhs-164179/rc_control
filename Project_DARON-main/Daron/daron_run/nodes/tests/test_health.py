#!/usr/bin/python3
"""lib/health.py 단위 테스트 — ROS 없이 실행: /usr/bin/python3 nodes/tests/test_health.py (pytest 있으면 pytest 도 됨).
[261016] local_test/health_v2/tests 에서 repo 로 옮김 + 미수신(never)·끊김 직후 회전 창 테스트 추가."""
import math
import os
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, os.environ.get('HEALTH_NODES_DIR', str(Path(__file__).resolve().parents[1])))   # 검사 대상 = nodes/
from lib.health import (DEGRADED, FAIL, OK, HealthMonitor, HealthParams, JumpRemover, cov_scale,  # noqa: E402
                        estimate_extrinsic, so3_exp, so3_log)
from lib.rotations import mat_to_quat  # noqa: E402

G = np.array([0.0, 0.0, 9.81])
R_BI = so3_exp(np.radians([3.9, 2.9, -0.5]))      # real_output 추정값과 비슷한 장착 회전
BIAS = np.radians([-0.04, -0.02, 0.02])           # 기동 정지 창에서 추정될 자이로 bias


def _omega(t, yaw_amp=0.15):
    """body 각속도 [rad/s]: 15 s 정지 뒤 yaw·pitch 가 섞인 주행."""
    if t < 16.0:
        return np.zeros(3)
    return np.array([0.02 * math.sin(0.7 * t), 0.05 * math.sin(0.3 * t), yaw_amp * math.sin(0.5 * t)])


def _run(t_end, imu_ok=lambda t: True, imu_burst=lambda t: False, slam_ok=lambda t: True,
         slam_drift_dps=0.0, drift_from=1e9, ahrs_jump_at=None, params=None, R_bi=R_BI,
         slam_jump_at=None, slam_jump_m=6.0, slam_cut=lambda t: False, yaw_amp=0.15):
    """합성 데이터로 HealthMonitor 를 돌려 (t, report) 목록을 돌려준다."""
    # 합성 자이로에는 BIAS 를 넣었으므로 원시 입력처럼 'startup' 으로 뺀다
    hm = HealthMonitor(params or HealthParams(gyro_bias_mode="startup"), R_body_imu=R_bi)
    R = np.eye(3)                     # 참 body 자세
    drift = np.eye(3)
    dt = 0.001
    reports, pending = [], []
    t = 0.0
    k = 0
    while t < t_end:
        w = _omega(t, yaw_amp)
        R = R @ so3_exp(w * dt)
        if t >= drift_from:
            drift = drift @ so3_exp(np.radians([0.0, 0.0, slam_drift_dps]) * dt)
        if k % 10 == 0:                                   # IMU 100 Hz
            R_imu = R @ R_bi if ahrs_jump_at is None or t < ahrs_jump_at else R @ so3_exp(np.radians([0, 0, 20])) @ R_bi
            sample = (t, R_bi.T @ w + BIAS, R_imu.T @ G, mat_to_quat(R_imu))
            if imu_burst(t):
                pending.append(sample)                    # 보류했다가 몰아서, stamp 는 도착 시각으로 압축
            elif imu_ok(t):
                for i, s in enumerate(pending):
                    hm.imu_sample(t - 0.001 * (len(pending) - i), s[1], s[2], s[3])
                pending.clear()
                hm.imu_sample(*sample)
        if k % 33 == 0 and not slam_cut(t):               # SLAM ~30 Hz (slam_cut = 영상 끊김: 메시지 자체가 없음)
            Rs = R @ drift
            pos = np.array([slam_jump_m, 0.0, 0.0]) if slam_jump_at is not None and t >= slam_jump_at else np.zeros(3)
            hm.slam_sample(t, pos, mat_to_quat(Rs), 0.01 if slam_ok(t) else 1e6)
        if k % 100 == 0:                                  # evaluate 10 Hz
            reports.append(hm.evaluate(t))
        t += dt
        k += 1
    return hm, reports


def _levels(reports, t0, t1):
    ins = {r.ins_level for r in reports if t0 <= r.t < t1}
    slam = {r.slam_level for r in reports if t0 <= r.t < t1}
    return ins, slam


def test_so3_roundtrip():
    for v in (np.array([0.1, -0.2, 0.3]), np.array([0.0, 0.0, math.pi - 1e-6]), np.zeros(3)):
        assert np.allclose(so3_log(so3_exp(v)), v, atol=1e-6)


def test_cov_scale():
    assert cov_scale(1.0) == 1.0
    assert cov_scale(0.5) == 4.0
    assert cov_scale(0.0) is None


def test_clean_run_is_all_ok_and_residual_small():
    hm, rep = _run(60.0)
    ins, slam = _levels(rep, 0, 60)
    assert ins == {OK} and slam == {OK}
    for w, log in hm.rot_log.items():
        assert log, f"{w}s 창이 한 번도 평가되지 않음"
        assert max(e[1] for e in log) < 0.5, (w, max(e[1] for e in log))


def test_imu_stall_blames_ins_not_slam():
    # real_output 재현: 30 s 에 IMU 가 멈춘다
    _, rep = _run(45.0, imu_ok=lambda t: t < 30.0)
    ins, slam = _levels(rep, 30.3, 45)
    assert ins == {FAIL}
    assert slam == {OK}
    assert any("dead" in r.ins_reasons for r in rep if r.t > 31.0)


def test_imu_burst_degrades_ins_and_skips_cross_check():
    _, rep = _run(40.0, imu_burst=lambda t: 25.0 <= t < 25.15)
    ins, slam = _levels(rep, 25.1, 26.0)
    assert DEGRADED in ins or FAIL in ins
    assert slam == {OK}


def test_hysteresis_recovers_after_recover_s():
    _, rep = _run(40.0, imu_ok=lambda t: not (25.0 <= t < 25.5))
    at = {round(r.t, 1): r.ins_level for r in rep}
    assert at[25.4] == FAIL
    assert at[26.5] != OK            # 복귀 직후는 아직 (recover_s = 2 s)
    assert at[28.5] == OK


def test_slam_slow_yaw_drift_detected_by_10s_window():
    # 0.5°/s — 1 s 창(임계 3°)엔 안 걸리고 10 s 창에서 5° 로 걸려야 한다
    hm, rep = _run(70.0, slam_drift_dps=0.5, drift_from=30.0)
    assert max(e[1] for e in hm.rot_log[1.0]) < 3.0
    ins, slam = _levels(rep, 45, 50)
    assert slam == {DEGRADED}
    assert any("rot_vs_gyro" in r.slam_reasons for r in rep)
    # 불일치가 both_after_s(10 s) 넘게 이어지면 INS 도 DEGRADED
    ins_late, _ = _levels(rep, 65, 70)
    assert DEGRADED in ins_late


def test_slam_lost_is_fail_and_ins_untouched():
    _, rep = _run(40.0, slam_ok=lambda t: not (25.0 <= t < 27.0))
    ins, slam = _levels(rep, 25.1, 27.0)
    assert slam == {FAIL}
    assert ins == {OK}


def test_ahrs_jump_flags_ins():
    _, rep = _run(40.0, ahrs_jump_at=30.0)
    ins, _ = _levels(rep, 30.0, 32.0)
    assert DEGRADED in ins
    assert any("ahrs_vs_gyro" in r.ins_reasons for r in rep)


def test_estimate_extrinsic_recovers_rotation_and_offset():
    # 실제 bag 처럼 정지로 시작(추정 함수가 앞 5% 를 bias 로 뺀다) + 빠른 성분이 있는 회전
    def om(t):
        if t < 5.0:
            return np.zeros(3)
        return np.array([0.05 * math.sin(1.9 * t), 0.08 * math.sin(1.3 * t + 1), 0.15 * math.sin(2.3 * t)])

    rng = np.random.default_rng(0)
    ts = np.arange(0, 60, 1 / 30)
    ti = np.arange(0, 60, 1 / 100)
    R, Rs, idx = np.eye(3), [], 0
    for tf in np.arange(0, 60, 0.001):
        R = R @ so3_exp(om(tf) * 0.001)
        if idx < len(ts) and tf >= ts[idx]:
            Rs.append(R.copy())
            idx += 1
    lag = 0.02                        # IMU stamp 가 참 시각보다 20 ms 늦다 → 더할 오프셋은 -0.02
    w_imu = np.array([R_BI.T @ om(t - lag) for t in ti]) + BIAS + rng.normal(0, 0.002, (len(ti), 3))
    R_est, off_est, corr, _ = estimate_extrinsic(ts[:len(Rs)], Rs, ti, w_imu)
    assert corr > 0.95, corr
    assert abs(off_est + lag) <= 0.01, off_est
    assert math.degrees(np.linalg.norm(so3_log(R_est.T @ R_BI))) < 1.0


def test_slam_position_jump_is_fail():
    # 260929 실시간 책상 테스트 재현: cuVSLAM 이 LOST 없이 6 m 를 튀었다
    _, rep = _run(40.0, slam_jump_at=30.0)
    _, slam = _levels(rep, 30.0, 31.5)
    assert FAIL in slam
    assert any("jump" in r.slam_reasons for r in rep)


def test_huge_rotation_mismatch_is_fail():
    # 10 s 창에서 수십 도 어긋나면 지속 확인 없이 FAIL (책상 테스트 49°)
    _, rep = _run(60.0, slam_drift_dps=5.0, drift_from=40.0)
    assert any(r.slam_level == FAIL and "rot_fail" in r.slam_reasons for r in rep if r.t > 40.0)


def test_jump_remover_keeps_stream_continuous():
    jr = JumpRemover(0.10)
    outs, jumps = [], []
    for k in range(100):
        x = 0.002 * k + (6.0 if k >= 50 else 0.0)          # 50 번째에 6 m 점프, 나머지는 2 mm/프레임 이동
        T = np.eye(4)
        T[:3, :3] = so3_exp(np.array([0.0, 0.0, 0.001 * k]))
        T[0, 3] = x
        o, j = jr.apply(T)
        outs.append(o[:3, 3].copy())
        jumps.append(j)
    steps = np.linalg.norm(np.diff(outs, axis=0), axis=1)
    assert steps.max() < 0.01, steps.max()                  # 출력에는 점프가 없다
    assert jr.count == 1 and abs(jumps[50] - 6.0) < 0.01
    assert abs(np.linalg.norm(outs[-1] - outs[0]) - 0.002 * 98) < 0.01   # 실제 이동(점프 프레임 1개 제외)은 보존


def test_slam_never_received_is_fail():
    # 261016: D405 가 USB 2.1 로 붙어 infra2 미개방 → SLAM 0 Hz 인데 health 는 영원히 OK 였다
    p = HealthParams(gyro_bias_mode="startup")
    _, rep = _run(30.0, slam_cut=lambda t: t < 22.0, params=p)
    _, slam_early = _levels(rep, 0, p.first_sample_s)
    assert slam_early == {OK}                                   # 기동 여유 동안은 OK (예전과 같음)
    _, slam = _levels(rep, p.first_sample_s + 0.2, 22.0)
    assert slam == {FAIL}
    assert any("never" in r.slam_reasons for r in rep)
    _, slam_after = _levels(rep, 22.2, 30.0)
    assert slam_after == {OK}                                   # 들어오기 시작하면 바로 풀린다 (히스테리시스 아님)


def test_ins_never_received_is_fail():
    p = HealthParams(gyro_bias_mode="startup")
    _, rep = _run(20.0, imu_ok=lambda t: False, params=p)
    ins, slam = _levels(rep, p.first_sample_s + 0.2, 20.0)
    assert ins == {FAIL}
    assert any("never" in r.ins_reasons for r in rep)
    assert slam == {OK}


def test_rotation_window_after_video_cut_no_false_fail():
    # 261016: 회전 중 영상이 1.5 s 끊겼다 돌아오면 창 시작점이 끊김 이전 SLAM 샘플로 잡혀 rot_fail 오경보.
    # yaw 0.4 rad/s 진폭, 회전 최고점(28.27 s)을 덮는 끊김. SLAM 자체는 정확하다.
    _, rep = _run(40.0, yaw_amp=0.4, slam_cut=lambda t: 27.5 <= t < 29.0)
    bad = [r for r in rep if r.t > 29.0 and ("rot_fail" in r.slam_reasons or "rot_vs_gyro" in r.slam_reasons)]
    assert not bad, [(round(r.t, 1), r.slam_reasons) for r in bad[:3]]
    _, slam = _levels(rep, 30.0, 40.0)
    assert slam == {OK}


if __name__ == "__main__":
    fails = 0
    for name, fn in list(globals().items()):
        if name.startswith("test_") and callable(fn):
            try:
                fn()
                print(f"PASS {name}")
            except AssertionError as e:
                fails += 1
                print(f"FAIL {name}: {e}")
    sys.exit(1 if fails else 0)
