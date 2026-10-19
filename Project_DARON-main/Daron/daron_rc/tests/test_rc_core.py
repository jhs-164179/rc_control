"""rc_core (261015) — PPM 디코더·RC 중재. ROS·하드웨어 없이: python3 -m pytest Daron/daron_rc/tests"""
import os
import sys

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'rover'))
from rc_core import PpmDecoder, RcArbiter, RcConfig, config_with  # noqa: E402

C = 1500
RC, MID, AUTO = 1000, 1500, 2000


# ============================================================
# PPM 디코더
# ============================================================
def edges(frames, sync_us=6000, t0_ns=1_000_000_000):
    """채널 값 목록들 -> 상승 에지 타임스탬프 [ns]. 각 프레임 앞에 동기 간격."""
    t = t0_ns
    out = [t]
    for fr in frames:
        t += sync_us * 1000
        out.append(t)
        for v in fr:
            t += v * 1000
            out.append(t)
    t += sync_us * 1000
    out.append(t)             # 마지막 프레임을 닫는 동기
    return out


def decode(dec, ts):
    return [f for f in (dec.feed(t) for t in ts) if f is not None]


FR = [1000, 1500, 1600, 1400, 1100, 1200, 1300, 1900]


def test_full_frames_decoded():
    dec = PpmDecoder(channels=8)
    got = decode(dec, edges([FR, FR, FR]))
    assert got == [FR, FR, FR]
    assert dec.frames == 3 and dec.rejected == 0


def test_first_partial_frame_dropped():
    """프레임 중간(CH5 직전)부터 듣기 시작 -> 그 조각을 CH1~ 로 받으면 안 된다."""
    ts = edges([FR, FR])
    start = 1 + 1 + 4          # 첫 동기 에지 + CH1~CH4 에지를 건너뛴 위치
    dec = PpmDecoder(channels=8)
    got = decode(dec, ts[start:])
    assert got == [FR]         # 두 번째(완전한) 프레임만


def test_glitch_edge_drops_whole_frame():
    """프레임 중간 노이즈 에지 -> 그 프레임 통째로 버림. 남은 조각이 CH1 로 밀려 들어오면 안 된다."""
    ts = edges([FR, FR, FR])
    i = 1 + 1 + 3              # 첫 프레임 CH3 끝 에지
    ts = ts[:i] + [ts[i] + 150_000] + ts[i:]   # 150 µs 뒤 가짜 에지
    ts = sorted(ts)
    dec = PpmDecoder(channels=8)
    got = decode(dec, ts)
    assert got == [FR, FR]
    assert dec.rejected == 1


def test_missing_edge_drops_frame():
    ts = edges([FR, FR])
    del ts[1 + 1 + 2]          # 첫 프레임 CH3 끝 에지 빠짐 -> 1600+1400 = 3000 µs (max 초과, 동기 아님)
    dec = PpmDecoder(channels=8)
    assert decode(dec, ts) == [FR]


def test_missing_edge_looking_like_sync_never_shifts_channels():
    """빠진 에지 때문에 두 채널 합(1900+1900 µs)이 동기보다 길어지면 가짜 동기가 된다.
    그 앞뒤 조각은 개수가 모자라므로 버려져야 한다 (뒤 조각이 CH1 로 밀려 들어오면 안 됨)."""
    fr = [1500, 1900, 1900, 1500, 1500, 1500, 1500, 1500]
    ts = edges([fr, fr, fr])
    del ts[1 + 1 + 1]          # 첫 프레임 CH2 끝 에지 빠짐 -> 3800 µs 간격
    dec = PpmDecoder(channels=8)
    got = decode(dec, ts)
    assert got == [fr, fr]
    assert all(len(f) == 8 for f in got)


def test_stale_frame_not_emitted_after_signal_gap():
    """송신기 OFF 직전 프레임(CH1 AUTO)이 신호 복귀 때 '새 값'으로 나가면 안 된다 (e2e 에서 발견)."""
    auto = [2000] + FR[1:]
    rc = [1000] + FR[1:]
    before = edges([auto, auto])[:-1]           # 마지막 프레임을 닫는 동기 에지 없이 끊김
    after = edges([rc, rc], t0_ns=before[-1] + 2_000_000_000)   # 2 s 뒤 복귀
    dec = PpmDecoder(channels=8)
    got = decode(dec, before + after)
    assert got == [auto, rc, rc]                # 끊기기 전 두 번째 auto 프레임은 버려짐
    # 대조: 간격 상한이 없으면 묵은 auto 가 복귀 순간 나간다 (고치기 전 동작)
    old = PpmDecoder(channels=8, max_gap_us=1e12)
    assert decode(old, before + after) == [auto, auto, rc, rc]


def test_wrong_channel_count_rejected():
    dec = PpmDecoder(channels=8)
    got = decode(dec, edges([FR[:6], FR[:6]]))
    assert got == [] and dec.rejected == 2 and dec.last_len == 6
    dec6 = PpmDecoder(channels=6)
    assert decode(dec6, edges([FR[:6]])) == [FR[:6]]


def test_too_many_channels_rejected():
    dec = PpmDecoder(channels=4)
    assert decode(dec, edges([FR])) == []


# ============================================================
# RC 중재
# ============================================================
class Sim:
    """20 Hz 로 채널을 넣고 step 을 부르는 작은 하네스."""

    def __init__(self, **cfg):
        cfg.setdefault('actuation', True)
        self.a = RcArbiter(RcConfig(**cfg))
        self.t = 100.0
        self.events = []

    def run(self, secs, ch, dt=0.05, **auto):
        out = None
        n = max(1, int(round(secs / dt)))
        for _ in range(n):
            self.t += dt
            if ch is not None:
                self.a.update_channels(ch, self.t)
            out = self.a.step(self.t, **auto)
            self.events += out.events
        return out

    def auto_kw(self, enable=True):
        return dict(auto_enable=enable, auto_rpm_t=self.t, auto_steer_t=self.t)


def test_no_link_is_stop():
    s = Sim()
    o = s.run(1.0, None)
    assert (o.source, o.mode, o.link) == ('STOP', 'idle', False)


def test_rc_arm_requires_stable_then_neutral():
    s = Sim()
    o = s.run(0.2, [RC, C, C, C])
    assert o.mode == 'idle' and not o.armed           # CH1 안정화(0.3 s) 전
    o = s.run(0.5, [RC, C, C, C])
    assert o.source == 'RC'
    o = s.run(0.5, [RC, C, C, C])
    assert o.armed and o.mode == 'manual' and o.drive == 0


def test_rc_not_armed_while_throttle_held():
    s = Sim()
    o = s.run(2.0, [RC, 1800, C, C])                   # 방아쇠를 당긴 채 RC 선택
    assert o.source == 'RC' and not o.armed and o.drive == 0 and o.mode == 'idle'
    o = s.run(0.6, [RC, C, C, C])
    assert o.armed
    o = s.run(0.1, [RC, 1800, C, C])
    assert o.drive == 1


def test_arm_band_never_inside_drive_deadzone():
    """arm 이 되는 스로틀 위치는 항상 구동 0 이어야 한다 (첨부본: arm 90 > 데드존 70 버그)."""
    with pytest.raises(ValueError):
        RcConfig(neutral_arm_us=90, drive_deadzone_us=70).validate()
    s = Sim()
    edge = C + s.a.cfg.neutral_arm_us                   # arm 이 허용되는 가장 끝
    o = s.run(1.5, [RC, edge, C, C])
    assert o.armed and o.drive == 0


def test_drive_direction_and_reverse():
    s = Sim()
    s.run(1.5, [RC, C, C, C])
    assert s.run(0.1, [RC, 1700, C, C]).drive == 1
    assert s.run(0.1, [RC, 1300, C, C]).drive == -1
    s2 = Sim(ch2_reverse=True)
    s2.run(1.5, [RC, C, C, C])
    assert s2.run(0.1, [RC, 1700, C, C]).drive == -1


def test_steer_continuous_at_deadzone_and_full_scale():
    s = Sim()
    s.run(1.5, [RC, C, C, C])
    dz = s.a.cfg.steer_deadzone_us
    assert s.run(0.05, [RC, C, C + dz, C]).steer_frac == 0.0
    small = s.run(0.05, [RC, C, C + dz + 5, C]).steer_frac
    assert 0 < small < 0.03                             # 데드존 경계에서 튀지 않는다
    assert s.run(0.05, [RC, C, 2000, C]).steer_frac == pytest.approx(1.0)
    assert s.run(0.05, [RC, C, 1000, C]).steer_frac == pytest.approx(-1.0)
    assert s.run(0.05, [RC, C, 2100, C]).steer_frac == 1.0


def test_ch4_only_when_enabled():
    s = Sim()
    s.run(1.5, [RC, C, C, C])
    assert s.run(0.1, [RC, C, C, 2000]).jog == 0
    s4 = Sim(ch4_enable=True)
    s4.run(1.5, [RC, C, C, C])
    assert s4.run(0.1, [RC, C, C, 2000]).jog == 1
    assert s4.run(0.1, [RC, C, C, 1000]).jog == -1
    assert s4.run(0.1, [RC, C, C, C + 140]).jog == 0


def test_link_loss_stops_and_requires_rearm():
    s = Sim()
    s.run(1.5, [RC, C, C, C])
    assert s.run(0.1, [RC, 1800, C, C]).drive == 1
    o = s.run(0.3, None)                                 # 0.25 s 넘게 새 값 없음
    assert (o.source, o.mode, o.drive, o.link) == ('STOP', 'idle', 0, False)
    o = s.run(1.0, [RC, 1800, C, C])                     # 방아쇠 당긴 채 복귀 -> arm 안 됨
    assert not o.armed and o.drive == 0
    s.a.update_channels([], s.t)                         # PPM 노드의 빈 배열 = 즉시 끊김
    assert s.a.step(s.t).link is False


def test_ch1_middle_is_stop():
    s = Sim()
    s.run(1.5, [RC, C, C, C])
    o = s.run(0.1, [MID, 1800, C, C])
    assert o.source == 'STOP' and o.mode == 'idle' and o.drive == 0


def test_ch1_bounce_disarms():
    """RC -> AUTO -> RC 를 0.3 s 안에 튕겨도 arm 이 살아남아 바로 달리면 안 된다."""
    s = Sim()
    s.run(1.5, [RC, C, C, C])
    s.run(0.1, [AUTO, C, C, C])
    o = s.run(0.4, [RC, 1800, C, C])
    assert not o.armed and o.drive == 0


def test_gui_stop_latch():
    s = Sim()
    s.run(1.5, [RC, C, C, C])
    s.a.request('STOP')
    o = s.run(2.0, [RC, C, C, C])                        # 중립 유지해도 래치는 안 풀린다
    assert o.latched and o.source == 'STOP' and o.mode == 'idle'
    assert 'AUTO' in s.a.request('AUTO') or '거부' in s.a.request('AUTO')
    s.a.request('RC')
    o = s.run(0.1, [RC, C, C, C])
    assert not o.latched and not o.armed
    o = s.run(1.0, [RC, C, C, C])
    assert o.armed and o.mode == 'manual'


def test_auto_needs_all_conditions():
    s = Sim()
    o = s.run(1.5, [AUTO, C, C, C], auto_enable=True)    # 추종기 지령 없음
    assert o.source == 'AUTO' and o.auto_ready and o.mode == 'idle'
    o = s.run(0.1, [AUTO, C, C, C], **s.auto_kw(enable=False))
    assert o.mode == 'idle'
    o = s.run(0.1, [AUTO, C, C, C], **s.auto_kw())
    assert o.mode == 'auto' and o.drive == 0 and o.steer_frac == 0.0
    old = s.t
    o = s.run(0.6, [AUTO, C, C, C], auto_enable=True, auto_rpm_t=old, auto_steer_t=s.t)
    assert o.mode == 'idle'                              # rpm 지령이 auto_timeout 넘게 끊김


def test_auto_throttle_override_stops_until_neutral():
    s = Sim()
    s.run(1.5, [AUTO, C, C, C], **s.auto_kw())
    assert s.run(0.05, [AUTO, C, C, C], **s.auto_kw()).mode == 'auto'
    o = s.run(0.1, [AUTO, 1800, C, C], **s.auto_kw())    # override_hold 0.15 s 전
    assert o.mode == 'auto'
    o = s.run(0.2, [AUTO, 1800, C, C], **s.auto_kw())
    assert o.mode == 'idle' and not o.auto_ready
    o = s.run(0.3, [AUTO, C, C, C], **s.auto_kw())
    assert o.mode == 'idle'
    o = s.run(0.4, [AUTO, C, C, C], **s.auto_kw())
    assert o.mode == 'auto'


def test_auto_ignores_rc_steer_and_drive():
    s = Sim(ch4_enable=True)
    s.run(1.5, [AUTO, C, C, C], **s.auto_kw())
    o = s.run(0.1, [AUTO, C + 50, 2000, 2000], **s.auto_kw())
    assert o.mode == 'auto' and o.steer_frac == 0.0 and o.drive == 0 and o.jog == 0


def test_dry_reports_would_but_outputs_zero():
    s = Sim(actuation=False, ch4_enable=True)
    s.run(1.5, [RC, C, C, C])
    o = s.run(0.1, [RC, 1800, 2000, 1000])
    assert o.would_mode == 'manual' and o.would_drive == 1 and o.would_jog == -1
    assert o.would_steer_frac == pytest.approx(1.0)
    assert (o.mode, o.drive, o.jog, o.steer_frac) == ('idle', 0, 0, 0.0)
    assert 'DRY' in o.reason


def test_out_of_range_channel_is_no_link():
    s = Sim()
    s.run(1.5, [RC, C, C, C])
    o = s.run(0.05, [RC, 3000, C, C])
    assert not o.link and o.mode == 'idle'


def test_config_with_validates_and_keeps_old():
    cfg = RcConfig()
    with pytest.raises(ValueError):
        config_with(cfg, mode_low_us=1600)
    assert cfg.mode_low_us == 1300
    assert config_with(cfg, steer_deadzone_us=40).steer_deadzone_us == 40


def test_events_are_reported():
    s = Sim()
    s.run(1.5, [RC, C, C, C])
    s.run(0.1, [MID, C, C, C])
    joined = ' / '.join(s.events)
    assert 'RC 모드' in joined and 'armed' in joined and 'STOP' in joined
