#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
rc_core.py — RC8X / R8FM PPM 4채널: PPM 프레임 디코더 + RC/AUTO 중재 상태기계. 작성 261015.

ROS·GPIO·모터 의존이 없다. rc_ppm_node.py(디코더)와 radxa.py(중재)가 import 하고,
tests/test_rc_core.py 가 하드웨어 없이 시간(now)을 직접 넣어 검증한다.

채널 (PPM 1-based 순서)
-----------------------
  CH1  RC/AUTO 선택   ≤ mode_low_us = RC,  ≥ mode_high_us = AUTO,  그 사이 = STOP
  CH2  구동           center ± drive_deadzone_us 밖이면 +1 / -1 (방향 지령, VESC 가 가감속)
  CH3  조향           center ± steer_deadzone_us 밖을 -1 ~ +1 로 다시 펴서 × max_steer_deg
  CH4  굴절 조그      ch4_enable 일 때만. center ± ch4_deadzone_us 밖이면 +1(CCW) / -1(CW)
  각 채널의 부호가 반대면 송신기 Reverse 대신 chN_reverse 로 뒤집을 수 있다 (2·center - 값).

중재 규칙
---------
  - 링크 없음(빈 프레임 / timeout_s 넘게 갱신 없음) 또는 CH1 중간값 → STOP (출력 0, arm 해제)
  - GUI STOP 래치 → STOP. 'RC'(또는 'RESUME') 요청으로만 풀린다. 풀려도 다시 arm 해야 움직인다
  - CH1 이 바뀌면(RC↔AUTO↔STOP) 즉시 arm / auto_ready 를 둘 다 푼다 (바운스로 재arm 없이 재개 금지)
  - CH1 값이 mode_stable_s 동안 같아야 그 모드로 들어간다. 그 동안은 출력 0
  - RC: CH2 를 neutral_arm_us 안에 neutral_hold_s 동안 두면 arm → 수동 조종
  - AUTO: CH2 중립 neutral_hold_s → auto_ready. 그 뒤 auto_enable 이고 rpm·조향 지령이 모두
    auto_timeout_s 안에 들어와 있어야 자율. AUTO 중 CH2 를 override_us 넘게 override_hold_s 동안
    당기면 auto_ready 를 풀어 정지한다 (다시 중립 유지해야 재개)
  - actuation=False(dry) 이면 위 판단은 그대로 하되(would_mode) 실제 출력(mode)은 항상 idle
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field, fields, replace

PPM_MIN_US = 800
PPM_MAX_US = 2200
N_RC = 4                       # 중재에 쓰는 앞 채널 수 (CH1~CH4)


# ============================================================
# PPM 디코더
# ============================================================
class PpmDecoder:
    """
    PPM-Sum 의 같은 방향 에지(상승) 타임스탬프 -> 완전한 프레임.

    PPM 은 에지 간격이 곧 채널 값이다. 채널 간격(min_us~max_us)이 channels 개 이어진 뒤
    sync_gap_us 보다 긴 간격이 오면 프레임 하나다.

    받는 프레임은 '동기 → 정확히 channels 개 → 동기' 하나뿐이다.
      - 시작 직후 첫 동기 전 간격은 버린다 (프레임 중간부터 듣기 시작했을 수 있다)
      - 범위 밖 간격(노이즈 에지, 빠진 에지)이 하나라도 끼면 다음 동기까지 통째로 버린다
      - 개수가 channels 와 다르면 버린다
    부분 프레임을 받으면 채널이 앞으로 밀려 CH2 자리에 다른 채널 값이 들어간다 —
    구동 채널이 바뀌는 것이므로 절대 받지 않는다.

    프레임은 다음 동기 간격을 봐야 확정되므로, 신호가 끊겼다(max_gap_us 넘게 에지 없음) 돌아오면
    끊기기 직전 프레임이 그때서야 나갈 수 있다. 그런 간격이면 대기 중이던 프레임을 버리고
    다음 동기부터 다시 받는다 (몇 초 묵은 CH1 값이 '새 값'으로 나가지 않게).
    """

    def __init__(self, channels: int = 8, sync_gap_us: float = 3000.0,
                 min_us: float = PPM_MIN_US, max_us: float = PPM_MAX_US,
                 max_gap_us: float = 50000.0):
        if channels < N_RC:
            raise ValueError(f"channels({channels}) 는 {N_RC} 이상이어야 합니다")
        if not (0 < min_us < max_us < sync_gap_us < max_gap_us):
            raise ValueError("min_us < max_us < sync_gap_us < max_gap_us 여야 합니다")
        self.channels = int(channels)
        self.sync_gap_us = float(sync_gap_us)
        self.min_us = float(min_us)
        self.max_us = float(max_us)
        self.max_gap_us = float(max_gap_us)
        self._prev_ns = None
        self._synced = False
        self._bad = False
        self._iv = []
        # 통계 (rc_ppm_node 가 주기적으로 로그에 찍는다)
        self.frames = 0            # 받은 프레임
        self.rejected = 0          # 동기는 맞았지만 버린 프레임 (글리치·개수 불일치)
        self.last_len = None       # 마지막으로 동기를 본 시점의 간격 개수 (개수 불일치 진단용)

    def reset(self):
        self._prev_ns = None
        self._synced = False
        self._bad = False
        self._iv = []

    def feed(self, t_ns: int):
        """에지 하나. 프레임이 완성되면 [µs 정수] * channels, 아니면 None."""
        prev, self._prev_ns = self._prev_ns, t_ns
        if prev is None:
            return None
        dt = (t_ns - prev) / 1000.0
        if dt > self.max_gap_us:
            # 신호 끊김 뒤 복귀. 대기 중이던 (끊기기 전) 프레임은 내보내지 않는다.
            self._synced, self._bad, self._iv = False, False, []
            return None
        if dt > self.sync_gap_us:
            out = None
            if self._synced:
                self.last_len = len(self._iv)
                if not self._bad and len(self._iv) == self.channels:
                    out = [int(round(x)) for x in self._iv]
                    self.frames += 1
                else:
                    self.rejected += 1
            self._synced, self._bad, self._iv = True, False, []
            return out
        if self.min_us <= dt <= self.max_us:
            if self._synced and not self._bad:
                self._iv.append(dt)
                if len(self._iv) > self.channels:
                    self._bad = True
        else:
            # 범위 밖(빠른 노이즈 에지, max~sync 사이, 역행 타임스탬프) -> 이 프레임은 버린다
            self._bad = True
        return None


# ============================================================
# RC / AUTO 중재
# ============================================================
@dataclass
class RcConfig:
    timeout_s: float = 0.25           # 이만큼 새 프레임이 없으면 링크 없음
    center_us: int = 1500
    span_us: int = 500                # center ± span = 끝까지 민 값
    mode_low_us: int = 1300           # CH1 ≤ → RC
    mode_high_us: int = 1700          # CH1 ≥ → AUTO   (사이 = STOP)
    mode_stable_s: float = 0.3        # CH1 이 이만큼 그대로여야 모드 전환
    neutral_arm_us: int = 90          # CH2 |값-center| ≤ 이면 중립 (arm / auto_ready 조건)
    neutral_hold_s: float = 0.5       # 중립을 이만큼 유지해야 arm / auto_ready
    drive_deadzone_us: int = 100      # CH2 구동 데드존. neutral_arm_us 이상이어야 한다
    steer_deadzone_us: int = 70       # CH3 조향 데드존
    override_us: int = 130            # AUTO 중 CH2 를 이 이상 당기면
    override_hold_s: float = 0.15     #   이만큼 유지 시 AUTO 정지
    ch4_enable: bool = False          # CH4 굴절 조그 (극성·한계 확인 전 false)
    ch4_deadzone_us: int = 150
    ch1_reverse: bool = False
    ch2_reverse: bool = False
    ch3_reverse: bool = False
    ch4_reverse: bool = False
    actuation: bool = False           # false = dry. 판단만 하고 출력은 항상 0
    auto_timeout_s: float = 0.5       # 경로 추종기 rpm·조향 지령 신선도

    def validate(self):
        errs = []
        if not 0 < self.span_us <= 1000:
            errs.append("span_us 는 0 초과 1000 이하")
        if not self.mode_low_us < self.center_us < self.mode_high_us:
            errs.append("mode_low_us < center_us < mode_high_us 여야 함 (중간값 = STOP 구간)")
        if self.neutral_arm_us > self.drive_deadzone_us:
            # 중립으로 arm 했는데 그 자리가 이미 구동 데드존 밖이면 arm 즉시 바퀴가 돈다
            errs.append(f"neutral_arm_us({self.neutral_arm_us}) 는 drive_deadzone_us"
                        f"({self.drive_deadzone_us}) 이하여야 함")
        if self.override_us <= self.neutral_arm_us:
            errs.append("override_us 는 neutral_arm_us 보다 커야 함")
        for n in ('drive_deadzone_us', 'steer_deadzone_us', 'ch4_deadzone_us'):
            if not 0 <= getattr(self, n) < self.span_us:
                errs.append(f"{n} 는 0 이상 span_us 미만")
        for n in ('timeout_s', 'mode_stable_s', 'neutral_hold_s', 'override_hold_s', 'auto_timeout_s'):
            v = getattr(self, n)
            if not (math.isfinite(v) and v >= 0.0):
                errs.append(f"{n} 는 0 이상")
        if self.timeout_s <= 0.0:
            errs.append("timeout_s 는 0 초과")
        if errs:
            raise ValueError("; ".join(errs))
        return self

    @classmethod
    def names(cls):
        return tuple(f.name for f in fields(cls))


@dataclass
class RcOutput:
    mode: str = 'idle'                # 실제 적용: 'manual' | 'auto' | 'idle'
    would_mode: str = 'idle'          # actuation 이 켜져 있었다면
    steer_frac: float = 0.0           # -1 ~ +1 (× max_steer_deg). mode == 'manual' 일 때만 0 아님
    drive: int = 0                    # -1 / 0 / +1
    jog: int = 0                      # -1 / 0 / +1
    would_steer_frac: float = 0.0     # dry 확인용 (actuation 과 무관하게 계산)
    would_drive: int = 0
    would_jog: int = 0
    source: str = 'STOP'              # 'STOP' | 'RC' | 'AUTO'
    switch_request: str = 'STOP'      # CH1 이 지금 가리키는 쪽
    link: bool = False
    armed: bool = False
    auto_ready: bool = False
    latched: bool = False
    channels: tuple = ()              # 부호 보정 후 CH1~CH4 (링크 없으면 빈 튜플)
    reason: str = ''                  # 화면 표시용 짧은 영문 (콘솔 폰트에 한글이 없다)
    events: list = field(default_factory=list)   # 로그로 남길 상태 변화 (한국어)


def _shape(v: float, center: float, dz: float, span: float) -> float:
    """데드존을 뺀 나머지를 -1 ~ +1 로 다시 편다. 데드존 경계에서 값이 튀지 않는다."""
    d = v - center
    if abs(d) <= dz:
        return 0.0
    f = (abs(d) - dz) / (span - dz)
    return math.copysign(min(1.0, f), d)


def _tri(v: float, center: float, dz: float) -> int:
    d = v - center
    return 0 if abs(d) <= dz else (1 if d > 0 else -1)


class RcArbiter:
    """
    CH1~CH4 + 경로 추종기 신선도 -> RcOutput. 시간은 호출자가 넣는다 (monotonic 초).
    상태 변화는 events(한국어 문장)로 돌려주므로 호출자가 로거로 찍는다.
    """

    def __init__(self, cfg: RcConfig = None):
        self.cfg = (cfg or RcConfig()).validate()
        self._ch = ()
        self._ch_t = None
        self.source = 'STOP'
        self.armed = False
        self.auto_ready = False
        self.latched = False
        self._cand = None
        self._cand_t = 0.0
        self._neutral_t = None
        self._override_t = None
        self._events = []

    # ---------- 입력 ----------
    def set_config(self, cfg: RcConfig):
        self.cfg = cfg.validate()

    def update_channels(self, ch, now: float) -> bool:
        """PPM 노드가 보낸 값. 4 개 미만이거나 범위 밖이 있으면 링크 없음으로 친다."""
        ch = list(ch or ())
        ok = len(ch) >= N_RC and all(PPM_MIN_US <= int(x) <= PPM_MAX_US for x in ch[:N_RC])
        if ok:
            self._ch, self._ch_t = tuple(int(x) for x in ch[:N_RC]), now
        else:
            self._ch, self._ch_t = (), None
        return ok

    def request(self, req: str) -> str:
        """GUI 요청. STOP = 래치, RC / RESUME = 래치 해제. 그 밖(AUTO 등)은 거부. 결과 문장을 돌려준다."""
        r = (req or '').strip().upper()
        if r == 'STOP':
            self.latched = True
            self._disarm()
            return 'GUI STOP 래치 — RC/RESUME 요청 전까지 정지'
        if r in ('RC', 'RESUME', 'RESET'):
            was = self.latched
            self.latched = False
            self._disarm()
            return ('STOP 래치 해제' if was else 'RC 재arm 요청') + \
                ' — CH1 선택 후 CH2 중립 유지로 다시 arm'
        return f"'{req}' 거부: AUTO 전환은 CH1 스위치로만 한다"

    # ---------- 판단 ----------
    def _disarm(self):
        self.armed = False
        self.auto_ready = False
        self._neutral_t = None
        self._override_t = None

    def _event(self, msg):
        self._events.append(msg)

    def _channels(self, now):
        c = self.cfg
        if self._ch_t is None or now - self._ch_t > c.timeout_s or len(self._ch) < N_RC:
            return None
        rev = (c.ch1_reverse, c.ch2_reverse, c.ch3_reverse, c.ch4_reverse)
        return tuple(2 * c.center_us - v if r else v for v, r in zip(self._ch, rev))

    def step(self, now: float, *, auto_enable: bool = False,
             auto_rpm_t: float = None, auto_steer_t: float = None) -> RcOutput:
        c = self.cfg
        ch = self._channels(now)
        link = ch is not None
        if link:
            sw, thr, st, po = ch
            cand = 'RC' if sw <= c.mode_low_us else ('AUTO' if sw >= c.mode_high_us else 'STOP')
        else:
            sw = thr = st = po = c.center_us
            cand = 'STOP'

        if cand != self._cand:
            if self._cand is not None:
                self._event(f"CH1 {self._cand} -> {cand}" + ('' if link else ' (RC 신호 없음)'))
            self._cand, self._cand_t = cand, now
            self._disarm()                       # 바운스로 arm 상태가 살아남지 않게
        stable = link and now - self._cand_t >= c.mode_stable_s
        neutral = link and abs(thr - c.center_us) <= c.neutral_arm_us

        def hold_neutral():
            if neutral:
                if self._neutral_t is None:
                    self._neutral_t = now
                return now - self._neutral_t >= c.neutral_hold_s
            self._neutral_t = None
            return False

        if self.latched or cand == 'STOP':
            if self.source != 'STOP':
                self._event('STOP' + (' (GUI 래치)' if self.latched else
                                      '' if link else ' (RC 신호 없음)'))
            self.source = 'STOP'
            self._disarm()
        elif stable and cand == 'RC':
            if self.source != 'RC':
                self.source = 'RC'
                self._disarm()
                self._event('RC 모드 (CH1). Wi-Fi 구동·조향·조그 무시. CH2 중립 유지하면 arm')
            if not self.armed and hold_neutral():
                self.armed = True
                self._event('RC armed (CH2 중립 유지)')
        elif stable and cand == 'AUTO':
            if self.source != 'AUTO':
                self.source = 'AUTO'
                self._disarm()
                self._event('AUTO 모드 (CH1). CH2 중립 유지 + 경로 추종 지령 대기')
            if not self.auto_ready:
                if hold_neutral():
                    self.auto_ready = True
                    self._override_t = None
                    self._event('AUTO 준비 (CH2 중립 유지)')
            elif abs(thr - c.center_us) > c.override_us:
                if self._override_t is None:
                    self._override_t = now
                elif now - self._override_t >= c.override_hold_s:
                    self.auto_ready = False
                    self._neutral_t = None
                    self._override_t = None
                    self._event('AUTO 정지: CH2 조작 (다시 중립 유지하면 재개)')
            else:
                self._override_t = None

        auto_fresh = (auto_enable and c.auto_timeout_s > 0
                      and auto_rpm_t is not None and auto_steer_t is not None
                      and now - auto_rpm_t <= c.auto_timeout_s
                      and now - auto_steer_t <= c.auto_timeout_s)

        out = RcOutput(source=self.source, switch_request=cand, link=link,
                       armed=self.armed, auto_ready=self.auto_ready,
                       latched=self.latched, channels=ch if link else ())
        settled = stable and cand == self.source
        if settled and self.source == 'RC' and self.armed:
            out.would_mode = 'manual'
            out.would_steer_frac = _shape(st, c.center_us, c.steer_deadzone_us, c.span_us)
            out.would_drive = _tri(thr, c.center_us, c.drive_deadzone_us)
            out.would_jog = _tri(po, c.center_us, c.ch4_deadzone_us) if c.ch4_enable else 0
        elif settled and self.source == 'AUTO' and self.auto_ready and auto_fresh:
            out.would_mode = 'auto'

        if c.actuation:
            out.mode = out.would_mode
            out.steer_frac, out.drive, out.jog = (out.would_steer_frac, out.would_drive,
                                                  out.would_jog)
        out.reason = self._reason(out, link, stable, cand, auto_enable, auto_fresh)
        out.events, self._events = self._events, []
        return out

    def _reason(self, o: RcOutput, link, stable, cand, auto_enable, auto_fresh) -> str:
        c = self.cfg
        if self.latched:
            return 'GUI STOP latched - press R (RESUME)'
        if not link:
            return 'NO RC SIGNAL (PPM timeout)'
        if cand == 'STOP':
            return 'CH1 in middle -> STOP'
        if not stable or cand != self.source:
            return 'CH1 settling'
        dry = '' if c.actuation else ' [DRY: output 0]'
        if self.source == 'RC':
            if not self.armed:
                return f'hold CH2 neutral {c.neutral_hold_s:.1f}s to arm' + dry
            return 'RC manual' + dry
        if not self.auto_ready:
            return f'hold CH2 neutral {c.neutral_hold_s:.1f}s for AUTO' + dry
        if not auto_enable:
            return 'auto_enable=false (console P)' + dry
        if not auto_fresh:
            return 'no path follower cmd' + dry
        return 'AUTO driving' + dry


def config_with(cfg: RcConfig, **kw) -> RcConfig:
    """검증된 새 설정 (런타임 파라미터 변경용). 실패하면 ValueError, 원래 설정은 그대로."""
    return replace(cfg, **kw).validate()
