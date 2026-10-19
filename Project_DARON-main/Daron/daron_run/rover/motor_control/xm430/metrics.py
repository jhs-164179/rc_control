"""Pure suspension recovery metrics, shared by live control and analysis."""

NAN = float("nan")

MIN_SPAN_DEG = 1.0

def recovery_metrics(t, err, vel, cur, settled_deg: float) -> dict:
    """
    t[s], err[deg], vel[deg/s], cur[mA] 의 같은 길이 리스트를 받는다.
    err 은 (goal - position) 이며 첫 표본이 초기 변위다.

    지표는 '도달한 복원량' 으로 정규화한다. 하중을 받는 서스펜션은 낮은
    유지 토크 때문에 정적 처짐(잔류 오차)이 남는 것이 정상이고, 목표선을
    기준으로 t90 을 재면 처짐이 조금만 있어도 전부 NaN 이 되어 방향 간
    비교 자체가 불가능해진다. 그래서 진행률

        x(t) = (err0 - err(t)) / (err0 - err_final)     0 -> 1

    로 옮겨 재고, 처짐 자체는 residual_deg 로 따로 보고한다.
    """
    n = len(t)
    out = dict(n=n, err0_deg=NAN, t90_s=NAN, t_settle_s=NAN, overshoot_pct=NAN,
               osc=0, peak_vel_dps=NAN, mean_vel_dps=NAN, peak_mA=NAN,
               residual_deg=NAN, span_deg=NAN, duration_s=NAN)
    if n < 3:
        return out
    t0 = t[0]
    e0 = err[0]
    k = max(1, min(5, n // 10))
    ef = sorted(err[-k:])[k // 2]          # 끝부분 중앙값 = 최종 평형 오차
    span = e0 - ef
    out.update(err0_deg=e0, residual_deg=ef, span_deg=span,
               duration_s=t[-1] - t0,
               peak_vel_dps=max(abs(v) for v in vel),
               peak_mA=max(abs(c) for c in cur))

    path = sum(abs(err[i] - err[i - 1]) for i in range(1, n))
    dur = t[-1] - t0
    out["mean_vel_dps"] = path / dur if dur > 0 else NAN
    if abs(span) < 1e-6:
        return out

    x = [(e0 - e) / span for e in err]     # 진행률 0 -> 1
    for i in range(n):
        if x[i] >= 0.9:
            out["t90_s"] = t[i] - t0
            break
    out["overshoot_pct"] = 100.0 * max(0.0, max(x) - 1.0)

    # 정착 시간: 최종 평형 주위 settled 밴드를 마지막으로 벗어난 직후
    band = max(settled_deg / abs(span), 0.02)
    idx = None
    for i in range(n - 1, -1, -1):
        if abs(x[i] - 1.0) > band:
            idx = i
            break
    if idx is None:
        out["t_settle_s"] = 0.0
    elif idx + 1 < n:
        out["t_settle_s"] = t[idx + 1] - t0

    prev = 0
    for xi in x:
        s = 1 if xi < 1.0 else (-1 if xi > 1.0 else 0)
        if s and prev and s != prev:
            out["osc"] += 1
        if s:
            prev = s
    return out

def _median(xs):
    xs = sorted(x for x in xs if x == x)
    if not xs:
        return NAN
    k = len(xs)
    return xs[k // 2] if k % 2 else 0.5 * (xs[k // 2 - 1] + xs[k // 2])
