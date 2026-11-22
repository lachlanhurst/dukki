"""Per second of a stand_test.py recording: policy label, tilt (mean / max), gravity, the joint
that moved most, peak current and fallen frames.

    python3 scripts/policy/timeline.py RECORDING.json
"""
import json, math, statistics as st, sys
NAMES = ["l_hyaw","l_hroll","l_hpitch","l_knee","l_ankle","neck","hpitch","hyaw","hroll","mouth","r_hyaw","r_hroll","r_hpitch","r_knee","r_ankle"]
fr = json.load(open(sys.argv[1] if len(sys.argv) > 1 else "/tmp/stand_test.json"))
for sec in range(0, int(fr[-1][0]) + 1):
    w = [p for t, p in fr if sec <= t < sec + 1]
    if not w: continue
    tilt = [math.degrees(math.acos(max(-1, min(1, -p["safety"]["gravity"][2])))) for p in w]
    g = w[-1]["safety"]["gravity"]
    rng = lambda j: max(math.degrees(p["joints"][j]) for p in w) - min(math.degrees(p["joints"][j]) for p in w)
    which = max(range(15), key=rng)
    cur = max(max(p["currents_ma"]) for p in w)
    fallen = sum(p["safety"]["fallen"] for p in w)
    print(f"{sec:2d}s {w[-1].get('policy')} tilt {st.mean(tilt):5.1f}/{max(tilt):5.1f} g[{g[0]:+.2f} {g[1]:+.2f} {g[2]:+.2f}] most moving {NAMES[which]:8} {rng(which):6.1f} deg  peak {cur:5.0f} mA  fallen {fallen}")
