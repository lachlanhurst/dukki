"""The walk policy's inputs on the robot against in simulation, per input: mean, spread within an
episode and mean change per tick, standing (robot 3 to 6.5 s) and walking at 0.15 m/s (7.5 to 9.5 s).

    uv run --with numpy python scripts/policy/obs_compare.py ROBOT.obs.npz SIM_NOISY.npz SIM_CLEAN.npz

ROBOT.obs.npz is obs_replay.py's output for a walk_test.py recording; the two sim files are
sim/obs_dump.py's, with and without the training's sensor noise.
"""
import sys

import numpy as np

R = np.load(sys.argv[1])
names = ["l_hyaw", "l_hroll", "l_hpitch", "l_knee", "l_ankle", "neck", "hpitch", "hyaw", "hroll",
         "r_hyaw", "r_hroll", "r_hpitch", "r_knee", "r_ankle"]
labels = [f"gyro_{a}" for a in "xyz"] + [f"grav_{a}" for a in "xyz"] + [f"q_{n}" for n in names] + \
         [f"qd_{n}" for n in names] + [f"act_{n}" for n in names]


def robot(a, b):
    m = R["ok"] & (R["t"] >= a) & (R["t"] < b)
    return R["obs"][m][:, :48], R["act"][m]


def sim(f, envs):
    S = np.load(f)
    return S["obs"][100:, envs, :48], S["act"][100:, envs]  # after 2 s settle


def summary(o):
    if o.ndim == 2:
        o = o[:, None]
    mean = o.reshape(-1, o.shape[-1]).mean(0)
    std = o.std(0).mean(0)  # spread within an episode
    jit = np.abs(np.diff(o, axis=0)).mean((0, 1))  # mean |change per tick|
    return mean, std, jit


cases = [("STAND", robot(3.0, 6.5), sim(sys.argv[2], slice(0, 8)), sim(sys.argv[3], slice(0, 8))),
         ("WALK 0.15", robot(7.5, 9.5), sim(sys.argv[2], slice(8, 16)), sim(sys.argv[3], slice(8, 16)))]
for name, (ro, ra), (so, sa), (co, ca) in cases:
    rm, rs, rj = summary(ro)
    sm, ss, sj = summary(so)
    cm, cs, cj = summary(co)
    print(f"\n=== {name}: robot {len(ro)} ticks; sim 8 envs x {so.shape[0]} ticks (S noisy obs / C clean obs)")
    print(f"{'input':<14}{'mean R':>8}{'S':>8}{'C':>8} |{'std R':>7}{'S':>7}{'C':>7} |{'|d|/tick R':>11}{'S':>7}{'C':>7}  R/S")
    for k, l in enumerate(labels):
        print(f"{l:<14}{rm[k]:8.3f}{sm[k]:8.3f}{cm[k]:8.3f} |{rs[k]:7.3f}{ss[k]:7.3f}{cs[k]:7.3f} |"
              f"{rj[k]:11.3f}{sj[k]:7.3f}{cj[k]:7.3f}  {rj[k] / max(sj[k], 1e-6):5.1f}")
    d = lambda a: np.abs(np.diff(a, axis=0)).mean()
    print(f"policy output |d action|/tick: robot {d(ra):.3f}  sim noisy {d(sa):.3f}  sim clean {d(ca):.3f}")
