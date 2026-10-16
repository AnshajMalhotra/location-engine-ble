# kalman_rssi.py - 1D Kalman filter per anchor on tag T1's RSSI, compared with the 1-second median.
# Usage: python analysis/kalman_rssi.py [data folder]      (default: data/R_L_01; figure goes to figures/)
import os, sys, json
import numpy as np, pandas as pd, matplotlib.pyplot as plt, matplotlib.dates as mdates
from scipy.optimize import minimize

HERE = os.path.dirname(os.path.abspath(__file__))
D = sys.argv[1] if len(sys.argv) > 1 else os.path.join(HERE, "..", "data", "R_L_01")
ble = pd.read_json(f"{D}/ble_packets_R_L_01.ndjson", lines=True)
ref = pd.read_json(f"{D}/ground_truth_R_L_01.ndjson", lines=True)
anchors = {a["anchor_id"]: np.array([a["x"], a["y"]]) for a in json.load(open(f"{D}/anchor.json"))["anchors"]}
IDS = sorted(anchors)

ble = ble[ble.tag_id == "T1"].assign(t=lambda d: pd.to_datetime(d.ts_gateway))
grid = pd.date_range(ble.t.min(), ble.t.max(), freq="1s")          # 1,442 one-second epochs
day = "2026-04-14 "
CAL = (pd.Timestamp(day + "16:29:43Z"), pd.Timestamp(day + "16:30:40Z"))    # tag still at start (SLAM moves < 5 cm/s)
CHECK = (pd.Timestamp(day + "16:51:30Z"), pd.Timestamp(day + "16:53:44Z"))  # tag still at end

# R = measurement noise: how much RSSI jumps while the tag does NOT move (calibration window only)
in_cal = ble.t.between(*CAL)
R = ble[in_cal].groupby("anchor_id").rssi.var().to_dict()

def kalman(packets, Q, R):
    """Random-walk model: true RSSI x_k = x_(k-1) + w (var Q per second), packet z = x + v (var R)."""
    by_sec = packets.groupby("t").rssi.apply(list)
    x, P, out, nis = None, None, [], []
    for t in grid:
        if x is not None:
            P += Q                                   # predict: x stays, uncertainty grows
        for z in by_sec.get(t, []):                  # update with every packet received in this second
            if x is None:
                x, P = float(z), R                   # start at the first packet
                continue
            S = P + R
            nis.append((z - x) ** 2 / S)             # normalised innovation: ~1 on average if Q and R fit
            K = P / S
            x, P = x + K * (z - x), (1 - K) * P
        out.append(np.nan if x is None else x)
    return pd.Series(out, index=grid), np.mean(nis)

# Q = how fast the true RSSI changes, from physics, NOT from the SLAM error (that would be tuning on the test):
# walking ~0.5 m/s at ~4 m from an anchor, path-loss slope 10n/(ln10*d) ~ 2.7 dB/m -> ~1.4 dB/s -> Q ~ 2 dB^2/s
Q = 2.0
# Consistency check: mean NIS should be ~1 if the Gaussian white-noise model fits. Far above 1 for every Q = it doesn't.
Qs = [0.3, 2, 30]
mean_nis = {q: np.mean([kalman(ble[ble.anchor_id == a], q, R[a])[1] for a in IDS]) for q in Qs}

med = ble.groupby(["t", "anchor_id"]).rssi.median().unstack().reindex(grid)
kf = pd.DataFrame({a: kalman(ble[ble.anchor_id == a], Q, R[a])[0] for a in IDS})

# Position: same range fit as the shared RawSolver (A=-60 dBm, n=2.5, Nelder-Mead from [10, 5]); height ignored
A, n = -60.0, 2.5
def fit(rssi_row):
    d = 10 ** ((A - rssi_row.values) / (10 * n))
    P = np.array([anchors[a] for a in rssi_row.index])
    return minimize(lambda p: np.sum((np.linalg.norm(P - p, axis=1) - d) ** 2), [10.0, 5.0],
                    method="Nelder-Mead", options={"maxiter": 200}).x

# Reference: collapse repeated timestamps, then nearest pose within +-500 ms of each epoch (evaluation only)
ref = ref.assign(t=pd.to_datetime(ref.ts)).groupby("t", as_index=False)[["x", "y"]].median()
pairs = pd.merge_asof(pd.DataFrame({"t": grid}), ref, on="t", direction="nearest", tolerance=pd.Timedelta("500ms"))
ok = (med.notna().all(axis=1).values) & pairs.x.notna().values        # complete median epochs with a reference
truth = pairs.loc[ok, ["x", "y"]].values
err = {name: np.linalg.norm(np.array([fit(r) for _, r in df[ok].iterrows()]) - truth, axis=1)
       for name, df in [("1-s median", med), ("Kalman", kf)]}

print("R per anchor (dB^2, static start):", {a: round(R[a], 2) for a in IDS})
print("mean NIS per Q:", {q: round(v, 2) for q, v in mean_nis.items()}, "-> Q =", Q, "dB^2/s")
still = grid.to_series().between(*CHECK).values
for name, df in [("1-s median", med), ("Kalman", kf)]:
    print(f"{name:10s} RSSI std while still at end (dB):", df[still].std().round(2).to_dict())
for name, e in err.items():
    print(f"{name:10s} n={len(e)} RMSE={np.sqrt(np.mean(e**2)):.2f} m  median={np.median(e):.2f}  P90={np.percentile(e, 90):.2f}")

# Figure: (a) what the filter does to one anchor's RSSI, (b) what that does to position error
BLUE, ORANGE, GRAY = "#2a78d6", "#eb6834", "#9a9a94"
plt.rcParams.update({"font.size": 15})  # readable when shown on a slide
fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(13, 4.6))
a = "A2"
pk = ble[ble.anchor_id == a]
ax1.scatter(pk.t, pk.rssi, s=4, color=GRAY, alpha=0.4, label="raw packets")
ax1.plot(grid, med[a], color=BLUE, lw=1, label="1-s median")
ax1.plot(grid, kf[a], color=ORANGE, lw=2, label=f"Kalman (Q={Q}, R={R[a]:.1f})")
for w in (CAL, CHECK):
    ax1.axvspan(*w, color=GRAY, alpha=0.12, lw=0)
ax1.set(title=f"RSSI at anchor {a}, UTC (shaded: tag still)", ylabel="RSSI (dBm)")
ax1.legend(loc="upper center", bbox_to_anchor=(0.5, -0.1), ncol=3, frameon=False, handlelength=1.2, columnspacing=1)
ax1.xaxis.set_major_formatter(mdates.DateFormatter("%H:%M"))
for (name, e), c in zip(err.items(), (BLUE, ORANGE)):
    ax2.plot(np.sort(e), np.linspace(0, 1, len(e)), color=c, lw=2, label=f"{name}  (median {np.median(e):.2f} m)")
ax2.set(title=f"Position error vs SLAM ({ok.sum()} epochs)", xlabel="error (m)", ylabel="fraction of epochs")
ax2.legend(loc="lower right", frameon=False)
for ax in (ax1, ax2):
    ax.grid(alpha=0.25); ax.spines[["top", "right"]].set_visible(False)
fig.tight_layout()
out = os.path.join(HERE, "..", "figures", "kalman_rssi.png")
fig.savefig(out, dpi=200)
print("saved", os.path.normpath(out))
