# replay_pf.py - replay the shared solvers on the recorded walk R_L_01 (tag T1) and compare them with SLAM.
# Usage: python analysis/replay_pf.py [data folder] [seed ...]      (defaults: data/R_L_01, seed 1024)
import os, sys, json
import numpy as np, pandas as pd

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "shared_sil"))          # the team's solver code, unchanged
from config import RFConfig
from solvers import RawSolver, ParticleFilterSolver

D = sys.argv[1] if len(sys.argv) > 1 else os.path.join(HERE, "..", "data", "R_L_01")
SEEDS = [int(s) for s in sys.argv[2:]] or [1024]

# ---- load: T1 packets, anchor registry, SLAM reference -------------------------------------------
ble = pd.read_json(f"{D}/ble_packets_R_L_01.ndjson", lines=True)
ref = pd.read_json(f"{D}/ground_truth_R_L_01.ndjson", lines=True)
reg = {a["anchor_id"]: (a["x"], a["y"]) for a in json.load(open(f"{D}/anchor.json"))["anchors"]}
IDS = sorted(reg)
A = np.array([reg[a] for a in IDS])                                   # 2D only: antenna height ignored

t1 = ble[ble.tag_id == "T1"].assign(t=lambda d: pd.to_datetime(d.ts_gateway))
grid = pd.date_range(t1.t.min(), t1.t.max(), freq="1s")
med = t1.groupby(["t", "anchor_id"]).rssi.median().unstack().reindex(index=grid, columns=IDS)  # NaN = not heard

ref = ref.assign(t=pd.to_datetime(ref.ts)).groupby("t", as_index=False)[["x", "y"]].median()   # collapse repeats
ref["t_ref"] = ref.t
pairs = pd.merge_asof(pd.DataFrame({"t": grid}), ref, on="t", direction="nearest", tolerance=pd.Timedelta("500ms"))
matched = pairs.x.notna().values
complete = med.notna().all(axis=1).values
paired = matched & complete                                           # the comparison set
truth = pairs.loc[paired, ["x", "y"]].values

offset_ms = (pairs.t_ref - pairs.t).abs().dt.total_seconds()[paired] * 1000
print(f"packets {len(ble):,} (T1 {len(t1):,}) | epochs {len(grid):,} | anchors heard per epoch:",
      med.notna().sum(axis=1).value_counts().sort_index(ascending=False).to_dict())
print(f"matched to SLAM {matched.sum():,} | paired complete {paired.sum():,} ({paired.mean():.1%}) | "
      f"match offset median {offset_ms.median():.0f} ms, max {offset_ms.max():.0f} ms")

def report(name, est):
    e = np.linalg.norm(est - truth, axis=1)
    print(f"{name:22s} RMSE {np.sqrt(np.mean(e**2)):.2f} m | median {np.median(e):.2f} | "
          f"P90 {np.percentile(e, 90):.2f} | P99 {np.percentile(e, 99):.2f} | max {e.max():.2f}")

rf = RFConfig()                                                       # shared defaults: -60 dBm @ 1 m, n 2.5, sigma 3.5 dB

# ---- 1) shared RawSolver, unchanged, on complete epochs --------------------------------------------
raw = RawSolver(A, rf, top_k=3)
report("range fit (RawSolver)", np.array([raw.step(r) for r in med.values[paired]]))

# ---- 2) shared particle filter, adapted to the real recording --------------------------------------
class ReplayPF(ParticleFilterSolver):
    """Same random walk, likelihood, floor, weighted mean and resampling as the shared filter.
    Changes: a rectangle around the anchors (+5 m) replaces the synthetic office, no walls
    (no surveyed wall map for this session), and each update uses only the anchors heard."""
    def __init__(self, anchors, rf, n=1000):
        lo, hi = anchors.min(axis=0) - 5, anchors.max(axis=0) + 5
        super().__init__(n, [[lo[0], lo[1], hi[0], hi[1]]], [], anchors, rf, use_waf=False)
        self.all_anchors = anchors
        self.particles = np.random.uniform(lo, hi, size=(n, 2))

    def step(self, rssi):
        heard = np.isfinite(rssi)
        self._predict()
        if heard.any():
            self.anchors, self.top_k = self.all_anchors[heard], int(heard.sum())
            self._update(rssi[heard])
        else:                                                         # nothing heard: keep the prediction
            self.weights[:] = 1.0 / self.num_particles
        est = self._estimate()
        self._resample()
        return est

for seed in SEEDS:
    np.random.seed(seed)
    pf = ReplayPF(A, rf)
    est = np.array([pf.step(r) for r in med.values])                  # runs every second, partial epochs too
    report(f"particle filter s{seed}", est[paired])
