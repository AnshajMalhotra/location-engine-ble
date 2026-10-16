# seed_check.py - rerun the shared simulation (test.ipynb setup) for seed 1024 and for seeds 0-29.
# Usage: python analysis/seed_check.py [path to shared_sil]
import os, sys, numpy as np
sys.path.insert(0, sys.argv[1] if len(sys.argv) > 1 else os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "shared_sil"))
from config import SpatialConfig, RFConfig, KinematicsConfig
from environment import MapBuilder
from simulator import TrajectoryGenerator, SensorModel
from solvers import RawSolver, ParticleFilterSolver
from evaluator import PerformanceEvaluator

def run(seed):  # same call order as test.ipynb main(), so seed 1024 must reproduce it
    np.random.seed(seed)
    rf = RFConfig(); rooms, walls, anchors = MapBuilder(SpatialConfig()).build()
    gt = TrajectoryGenerator(KinematicsConfig()).generate([[2.5,8.0],[0.5,8.0],[0.5,5.0],[10.5,5.0],[10.5,2.0],[12.5,2.0]])
    sensor = SensorModel(rf, walls, anchors)
    solvers = [RawSolver(anchors, rf), ParticleFilterSolver(1000, rooms, walls, anchors, rf, use_waf=False),
               ParticleFilterSolver(1000, rooms, walls, anchors, rf, use_waf=True)]
    out = [[], [], []]
    for p in gt:
        m = sensor.get_measurements(p)
        for o, s in zip(out, solvers): o.append(s.step(m))
    ev = PerformanceEvaluator(rooms, walls, anchors)
    return len(gt), [ev.calculate_rmse(gt, np.array(o)) for o in out]

n, r = run(1024)
print(f"seed 1024: steps={n} raw={r[0]:.7f} naive={r[1]:.7f} waf={r[2]:.7f}")
assert n == 19 and abs(r[2] - 0.4884302) < 1e-6, "did not reproduce documented numbers"
R = np.array([run(s)[1] for s in range(30)])
for name, col in zip(["raw", "naivePF", "wafPF"], R.T):
    print(f"30 seeds {name}: median={np.median(col):.2f} min={col.min():.2f} max={col.max():.2f}")
print("WAF better than naive in", int((R[:,2] < R[:,1]).sum()), "of 30 seeds")
