# Shared simulation framework (DynaWo team code)

This is the software-in-the-loop framework from the project, written by the DynaWo team. It is kept here unchanged, so the scripts in `analysis/` run against the original solvers.

| File | Content |
|---|---|
| `config.py` | RF defaults (−60 dBm at 1 m, n = 2.5, σ = 3.5 dB, 8 dB per wall) and map settings |
| `environment.py` | Simplified office: 9 rooms, 14 walls, 10 anchors |
| `simulator.py` | Trajectory generator and RSSI sensor model with wall losses |
| `solvers.py` | `RawSolver` (range fit, Nelder–Mead) and `ParticleFilterSolver` (with or without the wall model) |
| `evaluator.py` | RMSE and plot |
| `test.ipynb` | The original run (seed 1024, 19 steps) |

My review of its limitations is in `docs/results.md`.
