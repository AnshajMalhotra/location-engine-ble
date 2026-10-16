# Where every number comes from

All numbers below are printed by the scripts in `analysis/`, on the recording in `data/R_L_01/`.

## Coverage of the recording (`replay_pf.py`)

| Quantity | Value |
|---|---|
| BLE packets (all tags) / tag T1 | 31,435 / 9,626 |
| One-second epochs | 1,442 (16:29:43–16:53:44 UTC) |
| Epochs hearing 3 / 2 / 1 / 0 anchors | 1,052 / 354 / 35 / 1 |
| Epochs matched to a SLAM pose within ±0.5 s | 1,382 |
| Comparison set: all 3 anchors and a SLAM match | 1,014 (70.3%) |
| Match offset: median / max | 121 ms / 486 ms |

## Real walk vs SLAM, same 1,014 epochs (`replay_pf.py`)

| | RMSE | Median | P90 | P99 | Max |
|---|---|---|---|---|---|
| Range fit (shared RawSolver, unchanged) | 5.91 m | 4.19 m | 9.37 m | 14.57 m | 18.81 m |
| Particle filter, seed 1024 (shared filter, adapted) | 3.35 m | 2.94 m | 4.90 m | 7.12 m | 9.58 m |

Seeds 1024–1033 give a particle-filter RMSE of 3.34–3.36 m. That is the filter's randomness on one trace, not session-to-session spread.

Adaptation of the particle filter: a rectangle around the anchors (+5 m) replaces the synthetic office, there are no walls (no surveyed wall map exists for this session), and each update uses only the anchors heard in that second. RF settings are the shared defaults (−60 dBm at 1 m, n = 2.5, σ = 3.5 dB). Nothing was fitted to this trace.

## Kalman filter vs 1-s median, same 1,014 epochs (`kalman_rssi.py`)

| | RMSE | Median | P90 |
|---|---|---|---|
| 1-s median | 5.91 m | 4.19 m | 9.37 m |
| Kalman (Q = 2 dB²/s) | 5.60 m | 4.12 m | 8.69 m |

- R: RSSI variance while the tag stood still (16:29:43–16:30:40) — A1 0.56, A2 0.81, A3 0.35 dB².
- Q: from physics, not from the SLAM error: 0.5 m/s × about 2.7 dB/m at 4 m ≈ 1.4 dB/s.
- Mean NIS is 15–24 for any Q (ideal ≈ 1), so the noise is heavy-tailed and drifting, not white.
- Standing still at the end (16:51:30–16:53:44), the variance at A1 was 13.8 dB².

## Shared simulation (`seed_check.py`)

| | Seed 1024 | 30 seeds: median (min–max) |
|---|---|---|
| Raw range fit | 5.55 m | 5.32 m (4.37–6.38) |
| Particle filter, no walls | 3.88 m | 2.57 m (1.86–6.04) |
| Particle filter + walls | 0.49 m | 0.75 m (0.47–1.15) |

The wall-aware filter beats the other in 30 of 30 seeds. Simulator and solver share the same walls and 8 dB loss, so this tests the mechanism, not real accuracy.

## Limitations found in the shared solver (`shared_sil/solvers.py`)

1. The reset branch never fires: 1,000 × 1e-12 weight floor > 1e-15 threshold.
2. The position is the weighted mean of all particles, so it can land inside a wall.
3. `np.argsort` ranks a missing (NaN) RSSI as strongest, which crashes with "probabilities contain NaN".
4. The range fit restarts at [10, 5] every step and ignores `result.success`.
5. Resampling on every step reduces particle diversity; resampling only when N_eff < N/2 is the usual fix.
