# RSSI Location Engine: BLE indoor localisation

This is my working folder from the RSSI Location Engine project at DynaWo (internship, March to May 2026). I've tidied it up so it can be shown. It follows one tag from the radio packet, through MQTT and Node-RED, to a position estimate that I check against a SLAM reference.

```
data/R_L_01/     recorded walk, 14 April 2026: BLE packets, SLAM poses, anchor and tag registry
nodered/         Node-RED flows: BLE pipeline (13), SLAM export (14), MKGW4 byte parser (11), L02S AoA test (10)
shared_sil/      the team's simulation and solver code (unchanged)
analysis/        my scripts that rerun and evaluate everything on the recording
figures/         plots produced by the scripts
docs/            system architecture (HLD), data format, and where every number comes from
```

## Setup

- **Tag:** MOKO M2 BLE asset tag
- **Gateways:** 3 × MOKO MKGW3, fixed at surveyed positions, 2.2 m high (A1–A3)
- **Transport:** MQTT broker, topic `/MKGW3/+/send`
- **Processing:** Node-RED on a local server, writing NDJSON logs
- **Reference:** MRDVS V2 Pro SLAM module (LiDAR + vision + IMU) on the same test cart, read through the DeepHub location API every 0.5 s

## Who did what

| Folder | Origin |
|---|---|
| `data/` | Recorded during my internship (gateway setup, anchor survey, walk with the SLAM reference). |
| `nodered/` | The Node-RED flows I worked on during the internship: MQTT ingest, unification, whitelist, anchor mapping, aggregation, the SLAM export, the MKGW4 binary-frame parser and the L02S AoA geofence test. Host addresses and tokens have been removed. |
| `shared_sil/` | The simulation framework, range-fit solver and particle filter, written by the DynaWo team. I keep it unchanged so my scripts run against the original code. |
| `analysis/` | My scripts (October 2026, written with AI assistance) that replay the shared solvers on the real recording, add a Kalman filter, and check the simulation's seed sensitivity. I can explain every step. |

## Run it

```bash
pip install -r requirements.txt
python analysis/replay_pf.py        # coverage, range fit vs particle filter against SLAM (about 20 s)
python analysis/kalman_rssi.py      # Kalman filter vs 1-s median; writes figures/kalman_rssi.png
python analysis/seed_check.py       # shared simulation: seed 1024 and seeds 0-29
```

Expected output: see `docs/results.md`.

## What I learned

- **The input data limits accuracy more than the algorithm does.** Within one second the packets agree to about 1 dB. But even with the tag standing still, the RSSI at one anchor drifted by several dB. Smoothing cuts the tail of the error, not its median.
- **The reference is an input with its own assumptions.** BLE time is Node-RED arrival time, cut to 1 s. SLAM time is DeepHub's own clock. The frame, the clocks and the tag mounting all need checking before an error number means "accuracy".
- **The simulation's wall model works because the model matches.** The simulator and the solver share the same walls and wall loss. On real data, with default RF settings, the error stays at metre scale.

## Known limitations of the shared solver

These are listed in `docs/results.md`: the reset branch can never fire, the weighted mean can land inside a wall, a missing value (NaN) is ranked as the strongest anchor, the range fit never checks convergence, and the filter resamples on every step.
