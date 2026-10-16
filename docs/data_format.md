# Data format

## `ble_packets_R_L_01.ndjson` — one row per received packet

Written by the *Final Data Set Formatter* node (`nodered/flows_13_rtls_pipeline.json`).

| Field | Meaning |
|---|---|
| `ts_gateway` | Time the message **reached Node-RED** (`Date.now()` in *Unification*), floored to the second by *Aggregation*. Not the gateway's own clock. |
| `run_id` | Session name |
| `gateway_id`, `anchor_id` | Anchor the gateway MAC maps to (*Anchor validation*) |
| `tag_id` | Tag name from the session's tag table (`tags.json`) |
| `rssi` | Received signal strength, dBm |
| `adv_hz` | Constant 10, written by the formatter. **Not a measured rate.** |
| `seq` | Running counter in the formatter, not an over-the-air sequence number |

Several rows share one `ts_gateway`. The analysis takes the per-second median per anchor.

## `ground_truth_R_L_01.ndjson` — reference poses

Written by *SLAM → NDJSON Formatter* (`nodered/flows_14_slam_export.json`). The flow polls the DeepHub location API every 0.5 s.

| Field | Meaning |
|---|---|
| `ts` | DeepHub `timestamp_generated` — a different clock from the BLE timestamps |
| `x`, `y`, `yaw` | Pose in the local map frame (m, rad); `z` is written as 0 |
| `tag_id` | Written as `T1` by the formatter: only T1 carries a reference |
| `source` | Written as `lidar_slam` by the formatter, not reported by the device |

## `anchor.json`, `tags.json`, `route_plan.json`

Anchor positions (m) and gateway MACs, tag MACs, and the planned route for this session. The Node-RED export in `nodered/` is a later version (30 April), with updated tag and anchor tables. For R_L_01, always use this session's own files.
