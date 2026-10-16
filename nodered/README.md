# Node-RED flows

Import into Node-RED with *Menu → Import*. Broker and host addresses have been replaced with `<host>`, so set your own before deploying.

## `flows_13_rtls_pipeline.json` — tab "RTLS Data Pipeline" (MKGW3 path)

| Node | What it does |
|---|---|
| MKGW3 IN | MQTT subscription `/MKGW3/+/send`. The `+` wildcard takes every MKGW3 gateway on one subscription. |
| Unification | Parses a gateway JSON payload or a binary frame (header `0xEF`) into `gateway_mac`, `tag_mac`, `rssi`, `ts` (arrival time) |
| Whitelist Tags | Keeps only the known tag MACs (27 in this version) |
| Anchor validation | Maps a gateway MAC to an anchor ID and drops unknown gateways |
| Aggregation | Groups packets per tag into 1-second buckets and emits a bucket once it is complete |
| Final Data Set Formatter | Writes one NDJSON row per packet: `ts_gateway`, `run_id`, `anchor_id`, `tag_id`, `rssi`, `seq` |

The same tab also holds the MKGW4 and MK107 Pro inputs and the calibration exports. They are not covered here.

## `flows_14_slam_export.json` — tab "SLAM Clean Export"

| Node | What it does |
|---|---|
| SLAM Trigger | Inject node, fires every 0.5 s |
| Get SLAM Position | HTTP GET to the DeepHub location API |
| SLAM → NDJSON Formatter | Writes `ts` (DeepHub timestamp), `x`, `y`, `yaw`, `tag_id: T1`, `source: lidar_slam` |
| Save Ground Truth | Appends to `ground_truth.ndjson` |

## `flows_11_mkgw4_sensor.json` — tab "MKGW4 (sensor)"

MKGW4 is a cellular gateway with GNSS. It reports a binary frame instead of JSON.

| Node | What it does |
|---|---|
| MKGW4-report | MQTT subscription for the gateway reports |
| ByteParser: Commands | Checks header `0xEF`, reads the 2-byte command, 6-byte gateway MAC and 2-byte length, and cuts out the data block |
| Select by Command | Routes by command. `0x3089` = GNSS fix, `0x30A0` = beacon records. The others are not used yet |
| GNSS | Walks the TLV records (tag, 2-byte length, value). Longitude and latitude are int32 × 1e-7; LBS gives area code and cell ID |
| GPS fix → prepare gateway omlox | Only a GPS fix updates the gateway position (WGS84) on the location server |
| Beacon → split → decode beacon | Walks the beacon TLV records (type, MAC, RSSI, raw advert), keeps whitelisted beacons, then decodes the sensor values |
| Sensors Update / Location Update | Sends the decoded values and position to the location server over HTTP |

## `flows_10_l02s_aoa.json` — tab "L02S_AoA"

| Node | What it does |
|---|---|
| subscribe / unsubscribe → mqtt | Dynamic MQTT subscription to the gateway topic and the AoA position topic |
| switch → split → switch | Separates the two topics and picks the right records |
| temperature / X,Y,Z | Takes the temperature from the BXP sensor and x, y, z from the AoA engine |
| join | Merges both into one message |
| Test Case Validation | Temperature must be 0–6 °C and the tag must be inside a fridge box (x 1–3, y 2–4, z 0–2 m). Result: OK, TEMP_ALERT, GEOFENCE_ALERT or CRITICAL |

## Screenshots

`figures/nodered_flow13_rtls_pipeline.png`, `nodered_flow14_slam_export.png`, `nodered_flow11_mkgw4_byteparser.png` and `nodered_flow10_l02s_aoa.png` are the original tabs in the Node-RED editor, as laid out at DynaWo. For readability, a few far-away helper nodes (an unused inject, the calibration file outputs and two debug nodes) were left out of the screenshots. They are still in the JSON files. `nodered_unification_code.png` shows the Unification node's code.

## Things worth knowing

- **Two clocks:** BLE rows carry Node-RED arrival time, SLAM rows carry DeepHub time. Alignment is a real step, not a formality.
- **`adv_hz = 10` is a constant,** not a measured advertising rate.
- **Packets stay raw rows.** The per-second median is computed later, in Python.
