
# LoRa RFM9x Simulator Server - Technical Report (Updated)

## Overview

This simulator virtualizes a LoRa RFM9x communication environment using TCP sockets. It allows multiple simulated nodes (virtual radios) to communicate via a centralized server, mimicking real-world RF behavior by incorporating physical, environmental, and probabilistic signal models.

### Key Features

- Drop-in simulation for `adafruit_rfm9x` API using `--simulate` switch
- Supports point-to-point and broadcast communication
- Simulates RSSI, SNR, delays, packet collisions, and probabilistic packet loss
- Adjustable environmental factors: AQI, weather, obstacle material
- ACK/retry support for reliable datagram-style transmission
- Node location modeling to simulate realistic distance-based RF propagation

---

## Simulation Architecture

### Components

1. **`simulated_server.py`** – Acts as the central RF propagation environment, managing all node connections, routing packets, simulating RSSI/SNR, and introducing packet loss or delay based on real-world-inspired models.

2. **`simulated_rfm9x.py`** – Drop-in replacement for Adafruit's `RFM9x` class. Sends/receives JSON packets over TCP to/from the simulation server. Supports:
   - `send()`, `receive()`
   - `send_with_ack()`, `last_rssi`, `last_snr`

3. **`rfm9x_simpletest.py`** – Example demonstrating the simulation layer integration. Use:
   ```bash
   python3.10 rfm9x_simpletest.py --simulate --id=1 --location=0,0
   ```

---

## Signal Propagation Model

### RSSI Formula

```
RSSI = TxPower_dBm - (FSPL + Loss_rain + Loss_air + Loss_obstacle + random_noise)
```

Where:

- `TxPower_dBm`: typically +14 dBm
- `FSPL`: Free-space path loss
- `Loss_*`: Environmental losses
- `random_noise`: ±2 dB random variance

### SNR Estimate

```
SNR = RSSI - NoiseFloor_dBm
```

- `NoiseFloor_dBm`: −120 dBm (approximate for RFM9x)

---

## Environmental Loss Models

| Type             | Equation / Value                          | Notes |
|------------------|-------------------------------------------|-------|
| FSPL             | `32.45 + 20log10(d_km) + 20log10(f_MHz)`   | f = 868/915 MHz |
| Rain             | `α_rain * d_km`                            | `α_rain` from rain table |
| AQI              | `((AQI - 50) / 100) * 0.02 * d_km`         | Used only if AQI > 50 |
| Obstacle         | Lookup from `OBSTACLE_LOSS_DB` dictionary | Various materials |
| Random noise     | ±2 dB                                      | Added to every RSSI |

### Weather Delay (Added Latency)

```
Delay_ms = 5 * d_km + WeatherFactor + AQI Delay + Obstacle Delay + Jitter
```

Where:
- Weather adds extra delay (see `RAIN_DELAY_FACTORS`)
- AQI adds delay if AQI > 100
- Obstacle adds 5–25 ms
- Jitter adds 1–5 ms

---

## Packet Loss Model

A probabilistic model based on **SNR history and congestion**:

- If `avg_snr < 0`: high drop probability
- `0 ≤ avg_snr < threshold (2.0)`: moderate loss (30%–70%)
- `avg_snr ≥ threshold + 2`: low loss (5%)
- Additional drop if:
  - Node has consecutive losses (streak-based penalty)
  - Network congestion (`active_transmissions > max_inflight_packets`)

Losses are tracked per (sender, receiver) pair.

---

## Node Simulation Parameters

Each node connects to the server with:
```json
{
  "type": "register",
  "node_id": <int>,
  "location": [x, y]  // in kilometers
}
```

And sends packets like:
```json
{
  "type": "tx",
  "from": <node_id>,
  "data": "<payload>",
  "meta": {
    "destination": <to_id>,
    "tx_power": 23,
    "timestamp": 1680000000.0
  }
}
```

---

## Input Parameters Per Packet

| Parameter     | Type   | Default   | Description                                      |
|---------------|--------|-----------|--------------------------------------------------|
| `location`    | tuple  | `(0,0)`   | Node's physical position in km                  |
| `aqi`         | int    | 50        | Air Quality Index                                |
| `weather`     | str    | "clear"   | "clear", "fog", "light-rain", etc.              |
| `obstacle`    | str    | "open"    | One of the materials in OBSTACLE_LOSS_DB        |

---

## Collision & Congestion Handling

- **Collision**: If a message is received within 5 ms of the last message to the same node → mark as collision and drop
- **Congestion**: If `active_transmissions > max_inflight_packets (10)` → increase packet drop probability

---

## Logging

Server logs are written to both `simulator.log` and stdout:

- Delivered packets
- Dropped packets with reason (SNR, collision, congestion)
- Node registration and disconnects

---

## Example Run

```bash
# Start the server
$ python3 simulated_server.py

# Run two nodes in different terminals
$ python3 rfm9x_simpletest.py --simulate --id=1 --location=0,0
$ python3 rfm9x_simpletest.py --simulate --id=2 --location=0,10
```

---

## References

1. Rojas et al., "Forecasting LoRaWAN RSSI using Weather", *FGCS*, 2024.
2. NIST Report 6055: Attenuation through materials.
3. ITU-R P.525, P.838-3: FSPL and Rain Attenuation.
4. TRUE-RC Fog Attenuation Report (2022).
5. LoRaWAN Tech Specs: SNR thresholds, RSSI behavior.
6. Anzum et al., LoRa signal attenuation in foliage (2019–2022).

---
