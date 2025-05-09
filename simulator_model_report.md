
# LoRa RFM9x Simulation Framework

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

## 📡 System Architecture Overview

### Components

1. **`simulated_server.py`** — Core simulation server that:
   - Manages LoRa node registration.
   - Routes packets.
   - Simulates delay, RSSI, SNR, collisions, congestion, and loss.
   - Models physical and environmental degradation.

2. **`simulated_rfm9x.py`** — Drop-in replacement for `adafruit_rfm9x` Python API. Implements:
   - `send()`, `receive()`, `send_with_ack()`, `last_rssi`, `last_snr`.

3. **`rfm9x_simpletest.py`** — Example script to simulate a node using the above simulated client class.

---

## 🚀 How to Use

### 1. Start the Simulation Server
```bash
python3 simulated_server.py
```

### 2. Simulate a Node
```bash
python3 rfm9x_simpletest.py --simulate --id=1 --location=0,0
python3 rfm9x_simpletest.py --simulate --id=2 --location=0,5
```

---

## 📈 Simulation Models & Formulas

### RSSI (Received Signal Strength Indicator)
```
RSSI = TxPower_dBm - (FSPL + Loss_rain + Loss_aqi + Loss_obstacle)
```

- **FSPL** (Free-Space Path Loss): `32.45 + 20log10(d_km) + 20log10(f_MHz)`
- **AQI Loss**: `((AQI - 50) / 100) * 0.02 * d_km` (Only when AQI > 50)
- **Weather Loss**: `α_weather * d_km`
- **Obstacle Loss**: Lookup from `OBSTACLE_LOSS_DB`

### SNR (Signal-to-Noise Ratio)
```
SNR = min(SNR_max, RSSI - NoiseFloor - SNR_degradation)
```

- **Noise Floor**: `-174 + 10log10(BW) + NF` where NF = 6, BW = 125kHz
- **SNR Degradation**: Includes distance decay, weather attenuation, obstacle loss

### Airtime (LoRa Time-on-Air Calculation)
```
SymbolTime = (2^SF) / Bandwidth
PayloadSymbols = ceil((8*PL - 4*SF + 28 + 16 - 20*IH) / 4(SF - 2*DE)) * (CR + 4)
TotalTime = (PreambleLength + 4.25 + PayloadSymbols) * SymbolTime
```

- Follows Semtech Time-on-Air model

### Delay Model
```
Delay = Airtime + SNR_Penalty + Environment_Delay + Jitter
```

- **SNR_Penalty**: Sigmoid function `10 / (1 + e^(-k*(SNR_mid - snr)))`
- **Environmental Delay**: 
  - `weather_attn * distance_km * 5`
  - `obstacle_loss * 0.5`
- **Jitter**: Random delay [0.5ms - 1ms]

---

## ⚠️ Packet Drop Mechanisms

### Conditions:
- **Low SNR**: Below SF’s minimum SNR threshold.
- **Congestion**: Too many simultaneous active transmissions.
- **Collision**: Node is already receiving another packet.
- **Loss Streak Penalty**: Consecutive drops increase future drop chance.

---

## 🧪 Node Simulation Parameters (`simulated_rfm9x.py`)

---

## Input Parameters Per Packet

| Parameter     | Type   | Default   | Description                                      |
|---------------|--------|-----------|--------------------------------------------------|
| `location`    | tuple  | `(0,0)`   | Node's physical position in km                  |
| `aqi`         | int    | 50        | Air Quality Index                               |
| `weather`     | str    | "clear"   | "clear", "fog", "light-rain", etc.              |
| `obstacle`    | str    | "open"    | One of the materials in OBSTACLE_LOSS_DB        |
| `tx_power`    | int    | 23        | power at which the virtual node is operating    |
| `sf`          | int    | 7         | spread factor of the node                       |

---

### Node Registration Format:
```json
{
  "type": "register",
  "node_id": 1,
  "location": [0, 0]
}
```

### Packet Transmission Format:
```json
{
  "type": "tx",
  "from": 1,
  "data": "Hello",
  "meta": {
    "destination": 2,
    "tx_power": 23,
    "aqi": 80,
    "weather": "moderate-rain",
    "obstacle": "concrete_203mm",
    "sf": 9
  }
}
```

---

## 🔧 Configuration Parameters

These are defined at the top of `simulated_server.py`:

```python
DEFAULT_AQI = 50
DEFAULT_WEATHER = 'clear'
DEFAULT_OBSTACLE = 'open'
DEFAULT_SPREAD_FACTOR = 7
MAX_RANGE_KM = 25.0
TX_POWER_DBM = 23

SF_SNR_RANGES = {
    7: (-7.5, 12.5), 8: (-10, 10), 9: (-13, 7.5),
    10: (-15, 5), 11: (-17.5, 2.5), 12: (-20, 0),
}

WEATHER_ATTEN_DB_PER_KM = {
    'clear': 0.0, 'fog': 0.02, 'light-rain': 0.05,
    'moderate-rain': 0.1, 'heavy-rain': 0.2
}

OBSTACLE_LOSS_DB = {
    "glass_6mm": 0.8, "wood_76mm": 2.8, "brick_89mm": 3.5, ...
}
```

### 🔄 To Add New Weather or Obstacle Type
Update:
```python
WEATHER_ATTEN_DB_PER_KM['hail'] = 0.25
OBSTACLE_LOSS_DB['metal_wall_3mm'] = 18.5
```

---

## 📝 Logging

Each message log shows:
- RSSI, SNR
- Drop Reason (COLLISION, LOW_SNR, CONGESTION)
- Final Delay
- Target node ID

---

## 📦 Output Fields

All received packets at client contain:
```json
{
  "data": "Hello",
  "rssi": -97.24,
  "snr": 5.7,
  "meta": { ... }
}
```

---

## 🤖 Extending the Model

To implement new behaviors (e.g., terrain mapping, mobility, link fade), extend the following methods in `SimulatorServer`:

- `compute_environmental_loss()`
- `compute_snr()`
- `calculate_transmission_delay()`
- `should_drop()`

These are designed to be modular and extensible.

---

## 📚 References

1. Semtech LoRa Modem Design Guide (TOA formula)
2. ITU-R P.525 FSPL Model
3. ITU-R P.838-3 Rain Attenuation Model
4. NIST 6055 Obstacle Attenuation Report
5. Rojas et al., "Forecasting LoRaWAN RSSI using Weather", FGCS 2024
6. LoRaWAN Tech Specs
7. Anzum et al., “LoRa Signal Attenuation under Environmental Impact” 2022
