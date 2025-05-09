
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

3. **`rfm9x_simulation_node.py`** — Example script to simulate a node using the above simulated client class.

4. **`./examples/..`** — Modified original example scripts to simulate a node using the above simulated client class.

---

## 🚀 How to Use

### 1. Start the Simulation Server
```bash
python3 simulated_server.py
```

### 2. Simulate a Node using different terminals
```bash
'''Start a receiver node:'''
python3 rfm9x_simulation_node.py --simulate --id=1 --location=0,0 --mode=rx

'''Start a transmitter node (sends a message every 2s, sends data to node 1):'''
python3 rfm9x_simulation_node.py --simulate --id=2 --location=1,1 --mode=tx --interval=2 --destination=1

'''Start a transmitter node (sends a message every 2s, sends data to all nodes):'''
python3 rfm9x_simulation_node.py --simulate --id=2 --location=1,1 --mode=tx --interval=2 --broadcast

```


---

## 📈 Updated Simulation Models & Formulas

### 📡 RSSI (Received Signal Strength Indicator)

The RSSI simulates the received signal strength after accounting for path loss and environmental degradation:

```
RSSI = TxPower_dBm - TotalPathLoss
```

Where:
```
TotalPathLoss = FSPL + AQI_Loss + Weather_Loss + Obstacle_Loss + Terrain_Loss + Multipath_Fading + NearField_Attenuation
```

- **FSPL** (Free-Space Path Loss):  
  `FSPL = 32.45 + 20 * log10(distance_km) + 20 * log10(frequency_MHz)`  
- **AQI Loss** (for AQI > 50):  
  `AQI_Loss = ((AQI - 50) / 50)^1.5 * 0.5 * distance_km * (1 - 0.02*(SF - 7))`  
- **Weather Loss**:  
  `Weather_Loss = α_weather * distance_km` where `α_weather` is from empirical table  
- **Obstacle Loss**:  
  `Obstacle_Loss = Obstacle_dB * (1 - 0.025*(SF - 7))`  
- **Terrain Loss** (distance > 1km):  
  `Terrain_Loss ≈ log(distance_km + 1) * 3 * (1 - 0.03*(SF - 7))`  
- **Multipath Fading**:  
  `Multipath = Uniform[-f, +f]` with `f = 2.5 - 0.2*(SF - 7)`  
- **Near Field Loss** (distance < 10m):  
  `NearField_Attenuation = 15 * (1 - (distance_km / 0.01))`

---

### 📶 SNR (Signal-to-Noise Ratio)

```
SNR = RSSI - (NoiseFloor + EnvironmentalNoise) + ProcessingGain - DistanceDegradation + Fading
```

- **Noise Floor**:  
  `NoiseFloor = -174 + 10 * log10(BW_Hz) + NoiseFigure` (NF = 6, BW = 125000)  
- **Urban Noise Approximation**:  
  - If `distance_km < 5`: `UrbanNoise = 3 - 0.4 * distance_km`
  - Else: `UrbanNoise = 1`
- **Processing Gain**:  
  `PG = 10 * log10(2^SF)` → gain is dampened in effective SNR  
- **Distance Degradation**:  
  `Decay = (0.45 - 0.025*(SF - 7)) * distance_km`  
- **Max SNR Limit**: capped by `SF_SNR_RANGES[SF]`
- **Fading**:  
  `Fading = Uniform[-f, +f]` where `f = 2.5 - 0.2*(SF - 7)`

---

### ⏱️ Airtime (LoRa Time-on-Air)

Follows the Semtech SX1276 datasheet:

```
SymbolTime = (2^SF) / BW
PayloadSymbols = 8 + max(ceil((8*PL - 4*SF + 28 + 16 - 20*IH)/(4*(SF - 2*DE))) * (CR + 4), 0)
TotalTime_ms = (PreambleLength + 4.25 + PayloadSymbols) * SymbolTime * 1000
```

Where:
- `SF`: Spreading Factor (7–12)  
- `BW`: Bandwidth (typically 125000 Hz)  
- `IH`: Header enabled (0) or disabled (1)  
- `DE`: Low Data Rate Optimization  
- `CR`: Coding Rate denominator minus 4 (e.g., CR=1 for 4/5)  

---

### ⏳ Delay Model

Simulates actual processing and channel delay:

```
Delay_ms = Airtime + SNR_Penalty + EnvironmentDelay + HardwareDelay + Jitter
```

- **SNR Penalty**: Sigmoid:  
  `Penalty = max_ms / (1 + e^(k*(snr - mid)))`  
  Where `k = 1.5`, `mid = SNR_min + (SNR_max - SNR_min)/3`  
- **Environment Delay**:  
  `weather_factor * distance_km * 5 + obstacle_loss * 0.5`
- **Hardware Delay**:  
  `Base = 2 + 1.5*(SF - 7)`, scaled by weather and obstacle  
- **Jitter**:  
  `Random between 0.5 – 3 ms * (SF / 7.0)`

---

---

## ⚠️ Packet Drop Mechanisms

### Drop Conditions
| Condition             | Trigger Logic |
|-----------------------|---------------|
| **Below Sensitivity** | `RSSI < SF_SENSITIVITY[SF]` |
| **Low SNR**           | `SNR < SF_SNR_RANGE[SF][0]` |
| **Out of Range**      | Exceeds SF-based soft max range |
| **Congestion**        | Based on inflight transmissions (quadratic scale) |
| **Streak Penalty**    | Incremental drop penalty for repeated failures |
| **SNR Margin Penalty**| `exp(-margin / SF_factor)` |
| **RSSI Margin Penalty**| Proportional to how close RSSI is to minimum |
| **Interference**      | Based on concurrent SF collisions |

### Drop Probability
```math
P_{drop} = P_{congestion} + P_{streak} + P_{snr} + P_{rssi} + P_{interference}
```

Final decision is `drop = random() < min(P_{drop}, 0.98)`

---

## 🧪 Node Simulation Parameters (`simulated_rfm9x.py`)

## 📥 Input Parameters Per Packet

| Parameter     | Type   | Default   | Description                                      |
|---------------|--------|-----------|--------------------------------------------------|
| `location`    | tuple  | `(0,0)`   | Node's physical position in km                  |
| `aqi`         | int    | 50        | Air Quality Index                               |
| `weather`     | str    | "clear"   | Weather conditions: "clear", "fog", "light-rain", etc. |
| `obstacle`    | str    | "open"    | Obstacle material (see `OBSTACLE_LOSS_DB`)      |
| `tx_power`    | int    | 23        | Transmit power in dBm                           |
| `sf`          | int    | 7         | LoRa spreading factor (7–12)                    |
| `destination` | int    | 1         | Destination node ID                             |
| `frequency`   | float  | 915.0     | Operating frequency in MHz                      |

---

### 📝 Node Registration Format:
```json
{
  "type": "register",
  "node_id": 1,
  "location": [0, 0]
}
```

### 📤 Packet Transmission Format:
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

## 🔧 Configuration Parameters

These are defined at the top of `simulated_server.py`:

```python
DEFAULT_AQI = 50
DEFAULT_WEATHER = 'clear'
DEFAULT_OBSTACLE = 'open'
DEFAULT_SPREAD_FACTOR = 7
MAX_RANGE_KM = 25.0
TX_POWER_DBM = 23

SF_SENSITIVITY = {
    7: -123,
    8: -126,
    9: -129,
    10: -132,
    11: -134.5,
    12: -137
}

SF_SNR_RANGES = {
    7: (-7.5, 10.0),   # Min SNR required, Max typical SNR
    8: (-10.0, 9.0),
    9: (-12.5, 8.0),
    10: (-15.0, 7.0),
    11: (-17.5, 6.0),
    12: (-20.0, 5.0),
}

SF_MAX_RANGE_KM = {
    7: 5,
    8: 8,
    9: 12,
    10: 16,
    11: 20,
    12: 25
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
