'''
LoRa RFM9x Simulator Server
===========================

This script implements a TCP-based simulation server for Adafruit's RFM9x LoRa radio module,
allowing multiple simulated nodes (e.g., running in separate terminal sessions) to send and receive
messages without requiring physical hardware.

Each simulated node connects over TCP and behaves like a real RFM9x module, using JSON messages
to transmit packets (data). The server acts as a central router (Virtual AIR gap between RFM9x Nodes)
and simulates realistic wireless behavior.

Usage:
------
1. Run this server:
   $ python simulated_server.py

2. Start multiple simulated nodes from separate terminals:
   $ python simpletest.py --simulate --id=1
   $ python simpletest.py --simulate --id=2

3. Messages sent from one node will be routed by this server to others.
'''

import socket
import threading
import json
import random
import time
import signal
import sys
import logging
import math
from collections import defaultdict, deque

# ========================== CONFIGURABLE PARAMETERS ==========================
DEFAULT_AQI = 50
DEFAULT_WEATHER = 'clear'
DEFAULT_OBSTACLE = 'open'
DEFAULT_SPREAD_FACTOR = 7
TX_POWER_DBM = 23
MAX_RANGE_KM = 25.0

SF_SNR_RANGES = {
    7: (-7.5, 12.5),
    8: (-10, 10),
    9: (-13, 7.5),
    10: (-15, 5),
    11: (-17.5, 2.5),
    12: (-20, 0),
}

WEATHER_ATTEN_DB_PER_KM = {
    'clear': 0.0,
    'fog': 0.02,
    'light-rain': 0.05,
    'moderate-rain': 0.1,
    'heavy-rain': 0.2
}

OBSTACLE_LOSS_DB = {
    "glass_6mm": 0.8, "glass_13mm": 2, "wood_76mm": 2.8,
    "brick_89mm": 3.5, "brick_102mm": 5, "brick_178mm": 7,
    "brick_267mm": 12, "stone_wall_203mm": 12, "brick_concrete_192mm": 14,
    "stone_wall_406mm": 17, "concrete_203mm": 23, "reinforced_concrete_89mm": 27,
    "stone_wall_610mm": 28, "concrete_305mm": 35, "open": 0
}
# ================================================================================

logging.basicConfig(
    level=logging.INFO,
    format='[%(asctime)s] %(levelname)s: %(message)s',
    handlers=[logging.StreamHandler(sys.stdout)]
)
logger = logging.getLogger("SimulatorServer")


class SimulatorServer:
    def __init__(self, host='0.0.0.0', port=5000, max_clients=5):
        self.host = host
        self.port = port
        self.max_clients = max_clients

        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.server_socket.settimeout(1.0)

        self.clients = {}
        self.node_locations = {}
        self.rx_busy_until = {}
        self.lock = threading.Lock()
        self.stop_event = threading.Event()
        self.shutting_down = False
        self.loss_streaks = defaultdict(int)
        self.active_transmissions = 0
        self.max_inflight_packets = 10

    def should_drop(self, from_id, to_id):
        key = (from_id, to_id)
        prob = 0.0
        if self.active_transmissions > self.max_inflight_packets:
            prob += 0.3
        prob += min(self.loss_streaks[key] * 0.05, 0.3)
        drop = random.random() < prob
        self.loss_streaks[key] = self.loss_streaks[key] + 1 if drop else 0
        return drop

    def compute_environmental_loss(self, distance_km, aqi, weather, obstacle):
        path_loss = 32.45 + 20 * math.log10(distance_km) + 20 * math.log10(868)
        path_loss += WEATHER_ATTEN_DB_PER_KM.get(weather, 0.0) * distance_km
        if aqi > 50:
            path_loss += ((aqi - 50) / 100.0) * 0.02 * distance_km
        path_loss += OBSTACLE_LOSS_DB.get(obstacle, 0.0)
        return path_loss

    def compute_airtime_ms(self, payload_len, sf=7, bw=125000, cr=1, preamble_len=8, header_enabled=True, low_datarate_optimize=None):
        tsym = (2 ** sf) / bw
        de = 1 if (low_datarate_optimize if low_datarate_optimize is not None else sf >= 11) else 0
        ih = 0 if header_enabled else 1
        payload_symb_nb = 8 + max(
            math.ceil(
                (8 * payload_len - 4 * sf + 28 + 16 - 20 * ih)
                / (4 * (sf - 2 * de))
            ) * (cr + 4),
            0,
        )
        t_air = (preamble_len + 4.25) * tsym + payload_symb_nb * tsym
        return t_air * 1000.0

    def compute_snr(self, rssi, sf, distance_km, weather, obstacle):
        """
        Compute realistic SNR for LoRa given:
        - RSSI (after path loss)
        - Spreading Factor (defines theoretical max SNR)
        - Distance (models SNR decay)
        - Weather (adds dB/km attenuation)
        - Obstacle (adds fixed attenuation)
        """
        # --- Step 1: Noise floor ---
        noise_figure = 6  # typical for SX127x
        bandwidth = 125000  # Hz
        noise_floor = -174 + 10 * math.log10(bandwidth) + noise_figure  # dBm
        # --- Step 2: Theoretical SNR upper bound from SF ---
        sf_min_snr, sf_max_snr = SF_SNR_RANGES.get(sf, (-20, 12.5))
        # --- Step 3: SNR decay by distance ---
        decay_rate_db_per_km = 0.6
        snr_decay = distance_km * decay_rate_db_per_km
        # --- Step 4: Weather impact (extra SNR loss) ---
        weather_atten_db = WEATHER_ATTEN_DB_PER_KM.get(weather, 0.0) * distance_km
        # --- Step 5: Obstacle impact (static dB loss) ---
        obstacle_loss_db = OBSTACLE_LOSS_DB.get(obstacle, 0.0)
        # --- Step 6: Raw calculated SNR ---
        raw_snr = rssi - noise_floor
        # --- Step 7: Deduct environmental degradation from ideal max SNR ---
        realistic_snr = sf_max_snr - snr_decay - weather_atten_db - obstacle_loss_db
        snr = min(realistic_snr, raw_snr)
        snr = max(snr, sf_min_snr)
        return snr
    
    def snr_penalty_sigmoid(self, snr, snr_max=12.5):
        k = 0.9  # curve steepness
        x0 = snr_max / 2.0
        sigmoid = 1.0 / (1.0 + math.exp(-k * (x0 - snr)))
        return 10.0 * sigmoid  # max 10 ms penalty

    def _handle_signal(self, sig, frame):
        self.shutdown()

    def _handle_client(self, conn, addr):
        conn_file = conn.makefile('r')
        logger.info(f"[+] Connected from {addr}")
        node_id = None
        conn.settimeout(None)

        try:
            for line in conn_file:
                try:
                    msg = json.loads(line)
                except json.JSONDecodeError:
                    continue

                if msg["type"] == "register":
                    node_id = msg["node_id"]
                    location = tuple(msg.get("location", (0, 0)))
                    with self.lock:
                        self.clients[node_id] = conn
                        self.node_locations[node_id] = location
                    logger.info(f"[+] RFM9x Node {node_id} registered at {location}")

                elif msg["type"] == "tx":
                    meta = msg.get("meta", {})
                    from_id = msg.get("from")
                    to_id = meta.get("destination")
                    from_loc = self.node_locations.get(from_id, (0, 0))
                    tx_dbm = meta.get("tx_power", TX_POWER_DBM)
                    aqi = meta.get("aqi", DEFAULT_AQI)
                    weather = meta.get("weather", DEFAULT_WEATHER)
                    obstacle = meta.get("obstacle", DEFAULT_OBSTACLE)
                    sf = meta.get("sf", DEFAULT_SPREAD_FACTOR)
                    min_snr, max_snr = SF_SNR_RANGES.get(sf, (-20, 12.5))
                    payload_len = len(msg.get("data", ""))

                    self.active_transmissions += 1
                    try:
                        targets = [(to_id, self.clients.get(to_id))] if to_id != 0xFF else [
                            (nid, sock) for nid, sock in self.clients.items() if nid != from_id
                        ]

                        for nid, client_sock in targets:
                            to_loc = self.node_locations.get(nid, (0, 0))
                            distance_km = math.dist(from_loc, to_loc)
                            path_loss = self.compute_environmental_loss(distance_km, aqi, weather, obstacle)
                            rssi = tx_dbm - path_loss
                            rssi = max(-120, min(-30, rssi))
                            snr = self.compute_snr(rssi, sf, distance_km, weather, obstacle)

                            if distance_km > MAX_RANGE_KM:
                                logger.warning(
                                    f"[DROP] OUT_OF_RANGE: Packet from Node {from_id} to Node {nid} | "
                                    f"Distance: {distance_km:.2f} km > {MAX_RANGE_KM} km"
                                )
                                continue
                            
                            # 1. Airtime
                            airtime_ms = self.compute_airtime_ms(payload_len, sf=sf, cr=1)
                            # 2. SNR penalty (simulate harder decoding under low SNR)
                            snr_penalty_ms = self.snr_penalty_sigmoid(snr,max_snr)
                            # 3. Weather/Obstacle penalty (physical media effects)
                            media_penalty_ms = WEATHER_ATTEN_DB_PER_KM[weather] * distance_km * 5.0 + OBSTACLE_LOSS_DB[obstacle] * 0.5
                            # 4. Optional random jitter for realism
                            jitter_ms = random.uniform(0.5, 1.0)
                            # 5. Total delay
                            delay_ms = airtime_ms + snr_penalty_ms + media_penalty_ms + jitter_ms
                            now = time.time()

                            drop_reason = None
                            if now < self.rx_busy_until.get(nid, 0):
                                drop_reason = "COLLISION"
                            elif snr < min_snr:
                                drop_reason = "LOW_SNR"
                            elif self.should_drop(from_id, nid):
                                drop_reason = "CONGESTION"

                            if drop_reason:
                                logger.warning(
                                    f"[DROP] Packet from Node {from_id} to Node {nid} dropped | "
                                    f"Reason: {drop_reason} | RSSI: {rssi:.2f} dBm | SNR: {snr:.2f} dB | "
                                    f"Distance: {distance_km:.2f} km | Delay: {delay_ms:.2f} ms"
                                )
                                continue

                            self.rx_busy_until[nid] = now + delay_ms / 1000.0
                            msg["rssi"] = round(rssi, 2)
                            msg["snr"] = round(snr, 2)
                            try:
                                client_sock.sendall((json.dumps(msg) + '\n').encode())
                                logger.info(
                                    f"[✓] Delivered packet from Node {from_id} to Node {nid} | "
                                    f"RSSI: {rssi:.2f} dBm | SNR: {snr:.2f} dB | Distance: {distance_km:.2f} km | "
                                    f"Delay: {delay_ms:.2f} ms"
                                )
                            except Exception as e:
                                logger.warning(f"[x] Send failed to Node {nid}: {e}")
                    finally:
                        self.active_transmissions -= 1
        finally:
            if node_id:
                with self.lock:
                    self.clients.pop(node_id, None)
            try:
                conn.shutdown(socket.SHUT_RDWR)
            except:
                pass
            conn.close()
            logger.info(f"[-] Node {node_id} disconnected")
    
    def start(self):
        """
        Start the simulator server:
        - Binds to TCP port
        - Waits for incoming client connections
        - Launches new threads to handle each client
        - Handles graceful shutdown via signal interrupt
        """
        logger.info(f"[~] Starting simulator server on {self.host}:{self.port} ...")
        self.server_socket.bind((self.host, self.port))
        self.server_socket.listen(self.max_clients)
        signal.signal(signal.SIGINT, self._handle_signal)
        try:
            while not self.stop_event.is_set():
                try:
                    conn, addr = self.server_socket.accept()
                    threading.Thread(target=self._handle_client, args=(conn, addr), daemon=True).start()
                except socket.timeout:
                    continue
        finally:
            self.shutdown()

    def shutdown(self):
        if self.shutting_down:
            return
        self.shutting_down = True
        logger.info("[!] Shutting down server...")
        self.stop_event.set()
        with self.lock:
            for sock in self.clients.values():
                try:
                    sock.shutdown(socket.SHUT_RDWR)
                    sock.close()
                except:
                    pass
            self.clients.clear()
        try:
            self.server_socket.close()
        except:
            pass
        logger.info("[✓] Server shutdown complete.")
        sys.exit(0)


if __name__ == "__main__":
    server = SimulatorServer()
    server.start()
