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
TX_POWER_DBM = 23
NOISE_FLOOR_DBM = -120
SPREADING_FACTOR = 7

RAIN_DELAY_FACTORS = {
    'light-rain': 10,
    'moderate-rain': 15,
    'heavy-rain': 20,
    'fog': 5,
    'clear': 0
}

OBSTACLE_LOSS_DB = {
    "glass_6mm": 0.8,
    "glass_13mm": 2,
    "wood_76mm": 2.8,
    "brick_89mm": 3.5,
    "brick_102mm": 5,
    "brick_178mm": 7,
    "brick_267mm": 12,
    "stone_wall_203mm": 12,
    "brick_concrete_192mm": 14,
    "stone_wall_406mm": 17,
    "concrete_203mm": 23,
    "reinforced_concrete_89mm": 27,
    "stone_wall_610mm": 28,
    "concrete_305mm": 35,
    "open": 0
}

logging.basicConfig(
    level=logging.INFO,
    format='[%(asctime)s] %(levelname)s: %(message)s',
    handlers=[
        logging.FileHandler("simulator.log"),
        logging.StreamHandler(sys.stdout)
    ]
)

logger = logging.getLogger("SimulatorServer")

class SimulatorServer:
    def __init__(self, host='0.0.0.0', port=5000, max_clients=5,
                 min_delay=0.01, max_delay=0.1,
                 snr_drop_threshold=2.0):
        self.host = host
        self.port = port
        self.max_clients = max_clients
        self.min_delay = min_delay
        self.max_delay = max_delay
        self.snr_drop_threshold = snr_drop_threshold

        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.server_socket.settimeout(1.0)

        self.clients = {}
        self.node_locations = {}
        self.lock = threading.Lock()
        self.stop_event = threading.Event()
        self.shutting_down = False

        self.signal_history = defaultdict(lambda: deque(maxlen=10))
        self.loss_streaks = defaultdict(int)
        self.last_rx_time = defaultdict(float)
        self.active_transmissions = 0
        self.max_inflight_packets = 10

    def update_signal_history(self, from_id, to_id, snr):
        self.signal_history[(from_id, to_id)].append(snr)

    def average_snr(self, from_id, to_id, current_snr):
        history = self.signal_history.get((from_id, to_id), [])
        return sum(history) / len(history) if history else current_snr

    def check_collision(self, to_id):
        now = time.time()
        if abs(now - self.last_rx_time[to_id]) < 0.005:
            return True
        self.last_rx_time[to_id] = now
        return False

    def should_drop(self, from_id, to_id, snr):
        avg_snr = self.average_snr(from_id, to_id, snr)
        key = (from_id, to_id)

        if avg_snr < 0:
            base_prob = 1.0
        elif avg_snr < self.snr_drop_threshold:
            base_prob = 0.7
        elif avg_snr < self.snr_drop_threshold + 2:
            base_prob = 0.3
        else:
            base_prob = 0.05

        base_prob += min(self.loss_streaks[key] * 0.1, 0.5)
        if self.active_transmissions > self.max_inflight_packets:
            base_prob += 0.2

        drop = random.random() < min(base_prob, 1.0)
        self.loss_streaks[key] = self.loss_streaks[key] + 1 if drop else 0
        return drop

    def generate_rssi(self, base_rssi):
        noise = random.uniform(-2.0, 2.0)
        return max(-120, min(-40, base_rssi + noise))

    def compute_environmental_loss(self, distance_km, aqi, weather, obstacle):
        path_loss = 32.45 + 20 * math.log10(distance_km) + 20 * math.log10(868)
        path_loss += RAIN_DELAY_FACTORS.get(weather, 0.0) * distance_km
        if aqi > 50:
            path_loss += ((aqi - 50) / 100.0) * 0.02 * distance_km
        path_loss += OBSTACLE_LOSS_DB.get(obstacle, 0.0)
        return path_loss

    def compute_environmental_delay(self, distance_km, aqi, weather, obstacle):
        delay_ms = distance_km * 5
        delay_ms += distance_km * RAIN_DELAY_FACTORS.get(weather, 0)
        if aqi > 100:
            delay_ms += distance_km * ((aqi - 100) / 50) * 5
        if obstacle:
            delay_ms += random.uniform(5, 25)
        delay_ms += random.uniform(1, 5)
        return delay_ms

    def start(self):
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

                    with self.lock:
                        from_loc = self.node_locations.get(from_id, (0, 0))

                    aqi = meta.get("aqi", DEFAULT_AQI)
                    weather = meta.get("weather", DEFAULT_WEATHER)
                    obstacle = meta.get("obstacle", DEFAULT_OBSTACLE)

                    self.active_transmissions += 1

                    if to_id == 0xFF:
                        # Broadcast to all other nodes
                        with self.lock:
                            for nid, client_sock in self.clients.items():
                                if nid == from_id:
                                    continue
                                to_loc = self.node_locations.get(nid, (0, 0))
                                distance_km = math.dist(from_loc, to_loc)
                                delay = self.compute_environmental_delay(distance_km, aqi, weather, obstacle)
                                time.sleep(delay / 1000.0)
                                rssi = self.generate_rssi(TX_POWER_DBM - self.compute_environmental_loss(distance_km, aqi, weather, obstacle))
                                snr = rssi - NOISE_FLOOR_DBM
                                self.update_signal_history(from_id, nid, snr)
                                if self.check_collision(nid) or self.should_drop(from_id, nid, snr):
                                    logger.warning(f"[x] Dropped broadcast packet to Node {nid} (Distance={distance_km:.2f}km)")
                                    continue
                                msg["rssi"] = round(rssi, 2)
                                msg["snr"] = round(snr, 2)
                                try:
                                    client_sock.sendall((json.dumps(msg) + '\n').encode())
                                    logger.info(f"[✓] Delivered broadcast from Node {from_id} to Node {nid} (Distance={distance_km:.2f}km, RSSI={rssi:.2f}, SNR={snr:.2f})")
                                except Exception as e:
                                    logger.warning(f"[x] Failed to deliver to Node {nid}: {e}")
                    else:
                        with self.lock:
                            to_loc = self.node_locations.get(to_id, (0, 0))
                        distance_km = math.dist(from_loc, to_loc)
                        delay = self.compute_environmental_delay(distance_km, aqi, weather, obstacle)
                        time.sleep(delay / 1000.0)
                        rssi = self.generate_rssi(TX_POWER_DBM - self.compute_environmental_loss(distance_km, aqi, weather, obstacle))
                        snr = rssi - NOISE_FLOOR_DBM
                        self.update_signal_history(from_id, to_id, snr)
                        if self.check_collision(to_id) or self.should_drop(from_id, to_id, snr):
                            logger.warning(f"[x] Dropped unicast packet to Node {to_id} (Distance={distance_km:.2f}km)")
                            return
                        msg["rssi"] = round(rssi, 2)
                        msg["snr"] = round(snr, 2)
                        with self.lock:
                            target_sock = self.clients.get(to_id)
                        if target_sock:
                            try:
                                target_sock.sendall((json.dumps(msg) + '\n').encode())
                                logger.info(f"[✓] Delivered message from Node {from_id} to Node {to_id} (Distance={distance_km:.2f}km, RSSI={rssi:.2f}, SNR={snr:.2f})")
                            except Exception as e:
                                logger.error(f"[x] Failed to send to Node {to_id}: {e}")
                        else:
                            logger.warning(f"[!] Destination node {to_id} not found")

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

    def shutdown(self):
        if self.shutting_down:
            return
        self.shutting_down = True

        logger.info("\n[!] Shutting down server...")
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
