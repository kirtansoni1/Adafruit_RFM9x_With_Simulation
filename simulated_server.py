"""
LoRa RFM9x Simulator Server
===========================

This script simulates a LoRa communication environment using TCP connections.
Nodes act as LoRa modules and communicate via JSON messages routed through this central server.
The server simulates physical transmission conditions such as RSSI, SNR, and delay.

Usage:
------
1. Run this server:
   $ python simulated_server_formatted.py

2. Run multiple simulated nodes:
   $ python simpletest.py --simulate --id=1
   $ python simpletest.py --simulate --id=2
"""

# ========================== IMPORTS ==========================
import socket
import threading
import json
import random
import time
import signal
import sys
import logging
import math
from collections import defaultdict

# ========================== CONFIGURATIONS ==========================
DEFAULT_AQI = 50
DEFAULT_WEATHER = 'clear'
DEFAULT_OBSTACLE = 'open'
DEFAULT_SPREAD_FACTOR = 7
TX_POWER_DBM = 23
MAX_RANGE_KM = 25.0
NOISE_FIGURE = 6
BANDWIDTH = 125000  # Hz

SF_SNR_RANGES = {
    7: (-7.5, 12.5), 8: (-10, 10), 9: (-13, 7.5),
    10: (-15, 5), 11: (-17.5, 2.5), 12: (-20, 0),
}

WEATHER_ATTEN_DB_PER_KM = {
    'clear': 0.0, 'fog': 0.02, 'light-rain': 0.05,
    'moderate-rain': 0.1, 'heavy-rain': 0.2
}

OBSTACLE_LOSS_DB = {
    "glass_6mm": 0.8, "glass_13mm": 2, "wood_76mm": 2.8,
    "brick_89mm": 3.5, "brick_102mm": 5, "brick_178mm": 7,
    "brick_267mm": 12, "stone_wall_203mm": 12, "brick_concrete_192mm": 14,
    "stone_wall_406mm": 17, "concrete_203mm": 23, "reinforced_concrete_89mm": 27,
    "stone_wall_610mm": 28, "concrete_305mm": 35, "open": 0
}

# ========================== LOGGER SETUP ==========================
logging.basicConfig(
    level=logging.INFO,
    format='[%(asctime)s] %(levelname)s: %(message)s',
    handlers=[logging.StreamHandler(sys.stdout)]
)
logger = logging.getLogger("SimulatorServer")


# ========================== SIMULATOR SERVER ==========================
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
        """
        Determine whether to probabilistically drop a packet between two nodes.

        This simulates MAC-layer congestion using:
        1. Current network load (inflight transmissions)
        2. Historical drop streak for this (from_id, to_id) pair

        Returns:
            bool: True if packet should be dropped, False otherwise.
        """
        key = (from_id, to_id)
        prob = 0.0
        # 1: Apply base drop chance if current inflight transmissions exceed allowed threshold
        if self.active_transmissions > self.max_inflight_packets:
            prob += 0.3 # 30% chance of drop due to inflight congestion
        # 2: Increase drop probability with streak of prior drops for this node pair
        prob += min(self.loss_streaks[key] * 0.05, 0.3) # Each consecutive failure increases drop chance linearly up to 30%
        # 3: Perform probabilistic drop decision
        drop = random.random() < prob
        # 4: Update the drop streak history
        self.loss_streaks[key] = self.loss_streaks[key] + 1 if drop else 0
        return drop

    def compute_environmental_loss(self, distance_km, aqi, weather, obstacle):
        """
        Calculate total signal loss (in dB) from transmitter to receiver due to:
        1. Free-space path loss (ITU FSPL model)
        2. Weather attenuation (rain, fog, etc.)
        3. AQI-related degradation (if AQI > 50)
        4. Fixed obstacle attenuation (walls, glass, concrete)

        Args:
            distance_km (float): Distance between nodes in kilometers.
            aqi (int): Air Quality Index.
            weather (str): Weather condition key from lookup table.
            obstacle (str): Obstacle type key from lookup table.

        Returns:
            float: Total environmental path loss in dB.
        """
        # 1: Free-space path loss using ITU standard (assumes 868 MHz LoRa)
        # FSPL(dB) = 32.45 + 20*log10(distance_km) + 20*log10(frequency_MHz)
        path_loss = 32.45 + 20 * math.log10(distance_km) + 20 * math.log10(868)
        # 2: Weather attenuation (rain, fog etc) in dB/km multiplied by distance
        path_loss += WEATHER_ATTEN_DB_PER_KM.get(weather, 0.0) * distance_km
        # 3: AQI-based atmospheric degradation (only if AQI > 50)
        if aqi > 50:
            path_loss += ((aqi - 50) / 100.0) * 0.02 * distance_km # Scales linearly with excess AQI
        # 4: Add obstacle penetration loss from empirical dB table
        path_loss += OBSTACLE_LOSS_DB.get(obstacle, 0.0)
        return path_loss

    def compute_airtime_ms(self, payload_len, sf=7, bw=125000, cr=1, preamble_len=8, header_enabled=True, low_datarate_optimize=None):
        """
        Compute the airtime (in milliseconds) for a given LoRa packet.

        Based on Semtech's Time-on-Air formula. Includes:
        1. Symbol time (based on SF and BW)
        2. Low data rate optimization
        3. Header type (explicit/implicit)
        4. Coding rate and payload size

        Args:
            payload_len (int): Payload length in bytes.
            sf (int): Spreading factor (7–12).
            bw (int): Bandwidth in Hz.
            cr (int): Coding rate denominator (e.g. 1 = 4/5).
            preamble_len (int): LoRa preamble symbol length.
            header_enabled (bool): True = explicit header.
            low_datarate_optimize (bool or None): Forces optimization if True, auto if None.

        Returns:
            float: Estimated airtime in milliseconds.
        """
        # 1: Symbol duration calculation
        tsym = (2 ** sf) / bw
        # 2: Low data rate optimization enable flag
        de = 1 if (low_datarate_optimize if low_datarate_optimize is not None else sf >= 11) else 0
        # 3: Header mode (0=explicit, 1=implicit)
        ih = 0 if header_enabled else 1
        # 4: Calculate payload symbol count
        payload_symb_nb = 8 + max(
            math.ceil((8 * payload_len - 4 * sf + 28 + 16 - 20 * ih) / (4 * (sf - 2 * de))) * (cr + 4),
            0,
        )
        # 5: Total airtime = preamble + payload
        t_air = (preamble_len + 4.25) * tsym + payload_symb_nb * tsym
        return t_air * 1000.0

    def compute_snr(self, rssi, sf, distance_km, weather, obstacle):
        """
        Estimate realistic Signal-to-Noise Ratio (SNR) based on environment and SF.

        Steps:
        1. Calculate noise floor (thermal + receiver noise)
        2. Clamp raw SNR from RSSI
        3. Deduct environmental effects:
            - Distance decay (empirical)
            - Weather attenuation (db/km)
            - Obstacle loss (fixed dB)

        Args:
            rssi (float): Received Signal Strength Indicator in dBm.
            sf (int): Spreading factor (7–12).
            distance_km (float): Distance between nodes in kilometers.
            weather (str): Weather condition key.
            obstacle (str): Obstacle key.

        Returns:
            float: Estimated SNR value within SF bounds.
        """
        # 1: Calculate noise floor (thermal + receiver NF)
        noise_floor = -174 + 10 * math.log10(BANDWIDTH) + NOISE_FIGURE
        # 2: Fetch SNR bounds from SF definition
        sf_min_snr, sf_max_snr = SF_SNR_RANGES.get(sf, (-20, 12.5))
        decay_rate = 0.6
        # 3: Calculate raw SNR from RSSI - noise floor
        raw_snr = rssi - noise_floor
        # 4: Adjust SNR based on environmental losses
        realistic_snr = sf_max_snr - decay_rate * distance_km - WEATHER_ATTEN_DB_PER_KM[weather] * distance_km - OBSTACLE_LOSS_DB[obstacle]
        return max(sf_min_snr, min(realistic_snr, raw_snr))

    def snr_penalty_sigmoid(self, snr, snr_max=12.5):
        """
        Map SNR to a delay penalty using a sigmoid curve for smooth degradation.

        Higher SNR → lower decoding delay, modeled smoothly.
        Used to penalize low-SNR links realistically.

        Args:
            snr (float): Current SNR value.
            snr_max (float): Ideal/maximum SNR.

        Returns:
            float: Delay penalty in milliseconds (max ~10 ms).
        """
        k = 0.9
        x0 = snr_max / 2.0
        return 10.0 / (1.0 + math.exp(-k * (x0 - snr)))
    
    def calculate_transmission_delay(self, snr, sf, weather, distance_km, obstacle, payload_len, snr_max):
        """
        Calculate the full simulated transmission delay of a LoRa packet.

        Includes:
        1. LoRa modem airtime
        2. SNR-based decoding penalty
        3. Physical environment delay (weather/obstacle)
        4. Random jitter for realism

        Args:
            snr (float): Signal-to-noise ratio.
            sf (int): Spreading factor.
            weather (str): Weather condition.
            distance_km (float): Distance between nodes in km.
            obstacle (str): Obstacle type.
            payload_len (int): Payload length in bytes.
            snr_max (float): Max SNR allowed for this SF.

        Returns:
            float: Total delay in milliseconds.
        """
        # 1. Calculate Airtime
        airtime_ms = self.compute_airtime_ms(payload_len, sf=sf, cr=1)
        # 2. SNR penalty (simulate harder decoding under low SNR)
        snr_penalty_ms = self.snr_penalty_sigmoid(snr, snr_max)
        # 3. Weather/Obstacle penalty (physical media effects)
        media_penalty_ms = WEATHER_ATTEN_DB_PER_KM[weather] * distance_km * 5.0 + OBSTACLE_LOSS_DB[obstacle] * 0.5
        # 4. Optional random jitter for realism
        jitter_ms = random.uniform(0.5, 1.0)
        # 5. Total delay
        delay_ms = airtime_ms + snr_penalty_ms + media_penalty_ms + jitter_ms
        return delay_ms
    
    def get_drop_reason(self, now, nid, snr, min_snr, from_id):
        """
        Determine reason for packet drop, if any.
        Checks:
        1. Whether node is currently busy (collision)
        2. Whether SNR is below SF threshold (low SNR)
        3. Whether congestion model triggers a drop

        Args:
            now (float): Current timestamp.
            nid (int): Target node ID.
            snr (float): Computed SNR.
            min_snr (float): SF's minimum viable SNR.
            from_id (int): Sender node ID.

        Returns:
            str or None: Drop reason ("COLLISION", "LOW_SNR", "CONGESTION") or None if no drop.
        """
        drop_reason = None
        if now < self.rx_busy_until.get(nid, 0):
            drop_reason = "COLLISION"
        elif snr < min_snr:
            drop_reason = "LOW_SNR"
        elif self.should_drop(from_id, nid):
            drop_reason = "CONGESTION"
        return drop_reason

    def _handle_signal(self, sig, frame):
        """
        Handle system signals (like Ctrl+C) for graceful shutdown.
        """
        self.shutdown()

    def _handle_client(self, conn, addr):
        """
        Handle communication from a single simulated LoRa node.
        Handles:
        - Node registration with coordinates
        - Packet transmissions with metadata
        - Disconnect and cleanup

        Runs in its own thread per connection.
        """
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
                    self._process_transmission(msg)
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

    def _process_transmission(self, msg):
        """
        Core transmission logic triggered on every 'tx' message.

        Steps:
        1. Extract metadata and node positions
        2. Compute path loss, RSSI, SNR
        3. Check distance limits
        4. Calculate delay (airtime + penalties)
        5. Apply packet drop logic
        6. Forward packet to target client with final metadata

        Args:
            msg (dict): Transmission message from sender.
        """
        meta = msg.get("meta", {})
        from_id = msg.get("from")
        to_id = meta.get("destination")
        from_loc = self.node_locations.get(from_id, (0, 0))
        tx_dbm = meta.get("tx_power", TX_POWER_DBM)
        aqi = meta.get("aqi", DEFAULT_AQI)
        weather = meta.get("weather", DEFAULT_WEATHER)
        obstacle = meta.get("obstacle", DEFAULT_OBSTACLE)
        sf = meta.get("sf", DEFAULT_SPREAD_FACTOR)
        payload_len = len(msg.get("data", ""))
        min_snr, max_snr = SF_SNR_RANGES.get(sf, (-20, 12.5))

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
                    logger.warning(f"[DROP] OUT_OF_RANGE: Packet from Node {from_id} to Node {nid} | Distance: {distance_km:.2f} km")
                    continue
                
                delay_ms = self.calculate_transmission_delay(snr, sf, weather, distance_km, obstacle, payload_len, max_snr)
                now = time.time()

                drop_reason = self.get_drop_reason(now, nid, snr, min_snr, from_id)
                if drop_reason:
                    logger.warning(f"[DROP] Packet from {from_id} to {nid} dropped | Reason: {drop_reason} | RSSI: {rssi:.2f} | SNR: {snr:.2f} | Distance: {distance_km:.2f} km | Delay: {delay_ms:.2f} ms")
                    continue

                self.rx_busy_until[nid] = now + delay_ms / 1000.0
                msg["rssi"] = round(rssi, 2)
                msg["snr"] = round(snr, 2)
                try:
                    client_sock.sendall((json.dumps(msg) + '\n').encode())
                    logger.info(f"[✓] Delivered packet from {from_id} to {nid} | RSSI: {rssi:.2f} | SNR: {snr:.2f} | Distance: {distance_km:.2f} km | Delay: {delay_ms:.2f} ms")
                except Exception as e:
                    logger.warning(f"[x] Send failed to Node {nid}: {e}")
        finally:
            self.active_transmissions -= 1

    def start(self):
        """
        Start the server loop:
        - Accept TCP connections
        - Launch threads for client
        - Handle Ctrl+C for shutdown
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
        """
        Cleanly shutdown the server and all connected clients.
        """
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
