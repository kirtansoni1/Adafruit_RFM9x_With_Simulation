"""
LoRa RFM9x Simulator Server
===========================

This script simulates a LoRa communication environment using TCP connections.
Nodes act as LoRa modules and communicate via JSON messages routed through this central server.
The server simulates physical transmission conditions such as RSSI, SNR, and delay.

Usage:
------
1. Run this server:
   $ python simulated_server_improved.py

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
NOISE_FIGURE = 6
BANDWIDTH = 125000  # Hz
FREQUENCY_MHZ = 915  # LoRa frequency in MHz

# Based on Semtech SX1276 datasheet and field measurements
SF_SENSITIVITY = {
    7: -123,
    8: -126,
    9: -129,
    10: -132,
    11: -134.5,
    12: -137
}

# Theoretical and practical SNR requirements with ranges for different SFs
SF_SNR_RANGES = {
    7: (-7.5, 10.0),   # Min SNR required, Max typical SNR
    8: (-10.0, 9.0),
    9: (-12.5, 8.0),
    10: (-15.0, 7.0),
    11: (-17.5, 6.0),
    12: (-20.0, 5.0),
}

# Maximum effective communication ranges by SF in ideal conditions (km)
# These act as soft limits - signal quality degrades exponentially beyond these
SF_MAX_RANGE_KM = {
    7: 5,
    8: 8,
    9: 12,
    10: 16,
    11: 20,
    12: 25
}

# Weather attenuation (db/km) - values based on empirical RF propagation models
WEATHER_ATTEN_DB_PER_KM = {
    'clear': 0.2,        # Even clear air has some attenuation
    'fog': 0.4,          # Fog increases attenuation
    'light-rain': 0.7,   # Light rain (< 5mm/h)
    'moderate-rain': 1.2, # Moderate rain (5-10mm/h)
    'heavy-rain': 2.0    # Heavy rain (>10mm/h)
}

# Obstacle loss in dB - realistic values for 915MHz from ITU-R and RF literature
OBSTACLE_LOSS_DB = {
    "glass_6mm": 0.8, 
    "glass_13mm": 2.0, 
    "wood_76mm": 3.5,
    "brick_89mm": 4.5, 
    "brick_102mm": 6.0, 
    "brick_178mm": 8.5,
    "brick_267mm": 14.0, 
    "stone_wall_203mm": 15.0, 
    "brick_concrete_192mm": 18.0,
    "stone_wall_406mm": 24.0, 
    "concrete_203mm": 28.0, 
    "reinforced_concrete_89mm": 32.0,
    "stone_wall_610mm": 36.0, 
    "concrete_305mm": 42.0, 
    "open": 0.0
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
        self.node_frequency = {}
        self.rx_busy_until = {}
        self.lock = threading.Lock()
        self.stop_event = threading.Event()
        self.shutting_down = False
        self.loss_streaks = defaultdict(int)
        self.active_transmissions = 0
        self.max_inflight_packets = 10
        
        # Thermal noise floor (dBm) = -174 + 10*log10(BW)
        self.noise_floor_dbm = -174 + 10 * math.log10(BANDWIDTH) + NOISE_FIGURE

    def should_drop(self, from_id, to_id, rssi, snr, sf, distance_km):
        """
        Calculate packet drop probability based on:
        - Congestion (active transmissions)
        - Historical loss streaks
        - Physical link quality (RSSI, SNR, SF)
        - SF-specific factors
        - SNR margin compared to threshold

        Returns:
            bool: True if packet should be dropped, False otherwise.
        """
        key = (from_id, to_id)

        # 1. Below sensitivity threshold = guaranteed drop
        if rssi < SF_SENSITIVITY[sf]:
            return True
            
        # 2. Below minimum SNR threshold = guaranteed drop 
        min_snr, _ = SF_SNR_RANGES.get(sf, (-20, 5.0))
        if snr < min_snr:
            return True
            
        # 3. Above maximum theoretical range = high probability drop
        max_range = SF_MAX_RANGE_KM.get(sf, 10.0)
        
        # Distance-based probability increases dramatically beyond max range
        distance_ratio = distance_km / max_range
        if distance_ratio > 1.0:
            # Exponential increase in drop probability beyond max range
            distance_prob = min(0.95, (distance_ratio - 1.0) ** 2 * 0.9)
            if random.random() < distance_prob:
                return True

        # 4. Congestion-based drop probability
        inflight_ratio = self.active_transmissions / self.max_inflight_packets
        congestion_prob = min(inflight_ratio * inflight_ratio, 1.0) * 0.5  # Quadratic scaling up to 50%

        # 5. Streak penalty from history (bad links tend to stay bad)
        streak_prob = min(self.loss_streaks[key] * 0.07, 0.35)  # Caps at 35%

        # 6. SNR margin penalty (smaller margin = higher drop probability)
        snr_margin = snr - min_snr
        # Higher SF is more resilient to poor SNR margins
        snr_margin_factor = 4.0 + ((sf - 7) * 0.25)  # SF7: 4.0, SF12: 5.25
        snr_prob = math.exp(-snr_margin / snr_margin_factor) * 0.6  # Exponential decay

        # 7. RSSI quality penalty
        rssi_threshold = SF_SENSITIVITY[sf] + 5  # 5dB above sensitivity
        rssi_margin = rssi - rssi_threshold
        rssi_prob = 0.0 if rssi_margin > 0 else min(abs(rssi_margin / 10), 1.0) * 0.4

        # 8. SF-specific interference probability
        # Higher SF more susceptible to interference but resistant to noise
        # This includes channel usage and co-SF interference
        base_interference = 0.03 * self.active_transmissions / self.max_inflight_packets
        sf_interference_factor = {
            7: 0.7,  # Less affected (faster transmissions = less collision opportunity)
            8: 0.8,
            9: 0.9,
            10: 1.0,
            11: 1.1,
            12: 1.2   # More affected (longer airtime = more collision opportunity)
        }.get(sf, 1.0)
        interference_prob = base_interference * sf_interference_factor
        
        # 9. Total drop probability with random factor
        base_prob = congestion_prob + streak_prob + snr_prob + rssi_prob + interference_prob
        # Add some randomness but cap at 98%
        prob = min(base_prob, 0.98)
        
        # Execute the drop decision
        drop = random.random() < prob

        # Update loss streak history
        self.loss_streaks[key] = self.loss_streaks[key] + 1 if drop else 0
        
        # Debug log for drop probability components
        if random.random() < 0.05:  # Only log 5% of packets to avoid log spam
            logger.debug(f"Drop probability factors: congestion={congestion_prob:.2f}, "
                        f"streak={streak_prob:.2f}, snr={snr_prob:.2f}, rssi={rssi_prob:.2f}, "
                        f"interference={interference_prob:.2f}, SF={sf}, "
                        f"total={prob:.2f}, decision={'DROP' if drop else 'KEEP'}")
            
        return drop

    
    def compute_environmental_loss(self, from_id, to_id, distance_km, aqi, weather, obstacle, sf=7):
        """
        Calculate total signal loss (in dB) from transmitter to receiver, 
        with SF-specific characteristics.
        
        Args:
            from_id (int): Sender node ID
            to_id (int): Receiver node ID
            distance_km (float): Distance between nodes in kilometers.
            aqi (int): Air Quality Index.
            weather (str): Weather condition key from lookup table.
            obstacle (str): Obstacle type key from lookup table.
            sf (int): Spreading factor (7-12)

        Returns:
            float: Total environmental path loss in dB.
        """
        # Realistic minimum distance (set to 2 meters = 0.002 km)
        min_distance_km = 0.002
        
        # Apply minimum path loss even at zero distance (device separation/antenna characteristics)
        # This represents the mismatch and inefficiency in real-world radio systems
        min_path_loss = 32.0
        
        # 1: Set a realistic minimum distance and enforce realistic close-range behavior
        # Even at very close ranges, we never exceed realistic signal strength 
        effective_distance_km = max(distance_km, min_distance_km)
        
        # Add a near-field attenuation factor for very close distances
        # Real antennas don't follow the inverse square law in the near field
        near_field_attenuation = 0
        if distance_km < 0.010:  # Within 10 meters
            # Progressive attenuation that increases as we get closer
            near_field_attenuation = 15.0 * (1.0 - (distance_km / 0.010))
        
        # 2: Free-space path loss using ITU standard
        # FSPL(dB) = 32.45 + 20*log10(distance_km) + 20*log10(frequency_MHz)
        path_loss = 32.45 + 20 * math.log10(effective_distance_km) + 20 * math.log10(FREQUENCY_MHZ)
        
        # Add near-field component
        path_loss += near_field_attenuation
        
        # 3: Weather attenuation (rain, fog etc) in dB/km multiplied by distance
        # Higher SF slightly more resilient to weather effects (longer symbol time)
        weather_base = WEATHER_ATTEN_DB_PER_KM.get(weather, 0.2)
        sf_weather_reduction = (sf - 7) * 0.01  # Small reduction for higher SF
        weather_factor = max(0.1, weather_base * (1.0 - sf_weather_reduction))
        path_loss += weather_factor * effective_distance_km
        
        # 4: AQI-based atmospheric degradation (non-linear effect)
        if aqi > 50:
            # Non-linear scaling - higher AQI has increasingly worse effect
            # Higher SF slightly more resilient to particulate interference
            aqi_factor = ((aqi - 50) / 50.0) ** 1.5
            sf_aqi_reduction = (sf - 7) * 0.02  # Small reduction for higher SF
            path_loss += aqi_factor * 0.5 * effective_distance_km * (1.0 - sf_aqi_reduction)
            
        # 5: Add obstacle penetration loss from empirical dB table
        # Higher SF has better obstacle penetration
        obstacle_loss = OBSTACLE_LOSS_DB.get(obstacle, 0.0)
        if obstacle != "open":
            # Up to 15% better penetration at SF12 compared to SF7
            sf_penetration_factor = 1.0 - ((sf - 7) * 0.025)  # SF7: 1.0, SF12: 0.875
            path_loss += obstacle_loss * sf_penetration_factor
        else:
            path_loss += obstacle_loss
        
        # 6: Earth curvature effect - significant beyond ~8km
        # Affects all SFs similarly (physics of radio horizon)
        if effective_distance_km > 8.0:
            # Stronger effect as we approach radio horizon
            curvature_loss = ((effective_distance_km - 8.0) / 17.0) ** 2 * 10.0
            path_loss += curvature_loss
            
        # 7: Terrain roughness approximation - varies with distance
        # Higher SF slightly better in rough terrain
        if effective_distance_km > 1.0:
            # Random but deterministic terrain effect
            roughness_seed = hash(f"{effective_distance_km:.1f}") % 1000 / 1000.0
            base_roughness = roughness_seed * 3.0 * math.log(effective_distance_km + 1)
            sf_roughness_reduction = (sf - 7) * 0.03  # Small reduction for higher SF
            roughness_loss = base_roughness * (1.0 - sf_roughness_reduction)
            path_loss += roughness_loss
            
        # 8: Multipath fading - affects signal more in complex environments
        # Higher SF has better resistance to multipath effects
        if obstacle != "open":
            # More pronounced in non-open environments
            base_multipath = 2.5
        else:
            # Still present in open areas but less intense
            base_multipath = 0.8
        
        # SF-specific multipath resistance
        multipath_factor = base_multipath * (1.0 - ((sf - 7) * 0.05))  # SF7: full effect, SF12: 75% effect
        
        # Random but deterministic multipath component
        multipath_seed = hash(f"{from_id}{to_id}{effective_distance_km:.2f}") % 1000 / 1000.0
        multipath_loss = multipath_factor * multipath_seed * 5.0
        path_loss += multipath_loss
        
        # 9: Apply a constant minimum loss to ensure RSSI is realistic even at zero distance
        return max(path_loss, min_path_loss)

    def compute_snr(self, rssi, sf, distance_km, weather, obstacle):
        """
        Calculate Signal-to-Noise Ratio based on:
        1. RSSI (received signal strength)
        2. Noise floor (thermal + environmental)
        3. Distance-based noise increase
        4. Weather-dependent noise factors
        5. SF-specific processing gain

        Args:
            rssi (float): Received Signal Strength Indicator in dBm.
            sf (int): Spreading factor (7–12).
            distance_km (float): Distance between nodes in kilometers.
            weather (str): Weather condition key.
            obstacle (str): Obstacle key.

        Returns:
            float: Calculated SNR value.
        """
        # 1. Base noise calculation from thermal noise floor
        noise_power = self.noise_floor_dbm
        
        # 2. Environmental noise factors
        # Weather increases noise (electrical storms, rain static, etc.)
        weather_noise_addition = {
            'clear': 0,
            'fog': 0.5,
            'light-rain': 1.0,
            'moderate-rain': 2.0,
            'heavy-rain': 3.5
        }
        
        # 3. Distance-based noise increase (urban/industrial sources)
        # Noise tends to increase in more populated areas
        if distance_km < 5.0:
            # Short range likely means urban/industrial environment
            urban_noise = 3.0 - (distance_km * 0.4)  # More noise in urban areas
        else:
            # Long range likely means rural environment
            urban_noise = 1.0
            
        # 4. Aggregate noise components
        noise_power += weather_noise_addition.get(weather, 0)
        noise_power += urban_noise
        
        # 5. LoRa processing gain improves SNR for higher SF
        # Processing gain = 10 * log10(2^SF)
        processing_gain = 10 * math.log10(2**sf)
        
        # 6. Calculate raw SNR
        raw_snr = rssi - noise_power

        # Constrain final SNR to realistic bounds for the given SF
        min_snr, max_snr = SF_SNR_RANGES.get(sf, (-20, 5.0))
        
        # 7. Apply SF processing gain to get effective SNR
        # Higher SF has better processing gain but still constrained by physics
        effective_snr = min(raw_snr + (processing_gain / 10), max_snr)  # Dampened processing gain effect
        
        # 8. Calculate realistic SNR degradation with distance - scales with SF 
        # Higher SFs degrade more slowly with distance (key LoRa advantage)
        sf_distance_factor = 0.45 - ((sf - 7) * 0.025)  # SF7: 0.45, SF12: 0.325
        distance_degradation = sf_distance_factor * distance_km  # dB per km
        realistic_snr = effective_snr - distance_degradation
        
        # 9. SNR decay curve specific to this SF
        # Each SF has maximum theoretical range where it still works (soft limit)
        max_range = SF_MAX_RANGE_KM.get(sf, 10.0)
        if distance_km > max_range * 0.5:
            # Calculate how far we are into the decay region (50%-100% of max range)
            decay_progress = min(1.0, (distance_km - (max_range * 0.5)) / (max_range * 0.5))
            # Apply non-linear decay curve as we approach max range
            decay_factor = decay_progress ** 1.5  # Steeper drop-off as we approach max range
            snr_range = max_snr - min_snr
            max_possible_snr = max_snr - (decay_factor * snr_range)
            realistic_snr = min(realistic_snr, max_possible_snr)
        
        # 10. Apply random but deterministic SF-specific fading component
        fading_seed = hash(f"{distance_km:.1f}-{sf}") % 1000 / 1000.0
        # Higher SF has less fading variation (more stable links)
        fading_intensity = 2.5 - ((sf - 7) * 0.2)  # SF7: 2.5dB, SF12: 1.5dB
        fading_component = (fading_seed * 2 - 1) * fading_intensity  # -intensity to +intensity range
        realistic_snr += fading_component
        
        # 11. Final SNR is constrained to realistic range
        # but can still fall below minimum (causing packet loss)
        return realistic_snr + random.uniform(-0.1, 0.1)

    def compute_airtime_ms(self, payload_len, sf=7, bw=125000, cr=1, preamble_len=8, header_enabled=True, low_datarate_optimize=None):
        """
        Compute the airtime (in milliseconds) for a given LoRa packet.
        Improved version based on LoRaWAN standard and Semtech documentation.

        Args:
            payload_len (int): Payload length in bytes.
            sf (int): Spreading factor (7–12).
            bw (int): Bandwidth in Hz.
            cr (int): Coding rate denominator (4/x where x is 5-8) - 1 means 4/5.
            preamble_len (int): LoRa preamble symbol length.
            header_enabled (bool): True = explicit header.
            low_datarate_optimize (bool or None): Forces optimization if True, auto if None.

        Returns:
            float: Estimated airtime in milliseconds.
        """
        # 1: Symbol duration calculation (Ts = 2^SF / BW)
        tsym = (2 ** sf) / bw
        
        # 2: Low data rate optimization enable flag - automatic for SF11 and SF12
        # per LoRaWAN regional parameters
        de = 1 if (low_datarate_optimize if low_datarate_optimize is not None else sf >= 11) else 0
        
        # 3: Header mode (0=explicit, 1=implicit)
        ih = 0 if header_enabled else 1
        
        # 4: Calculate payload symbol count
        # n_payload = 8 + max(ceil((8*payload_len - 4*SF + 28 + 16 - 20*IH)/(4*(SF-2*DE))), 0)*(CR+4)
        # This follows Semtech's SX1276 datasheet formula
        n_payload = 8 + max(
            math.ceil((8 * payload_len - 4 * sf + 28 + 16 - 20 * ih) / (4 * (sf - 2 * de))) * (cr + 4),
            0,
        )
        
        # 5: Preamble duration
        t_preamble = (preamble_len + 4.25) * tsym
        
        # 6: Payload duration
        t_payload = n_payload * tsym
        
        # 7: Total airtime (ms)
        t_air = (t_preamble + t_payload) * 1000.0
        
        return t_air

    def snr_penalty_sigmoid(self, snr, snr_min, snr_max):
        """
        Map SNR to a delay penalty using a sigmoid curve.
        
        As SNR approaches minimum required value, decode time increases dramatically.
        Provides more realistic modeling of receiver performance near sensitivity edge.

        Args:
            snr (float): Current SNR value.
            snr_min (float): Minimum required SNR for the SF.
            snr_max (float): Maximum typical SNR for the SF.

        Returns:
            float: Delay penalty in milliseconds.
        """
        # Center point of sigmoid is 1/3 of the way from min to max
        mid_point = snr_min + (snr_max - snr_min) / 3
        
        # Steepness of curve - sharper near minimum SNR
        k = 1.5
        
        # Maximum penalty (in ms) - approached as SNR nears minimum
        max_penalty = 50
        
        # Sigmoid function: penalty = max_penalty / (1 + e^(k*(snr-mid)))
        penalty = max_penalty / (1.0 + math.exp(k * (snr - mid_point)))
        
        return penalty
    
    def calculate_transmission_delay(self, snr, sf, weather, distance_km, obstacle, payload_len):
        """
        Calculate the realistic transmission delay for a LoRa packet.
        Accounts for SF-specific characteristics in delay calculations.

        Args:
            snr (float): Signal-to-noise ratio.
            sf (int): Spreading factor.
            weather (str): Weather condition.
            distance_km (float): Distance between nodes in km.
            obstacle (str): Obstacle type.
            payload_len (int): Payload length in bytes.

        Returns:
            float: Total delay in milliseconds.
        """
        # 1. Physical airtime (time-on-air) - core component
        # Coding rate (CR) is typically 4/5 for standard LoRa
        coding_rate = 1
        airtime_ms = self.compute_airtime_ms(payload_len, sf=sf, cr=coding_rate)
        
        # 2. Propagation delay (speed of light) - minor but realistic
        # RF travels at ~300,000 km/s, so 3.33μs per km
        propagation_delay_ms = (distance_km / 300000) * 1000
        
        # 3. SNR-based decoding delay (harder to decode weak signals)
        min_snr, max_snr = SF_SNR_RANGES.get(sf, (-20, 5.0))
        snr_penalty_ms = self.snr_penalty_sigmoid(snr, min_snr, max_snr)
        
        # 4. Weather-based delay effects - slightly less impact on higher SF
        weather_base = {
            'clear': 1.0,
            'fog': 1.05,
            'light-rain': 1.1,
            'moderate-rain': 1.2,
            'heavy-rain': 1.35
        }.get(weather, 1.0)
        
        # Higher SF slightly more resilient to weather
        sf_weather_reduction = (sf - 7) * 0.01  # Small reduction for higher SF
        weather_factor = weather_base * (1.0 - sf_weather_reduction)
        
        # 5. Obstacle effect on signal quality and thus processing time
        # Higher SF works better through obstacles
        obstacle_factor = 1.0
        if obstacle != "open":
            # Non-open environments increase processing complexity
            base_obstacle_factor = 1.0 + (OBSTACLE_LOSS_DB.get(obstacle, 0.0) / 50.0)
            # SF-specific obstacle handling (higher SF = better penetration)
            sf_obstacle_reduction = (sf - 7) * 0.02  # Small reduction for higher SF
            obstacle_factor = base_obstacle_factor * (1.0 - sf_obstacle_reduction)
        
        # 6. Base processing delay (hardware dependent)
        # Higher SF requires more significant processing time
        base_processing_ms = 2.0 + (sf - 7) * 1.5  # Higher SF = more processing
        
        # 7. Synthesize processing components
        processing_delay_ms = base_processing_ms * weather_factor * obstacle_factor
        
        # 8. Random jitter for realism (hardware/stack timing variance)
        # Higher SF is more susceptible to timing jitter
        jitter_ms = random.uniform(0.5, 3.0) * (sf / 7.0)  # More jitter at higher SF
        
        # 9. Calculate total delay
        total_delay_ms = (airtime_ms + 
                        propagation_delay_ms + 
                        snr_penalty_ms + 
                        processing_delay_ms + 
                        jitter_ms)
        
        return total_delay_ms
    
    def get_drop_reason(self, now, rssi, sf, nid, snr, min_snr, from_id, distance_km):
        """
        Determine specific reason for packet drop, if any.
        
        Args:
            now (float): Current timestamp.
            rssi (float): Received signal strength.
            sf (int): Spreading factor.
            nid (int): Target node ID.
            snr (float): Signal-to-noise ratio.
            min_snr (float): Minimum viable SNR for this SF.
            from_id (int): Sender node ID.

        Returns:
            str or None: Drop reason or None if no drop.
        """
        # 1. Check for ongoing reception/collision
        if now < self.rx_busy_until.get(nid, 0):
            return "COLLISION"
            
        # 2. Check if below hardware sensitivity threshold
        if rssi < SF_SENSITIVITY[sf]:
            return "RSSI_TOO_LOW"
            
        # 3. Check if SNR is below minimum required for demodulation
        if snr < min_snr:
            return "SNR_TOO_LOW"
            
        # 4. Check if general packet drop conditions apply (statistical)
        if self.should_drop(from_id, nid, rssi, snr, sf, distance_km):
            # Determine more specific reason for the drop
            if self.active_transmissions > self.max_inflight_packets * 0.8:
                return "NETWORK_CONGESTION"
            elif self.loss_streaks.get((from_id, nid), 0) > 3:
                return "PERSISTENT_LINK_FAILURE"
            elif snr < min_snr + 3:  # Within 3dB of minimum
                return "MARGINAL_SNR"
            else:
                return "RANDOM_LOSS"
                
        # No drop
        return None

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
                    frequency = msg["frequency"]
                    with self.lock:
                        self.clients[node_id] = conn
                        self.node_locations[node_id] = location
                        self.node_frequency[node_id] = frequency
                    logger.info(f"[+] RFM9x Node {node_id} registered at {location} with frequency: {frequency}")
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
        4. Check Frequency match
        5. Calculate delay (airtime + penalties)
        6. Apply packet drop logic
        7. Forward packet to target client with final metadata

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
        min_snr, max_snr = SF_SNR_RANGES.get(sf, (-20, 5.0))
        sender_freq = self.node_frequency.get(from_id)

        self.active_transmissions += 1
        try:
            if to_id != 0xFF:
                # Unicast mode - specific target
                client_sock = self.clients.get(to_id)
                receiver_freq = self.node_frequency.get(to_id)
                if client_sock and sender_freq == receiver_freq:
                    targets = [(to_id, client_sock)]
                elif client_sock:
                    logger.warning(f"[DROP] FREQ_MISMATCH: Node {from_id} → Node {to_id} | {sender_freq} MHz ≠ {receiver_freq} MHz")
                    return
                else:
                    logger.warning(f"[DROP] INVALID_DESTINATION: Node {to_id} not registered or offline")
                    return
            else:
                # Broadcast mode with frequency check
                targets = [
                    (nid, sock)
                    for nid, sock in self.clients.items()
                    if nid != from_id and sock is not None and self.node_frequency.get(nid) == sender_freq
                ]
                
            for nid, client_sock in targets:
                to_loc = self.node_locations.get(nid, (0, 0))
                distance_km = math.dist(from_loc, to_loc)
                
                # Calculate signal parameters
                path_loss = self.compute_environmental_loss(from_id, nid, distance_km, aqi, weather, obstacle, sf)
                
                # Apply realistic RSSI limits - even with zero path loss, should never exceed -35dBm
                # This accounts for receiver front-end limitations and antenna inefficiencies
                max_realistic_rssi = -35  # dBm - realistic maximum for LoRa
                min_realistic_rssi = -150  # dBm - physical lower bound
                
                # Calculate RSSI with realistic bounds
                rssi = min(max_realistic_rssi, max(min_realistic_rssi, tx_dbm - path_loss)) + random.uniform(-1.5, 1.5)
                
                # Calculate SNR based on the realistic RSSI
                snr = self.compute_snr(rssi, sf, distance_km, weather, obstacle)
                
                # Calculate transmission delay
                delay_ms = self.calculate_transmission_delay(snr, sf, weather, distance_km, obstacle, payload_len)
                
                now = time.time()
                drop_reason = self.get_drop_reason(now, rssi, sf, nid, snr, min_snr, from_id, distance_km)
                
                if drop_reason:
                    logger.warning(f"[DROP] {drop_reason}: Packet from {from_id} to {nid} | "
                                f"RSSI: {rssi:.2f} dBm | SNR: {snr:.2f} dB | "
                                f"SF: {sf} | Distance: {distance_km:.2f} km | Delay: {delay_ms} ms")
                    continue

                # Mark receiver as busy for the duration of reception
                self.rx_busy_until[nid] = now + delay_ms / 1000.0
                
                # Add received signal parameters to message
                msg["rssi"] = round(rssi, 2)
                msg["snr"] = round(snr, 2)
                
                # Simulate transmission delay
                time.sleep(delay_ms/1000)
                
                # Deliver message to receiver
                try:
                    client_sock.sendall((json.dumps(msg) + '\n').encode())
                    logger.info(f"[✓] Delivered packet from {from_id} to {nid} | "
                            f"RSSI: {rssi:.2f} dBm | SNR: {snr:.2f} dB | "
                            f"SF: {sf} | Distance: {distance_km:.2f} km | Delay: {delay_ms:.2f} ms")
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
        logger.info(f"[~] Starting improved simulator server on {self.host}:{self.port} ...")
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