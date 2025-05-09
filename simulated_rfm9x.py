'''
Simulated RFM9x LoRa Client
===========================

This module provides a software simulation of the Adafruit RFM9x LoRa radio module,
allowing users to run CircuitPython-style radio communication code without needing
physical LoRa hardware.

Each simulated node connects to a central simulation server via TCP and mimics the
`adafruit_rfm9x.RFM9x` API, supporting basic send/receive operations along with
reliable datagram behavior (ACK/Retry).

Usage Example:
--------------
This client is typically used in conjunction with `./simulated_server.py`.

Run in simulation mode:
    python ./example/simpletest.py --simulate --id=1
'''

import socket
import json
import time
import random


class SimulatedRFM9x:

    def __init__(self, node_id=1, server_ip='localhost', server_port=5000, location=(0, 0), frequency = 915.0):
        """
        Initialize the simulated RFM9x module.

        Parameters:
        - node_id (int): Unique identifier for this simulated node.
        - server_ip (str): IP address of the simulation server.
        - server_port (int): Port of the simulation server.
        - location (tuple): (x, y) location in km
        - frequency (int): Frequency at which the node is sending data.
        """
        self.node_id = node_id
        self.location = location
        self.server = (server_ip, server_port)

        # Create and connect TCP socket to simulation server
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.connect(self.server)
        self.sock.settimeout(0.1)

        # Simulated configuration & telemetry
        self.tx_power = 23  # in dBm
        self.last_rssi = -42  # updated after each receive
        self.last_snr = 0.0
        self.enable_crc = True
        self.frequency = frequency

        # Packet metadata
        self.sequence_number = 0
        self.flags = 0
        self.node = self.node_id  # source address
        self.destination = None   # default is broadcast
        self.identifier = 0

        # Timing
        self.receive_timeout = 0.5
        self.ack_wait = 0.5
        self.ack_delay = None
        self.ack_retries = 5

        self._keep_listening = False  # internal state tracking
        self._register()

    def _register(self):
        """Send a registration message to the server to announce this node."""
        msg = {
            "type": "register",
            "node_id": self.node_id,
            "location": self.location,
            "frequency": self.frequency
        }
        self.sock.sendall((json.dumps(msg) + '\n').encode())

    def send(self, data: bytes, *, keep_listening=False, destination=None,
             node=None, identifier=None, flags=None):
        """
        Send a packet of data to the simulation server.

        Parameters:
        - data (bytes): The payload to transmit.
        - keep_listening (bool): If True, receive mode is re-enabled immediately after sending.
        - destination (int): Destination address (default: 0xFF broadcast).
        - node (int): Source address override (default: self.node).
        - identifier (int): Sequence number for reliable datagrams.
        - flags (int): Bit flags (ACK, retry, etc.).
        """
        if isinstance(data, bytes):
            data = data.decode()

        # Build metadata for the RadioHead-style header
        header = {
            "destination": destination if destination is not None else self.destination,
            "node": node if node is not None else self.node,
            "identifier": identifier if identifier is not None else self.identifier,
            "flags": flags if flags is not None else self.flags
        }

        msg = {
            "type": "tx",
            "from": self.node_id,
            "data": data,
            "meta": {
                **header,
                "tx_power": self.tx_power,
                "timestamp": time.time()
            }
        }

        self.sock.sendall((json.dumps(msg) + '\n').encode())
        self._keep_listening = keep_listening  # mock internal receive state

    def receive(self, *, keep_listening=True, with_header=False, with_ack=False, timeout=None):
        """
        Receive a packet from the server. If no packet is received in time, returns None.

        Parameters:
        - keep_listening (bool): Keep listen mode active after receiving.
        - with_header (bool): Include the 4-byte RadioHead header in the returned data.
        - with_ack (bool): Automatically send an ACK if requested.
        - timeout (float): Optional timeout override.

        Returns:
        - bytearray or None
        """
        timeout = timeout if timeout is not None else self.receive_timeout
        self.sock.settimeout(timeout)

        try:
            raw = self.sock.recv(4096)
            msg = json.loads(raw.decode())

            # Update telemetry
            self.last_rssi = msg.get("rssi", -90)
            self.last_snr = msg.get("snr", 0.0)

            payload = msg.get("data", "")
            header = msg.get("meta", {})

            # Respond with ACK if requested and not already an ACK
            if with_ack and not (header.get("flags", 0) & 0x80) and header.get("destination") != 0xFF:
                self._send_ack(
                    to_node=header["node"],
                    identifier=header["identifier"],
                    original_flags=header["flags"]
                )

            self._keep_listening = keep_listening  # mock internal receive state

            # Return payload with or without header
            if with_header:
                return bytearray([
                    header.get("destination", 0xFF),
                    header.get("node", 0xFF),
                    header.get("identifier", 0),
                    header.get("flags", 0)
                ]) + bytearray(payload, 'utf-8')

            return bytearray(payload, 'utf-8')

        except socket.timeout:
            return None
        except Exception as e:
            print(f"[SimulatedRFM9x] Error receiving: {e}")
            return None

    def _send_ack(self, to_node, identifier, original_flags):
        """
        Send an ACK message to the sender node.

        Parameters:
        - to_node (int): Destination node to acknowledge.
        - identifier (int): Packet ID to acknowledge.
        - original_flags (int): Flags from original message.
        """
        ack_msg = {
            "type": "tx",
            "from": self.node_id,
            "data": "!",
            "meta": {
                "destination": to_node,
                "node": self.node_id,
                "identifier": identifier,
                "flags": original_flags | 0x80,
                "tx_power": self.tx_power,
                "timestamp": time.time()
            }
        }

        if self.ack_delay:
            time.sleep(self.ack_delay)

        self.sock.sendall((json.dumps(ack_msg) + '\n').encode())

    def send_with_ack(self, data: bytes) -> bool:
        """
        Send a packet and wait for an ACK.

        Parameters:
        - data (bytes): The data to send.

        Returns:
        - bool: True if ACK received, False otherwise.
        """
        self.sequence_number = (self.sequence_number + 1) & 0xFF
        self.identifier = self.sequence_number
        retries = self.ack_retries

        while retries > 0:
            self.send(data, keep_listening=True, identifier=self.identifier, flags=self.flags)

            if self.destination == 0xFF:
                return True

            ack = self.receive(timeout=self.ack_wait, with_header=True)

            if ack and ack[3] & 0x80 and ack[2] == self.identifier:
                return True

            retries -= 1
            time.sleep(self.ack_wait + random.uniform(0, 0.1))

        return False
