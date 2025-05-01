import socket
import threading
import json
import random
import time
import signal
import sys


class SimulatorServer:
    def __init__(self, host='0.0.0.0', port=5000, max_clients=5,
                 loss_rate=0.05, min_delay=0.01, max_delay=0.1,
                 rssi_range=(-110, -40)):
        """
        Initialize the LoRa RFM9x Simulator Server.
        """
        self.host = host
        self.port = port
        self.max_clients = max_clients
        self.loss_rate = loss_rate
        self.min_delay = min_delay
        self.max_delay = max_delay
        self.rssi_range = rssi_range

        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.server_socket.settimeout(1.0)

        self.clients = {}        # node_id -> socket
        self.lock = threading.Lock()
        self.stop_event = threading.Event()
        self.shutting_down = False

    def generate_rssi(self):
        return random.randint(*self.rssi_range)

    def simulate_delay(self):
        time.sleep(random.uniform(self.min_delay, self.max_delay))

    def should_drop(self):
        return random.random() < self.loss_rate

    def start(self):
        print(f"[~] Starting simulator server on {self.host}:{self.port} ...")
        self.server_socket.bind((self.host, self.port))
        self.server_socket.listen(self.max_clients)

        # Set up signal handler
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
        """
        Handle one client's registration and TX messages.
        """
        print(f"[+] Connected from {addr}")
        node_id = None
        conn.settimeout(0.5)

        try:
            while not self.stop_event.is_set():
                try:
                    raw = conn.recv(4096)
                except socket.timeout:
                    continue
                except (ConnectionResetError, OSError):
                    break

                if not raw:
                    break

                try:
                    msg = json.loads(raw.decode())
                except json.JSONDecodeError:
                    continue

                if msg["type"] == "register":
                    node_id = msg["node_id"]
                    with self.lock:
                        self.clients[node_id] = conn
                    print(f"[+] Node {node_id} registered")

                elif msg["type"] == "tx":
                    self.simulate_delay()

                    if self.should_drop():
                        print(f"[x] Dropped packet from node {msg['from']}")
                        continue

                    meta = msg.get("meta", {})
                    dest = meta.get("destination", 0xFF)

                    msg["rssi"] = self.generate_rssi()
                    msg["snr"] = round(random.uniform(4.0, 12.0), 1)

                    if dest == 0xFF:
                        # Broadcast to all nodes except sender
                        with self.lock:
                            for nid, client_sock in self.clients.items():
                                if nid != msg["from"]:
                                    try:
                                        client_sock.sendall(json.dumps(msg).encode())
                                    except:
                                        pass
                    else:
                        with self.lock:
                            target_sock = self.clients.get(dest)
                        if target_sock:
                            try:
                                target_sock.sendall(json.dumps(msg).encode())
                                print(f"[>] Node {msg['from']} → Node {dest}")
                            except Exception as e:
                                print(f"[!] Failed to send to node {dest}: {e}")
                        else:
                            print(f"[!] Destination node {dest} not found")

        finally:
            if node_id:
                with self.lock:
                    self.clients.pop(node_id, None)
            try:
                conn.shutdown(socket.SHUT_RDWR)
            except:
                pass
            conn.close()
            print(f"[-] Node {node_id} disconnected")

    def shutdown(self):
        """
        Gracefully shut down the server and all client connections.
        """
        if self.shutting_down:
            return
        self.shutting_down = True

        print("\n[!] Shutting down server...")
        self.stop_event.set()

        # Close all client sockets
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

        print("[✓] Server shutdown complete.")
        sys.exit(0)


if __name__ == "__main__":
    server = SimulatorServer()
    server.start()
