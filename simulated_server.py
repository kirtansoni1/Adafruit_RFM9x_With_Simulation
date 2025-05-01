import socket
import threading
import json
import random
import time

# ---------------------------
# Simulation Parameters
# ---------------------------
MAX_CLIENTS = 5
LOSS_RATE = 0.05               # 5% packets lost
MIN_DELAY = 0.01               # seconds
MAX_DELAY = 0.1
RSSI_RANGE = (-110, -40)       # dBm range

# ---------------------------
# Global client registry
# ---------------------------
clients = {}  # node_id -> socket
lock = threading.Lock()


def generate_rssi():
    """Simulate RSSI value as an integer in dBm."""
    return random.randint(*RSSI_RANGE)


def simulate_delay():
    """Introduce a small real-world-like transmission delay."""
    time.sleep(random.uniform(MIN_DELAY, MAX_DELAY))


def should_drop():
    """Simulate packet loss."""
    return random.random() < LOSS_RATE


def handle_client(conn, addr):
    """
    Handle communication with a single simulated RFM9x node.
    """
    print(f"[+] Connected from {addr}")
    node_id = None

    try:
        while True:
            raw = conn.recv(4096)
            if not raw:
                break

            msg = json.loads(raw.decode())

            if msg["type"] == "register":
                node_id = msg["node_id"]
                with lock:
                    clients[node_id] = conn
                print(f"[+] Node {node_id} registered")

            elif msg["type"] == "tx":
                simulate_delay()

                if should_drop():
                    print(f"[x] Dropped packet from node {msg['from']}")
                    continue

                meta = msg.get("meta", {})
                dest = meta.get("destination", 0xFF)

                # Attach simulated RSSI and SNR
                msg["rssi"] = generate_rssi()
                msg["snr"] = round(random.uniform(4.0, 12.0), 1)

                # Broadcast
                if dest == 0xFF:
                    with lock:
                        for nid, client_sock in clients.items():
                            if nid != msg["from"]:
                                try:
                                    client_sock.sendall(json.dumps(msg).encode())
                                except Exception as e:
                                    print(f"[!] Broadcast to node {nid} failed: {e}")
                else:
                    with lock:
                        target_sock = clients.get(dest)
                    if target_sock:
                        try:
                            target_sock.sendall(json.dumps(msg).encode())
                            print(f"[>] Node {msg['from']} → Node {dest}")
                        except Exception as e:
                            print(f"[!] Failed to send to node {dest}: {e}")
                    else:
                        print(f"[!] Destination node {dest} not found")

    except (ConnectionResetError, json.JSONDecodeError):
        print(f"[!] Node {node_id} disconnected")

    finally:
        if node_id:
            with lock:
                if node_id in clients:
                    del clients[node_id]
        conn.close()
        print(f"[-] Connection from {addr} closed")


def main(host='0.0.0.0', port=5000):
    """
    Start the simulator server.
    """
    print(f"[~] Starting simulator server on {host}:{port} ...")
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.bind((host, port))
    server.listen(MAX_CLIENTS)

    try:
        while True:
            conn, addr = server.accept()
            threading.Thread(target=handle_client, args=(conn, addr), daemon=True).start()
    except KeyboardInterrupt:
        print("\n[!] Server shutdown requested")
    finally:
        server.close()


if __name__ == "__main__":
    main()
