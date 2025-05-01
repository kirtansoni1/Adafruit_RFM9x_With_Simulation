import socket
import threading
import json
import random
import time
import signal
import sys

# ---------------------------
# Simulation Parameters
# ---------------------------
MAX_CLIENTS = 5
LOSS_RATE = 0.05               # 5% packets lost
MIN_DELAY = 0.01               # seconds
MAX_DELAY = 0.1
RSSI_RANGE = (-110, -40)       # dBm range

# ---------------------------
# Globals
# ---------------------------
clients = {}  # node_id -> socket
client_threads = []
lock = threading.Lock()
stop_event = threading.Event()


def generate_rssi():
    return random.randint(*RSSI_RANGE)


def simulate_delay():
    time.sleep(random.uniform(MIN_DELAY, MAX_DELAY))


def should_drop():
    return random.random() < LOSS_RATE


def handle_client(conn, addr):
    """
    Handle communication with a single simulated RFM9x node.
    """
    print(f"[+] Connected from {addr}")
    node_id = None
    conn.settimeout(0.5)

    try:
        while not stop_event.is_set():
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

                msg["rssi"] = generate_rssi()
                msg["snr"] = round(random.uniform(4.0, 12.0), 1)

                if dest == 0xFF:
                    with lock:
                        for nid, client_sock in clients.items():
                            if nid != msg["from"]:
                                try:
                                    client_sock.sendall(json.dumps(msg).encode())
                                except:
                                    pass
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
    finally:
        if node_id:
            with lock:
                clients.pop(node_id, None)
        try:
            conn.shutdown(socket.SHUT_RDWR)
        except:
            pass
        conn.close()
        print(f"[-] Node {node_id} connection closed")


def shutdown_server(server_socket):
    """
    Gracefully shut down server and all clients.
    """
    print("\n[!] Shutting down server...")
    stop_event.set()

    with lock:
        for node_id, client_sock in clients.items():
            try:
                client_sock.shutdown(socket.SHUT_RDWR)
                client_sock.close()
            except:
                pass
        clients.clear()

    try:
        server_socket.close()
    except:
        pass

    print("[✓] Server shutdown complete.")
    sys.exit(0)


def main(host='0.0.0.0', port=5000):
    print(f"[~] Starting simulator server on {host}:{port} ...")
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.bind((host, port))
    server.listen(MAX_CLIENTS)
    server.settimeout(1.0)  # <-- IMPORTANT: to allow Ctrl+C interrupt

    # Ctrl+C support
    def signal_handler(sig, frame):
        shutdown_server(server)

    signal.signal(signal.SIGINT, signal_handler)

    try:
        while not stop_event.is_set():
            try:
                conn, addr = server.accept()
                t = threading.Thread(target=handle_client, args=(conn, addr), daemon=True)
                t.start()
                client_threads.append(t)
            except socket.timeout:
                continue  # re-check stop_event
    finally:
        shutdown_server(server)


if __name__ == "__main__":
    main()
