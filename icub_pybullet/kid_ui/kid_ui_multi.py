"""
Start multiple independent kid_ui.py instances (each with its own simulation,
web port, and video port) and print the addresses used to connect to them.
Stop with Ctrl+C.

Usage:
    python3 kid_ui_multi.py [-n 5] [--port 5001] [--stream-port 6001] [--world ball] [--log-dir DIR]

:Author: pyCub contributors
"""
import argparse
import os
import signal
import socket
import subprocess
import sys
import time

FILE_DIR = os.path.dirname(os.path.abspath(__file__))
# virtual/VPN interfaces that people cannot connect through
SKIP_PREFIXES = ("lo", "docker", "br-", "veth", "virbr", "lxc", "tailscale")


def local_addresses() -> list:
    """IPv4 addresses of the real network interfaces (e.g., LAN and the Wi-Fi hotspot)"""
    out = subprocess.run(["ip", "-4", "-o", "addr", "show", "up"], capture_output=True, text=True).stdout
    addresses = []
    for line in out.splitlines():
        parts = line.split()
        if not parts[1].startswith(SKIP_PREFIXES):
            addresses.append(parts[3].split("/")[0])
    return addresses


def port_free(port: int) -> bool:
    with socket.socket() as sock:
        return sock.connect_ex(("127.0.0.1", port)) != 0


def stop_all(procs: list) -> None:
    # signal the whole group, so the simulation processes of each instance are closed too
    for p in procs:
        try:
            os.killpg(p.pid, signal.SIGINT)
        except ProcessLookupError:
            pass
    deadline = time.time() + 10
    for p in procs:
        try:
            p.wait(max(0.1, deadline - time.time()))
        except subprocess.TimeoutExpired:
            pass
    for p in procs:
        try:
            os.killpg(p.pid, signal.SIGKILL)
        except ProcessLookupError:
            pass


def main() -> None:
    parser = argparse.ArgumentParser(description="Start multiple kid_ui.py instances")
    parser.add_argument("-n", "--count", type=int, default=5, help="number of instances (default: 5)")
    parser.add_argument("--port", type=int, default=5001, help="web port for the first instance")
    parser.add_argument("--stream-port", type=int, default=6001, help="video port for the first instance")
    parser.add_argument("--world", default="ball", help="world to load at startup")
    parser.add_argument("--log-dir", default=None, help="directory for instance logs (default: no logs)")
    args = parser.parse_args()

    if args.count < 1:
        parser.error("-n must be at least 1")
    web_ports = [args.port + i for i in range(args.count)]
    stream_ports = [args.stream_port + i for i in range(args.count)]
    if set(web_ports) & set(stream_ports):
        parser.error("web and video port ranges overlap")
    if max(web_ports + stream_ports) > 65535:
        parser.error("ports exceed 65535")
    busy = [p for p in web_ports + stream_ports if not port_free(p)]
    if busy:
        parser.error(f"ports are already in use: {busy}")
    if args.log_dir:
        os.makedirs(args.log_dir, exist_ok=True)

    procs = []
    try:
        for i, (web, stream) in enumerate(zip(web_ports, stream_ports), 1):
            log = open(os.path.join(args.log_dir, f"instance_{i}.log"), "w") if args.log_dir else subprocess.DEVNULL
            procs.append(subprocess.Popen(
                [sys.executable, os.path.join(FILE_DIR, "kid_ui.py"), "--host", "0.0.0.0", "--port", str(web),
                 "--stream-port", str(stream), "--world", args.world, "--no-browser"],
                stdout=log, stderr=log, start_new_session=True))

        addresses = local_addresses()
        for i, web in enumerate(web_ports, 1):
            print(f"Instance {i}: " + "  ".join(f"http://{ip}:{web}" for ip in addresses))
        print(f"Video ports: {stream_ports[0]}-{stream_ports[-1]} (allow these and web ports "
              f"{web_ports[0]}-{web_ports[-1]} through the firewall)")
        sys.stdout.flush()

        while any(p.poll() is None for p in procs):
            time.sleep(1)
    except KeyboardInterrupt:
        pass
    finally:
        stop_all(procs)


if __name__ == "__main__":
    main()
