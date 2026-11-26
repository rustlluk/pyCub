"""
Start multiple independent ``kid_ui.py`` instances (each with its own simulation,
web port, and video port) and print the addresses used to connect to them.
Stop with Ctrl+C.

Usage:
    python3 -m studio.kid_ui.kid_ui_multi [-n 5] [--port 5001] [--stream-port 6001] [--world ball] [--log-dir DIR]

:Author: pyCub contributors
"""
import argparse
import os
import signal
import socket
import subprocess
import sys
import time
from typing import List

FILE_DIR = os.path.dirname(os.path.abspath(__file__))
REPO_ROOT = os.path.dirname(os.path.dirname(FILE_DIR))
if REPO_ROOT not in sys.path:
    sys.path.insert(0, REPO_ROOT)

from icub_pybullet.runtime_paths import is_frozen, local_ipv4_addresses, terminate_process_tree


def port_free(port: int) -> bool:
    """Return True if a local TCP port is currently unbound."""
    with socket.socket() as sock:
        return sock.connect_ex(("127.0.0.1", port)) != 0


def popen_session_kwargs() -> dict:
    """Return platform-specific ``Popen`` kwargs that isolate child process groups."""
    if sys.platform == "win32":
        return {"creationflags": subprocess.CREATE_NEW_PROCESS_GROUP}
    return {"start_new_session": True}


def build_instance_cmd(web_port: int, stream_port: int, world: str, headless: bool = False) -> List[str]:
    """Build the command-line arguments to launch a single ``kid_ui.py`` instance."""
    if is_frozen():
        cmd = [sys.executable, "--worker-web-instance"]
    else:
        cmd = [sys.executable, os.path.join(FILE_DIR, "kid_ui.py")]
    cmd += [
        "--host", "0.0.0.0",
        "--port", str(web_port),
        "--stream-port", str(stream_port),
        "--world", world,
        "--no-browser",
    ]
    if headless:
        cmd.append("--headless")
    return cmd


def main(argv=None) -> None:
    """Parse CLI arguments, spawn N ``kid_ui.py`` processes, and monitor until interrupted."""
    parser = argparse.ArgumentParser(description="Start multiple kid_ui.py instances")
    parser.add_argument("-n", "--count", type=int, default=5, help="number of instances (default: 5)")
    parser.add_argument("--port", type=int, default=5001, help="web port for the first instance")
    parser.add_argument("--stream-port", type=int, default=6001, help="video port for the first instance")
    parser.add_argument("--world", default="ball", help="world to load at startup")
    parser.add_argument("--log-dir", default=None, help="directory for instance logs (default: no logs)")
    parser.add_argument("--headless", action="store_true", help="run without displaying the simulation (for testing)")
    args = parser.parse_args(argv)

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

    if hasattr(signal, "SIGTERM"):
        signal.signal(signal.SIGTERM, lambda *_: (_ for _ in ()).throw(KeyboardInterrupt()))
    procs = []
    log_handles = []
    try:
        for i, (web, stream) in enumerate(zip(web_ports, stream_ports), 1):
            if args.log_dir:
                log = open(os.path.join(args.log_dir, f"instance_{i}.log"), "w", encoding="utf-8")
                log_handles.append(log)
            else:
                log = subprocess.DEVNULL
            cmd = build_instance_cmd(web, stream, args.world, headless=args.headless)
            procs.append(subprocess.Popen(cmd, stdout=log, stderr=log, **popen_session_kwargs()))

        addresses = local_ipv4_addresses() or ["127.0.0.1"]
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
        terminate_process_tree(procs, timeout=8.0)
        for fh in log_handles:
            try:
                fh.close()
            except Exception:
                pass


if __name__ == "__main__":
    main()
