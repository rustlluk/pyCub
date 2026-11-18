"""
Landing page for kid_ui that starts multiple kid_ui.py instances via kid_ui_multi.py,
shows all running instances (IP and port) with the number of connected users,
and lets users either pick an instance to connect to or click "Auto-connect"
to connect to the best available instance.

Usage:
    python3 kid_ui_landing.py [-n 5] [--host 0.0.0.0] [--port 5000]
                              [--instance-port 5001] [--stream-port 6001]
                              [--world ball] [--log-dir DIR]

:Author: pyCub contributors
"""
import argparse
import atexit
from concurrent.futures import ThreadPoolExecutor
import json
import os
import subprocess
import sys
import threading
import urllib.error
import urllib.request
import webbrowser
from typing import Dict, List, Optional

from flask import Flask, jsonify, redirect, request

FILE_DIR = os.path.dirname(os.path.abspath(__file__))
if FILE_DIR not in sys.path:
    sys.path.insert(0, FILE_DIR)

from kid_ui_multi import local_addresses, port_free, stop_all

STATE_PRIORITY = {
    "ready": 0,
    "busy": 1,
    "loading": 2,
    "stopped": 3,
    "closed": 4,
}


def fetch_instance_status(instance_id: int, port: int, multi_alive: bool) -> Dict:
    """Query a single kid_ui.py instance on localhost for its current status and user count."""
    url = f"http://127.0.0.1:{port}/api/status"
    try:
        with urllib.request.urlopen(url, timeout=0.5) as resp:
            data = json.loads(resp.read().decode("utf-8"))
        return {
            "id": instance_id,
            "port": port,
            "online": True,
            "state": data.get("state", "stopped"),
            "users": int(data.get("users", 0)),
            "world": data.get("world"),
        }
    except (urllib.error.URLError, TimeoutError, ValueError, OSError):
        return {
            "id": instance_id,
            "port": port,
            "online": False,
            "state": "starting" if multi_alive else "offline",
            "users": 0,
            "world": None,
        }


def choose_best_instance(instances: List[Dict]) -> Optional[Dict]:
    """Select the best online instance (fewest connected users, preferring ready over busy)."""
    active = [inst for inst in instances if inst.get("online") and inst.get("state") in ("ready", "busy")]
    candidates = active or [inst for inst in instances if inst.get("online")]
    if not candidates:
        return None
    return min(
        candidates,
        key=lambda inst: (
            inst.get("users", 0),
            STATE_PRIORITY.get(inst.get("state", ""), 99),
            inst.get("id", 0),
        ),
    )


PAGE = """<!doctype html>
<html lang="cs"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width, initial-scale=1">
<title>pyCub – Výběr robota</title>
<style>
  * { box-sizing: border-box; }
  body { font-family: "Comic Sans MS", "Trebuchet MS", sans-serif; background: #eef6ff; margin: 0; padding: 14px; }
  #container { max-width: 960px; margin: 0 auto; }
  h1 { color: #1b4f8a; margin: 0; font-size: 1.5em; }
  h2 { color: #1b4f8a; margin: 0 0 10px; font-size: 1.15em; }
  #heading { display: flex; justify-content: space-between; align-items: center; gap: 8px; margin-bottom: 12px; }
  #language { flex: 0 0 auto; padding: 6px 12px; font-size: 0.85em; }
  section { background: #fff; border-radius: 12px; padding: 14px 16px; margin: 0 0 14px;
            box-shadow: 0 2px 6px rgba(0,0,0,.15); }
  button { font: inherit; font-size: 0.95em; padding: 8px 14px; border: 0; border-radius: 10px; cursor: pointer;
           background: #4a9df8; color: #fff; box-shadow: 0 3px #2d6fb8; }
  button:active { transform: translateY(2px); box-shadow: 0 1px #2d6fb8; }
  button:disabled { background: #aaa; box-shadow: 0 3px #777; cursor: not-allowed; }
  #auto-box { display: flex; flex-wrap: wrap; align-items: center; justify-content: space-between; gap: 12px; }
  #auto-info { flex: 1 1 260px; color: #1b4f8a; }
  #auto-hint { font-size: 0.92em; color: #4a6785; margin-top: 4px; }
  #auto-btn { background: #34a853; box-shadow: 0 4px #1f7a37; font-size: 1.15em; font-weight: bold;
              padding: 12px 22px; flex: 0 0 auto; }
  #auto-btn:active { box-shadow: 0 1px #1f7a37; }
  #auto-btn:disabled { background: #aaa; box-shadow: 0 3px #777; }
  #instances { display: grid; grid-template-columns: repeat(auto-fill, minmax(270px, 1fr)); gap: 12px; }
  .card { border: 2px solid #d7e8fc; border-radius: 12px; padding: 12px 14px; background: #fafdff;
          display: flex; flex-direction: column; gap: 8px; transition: border-color .15s, box-shadow .15s; }
  .card.best { border-color: #34a853; background: #f3fbf5; }
  .card.offline { opacity: 0.7; background: #f5f7fa; border-color: #ddd; }
  .card-header { display: flex; justify-content: space-between; align-items: center; gap: 6px; }
  .card-title { font-weight: bold; font-size: 1.1em; color: #1b4f8a; }
  .badge { font-size: 0.78em; padding: 3px 8px; border-radius: 999px; font-weight: bold; }
  .badge.best { background: #34a853; color: #fff; }
  .addr { font-family: monospace; font-size: 0.98em; background: #eef6ff; color: #1b4f8a;
          padding: 5px 8px; border-radius: 8px; word-break: break-all; }
  .meta { display: flex; justify-content: space-between; align-items: center; font-size: 0.92em; color: #2c4c6e; }
  .users-pill { font-weight: bold; padding: 3px 9px; border-radius: 999px; background: #e3f2fd; color: #1b4f8a; }
  .users-pill.free { background: #e6f4ea; color: #1e7e34; }
  .users-pill.occupied { background: #fff3cd; color: #856404; }
  .connect-btn { width: 100%; margin-top: 4px; font-weight: bold; }
</style></head><body>
<div id="container">
  <div id="heading">
    <h1 data-i18n="heading"></h1>
    <button id="language" onclick="toggleLanguage()"></button>
  </div>

  <section id="auto-box">
    <div id="auto-info">
      <h2 data-i18n="autoTitle" style="margin-bottom:2px"></h2>
      <div id="auto-hint">…</div>
    </div>
    <button id="auto-btn" data-i18n="autoConnect" onclick="autoConnect()" disabled></button>
  </section>

  <section>
    <h2 data-i18n="instancesTitle"></h2>
    <div id="instances"></div>
  </section>
</div>
<script>
const TEXT = {
  cs: {
    pageTitle: "pyCub – Výběr robota",
    heading: "🤖 pyCub – Výběr robota",
    switchLanguage: "English",
    autoTitle: "🚀 Rychlé připojení",
    autoConnect: "🚀 Automaticky připojit",
    autoBest: "Doporučená instance:",
    autoWaiting: "⏳ Spouštím instance, chvilku strpení…",
    instancesTitle: "🖥️ Běžící instance",
    instanceName: "Instance",
    bestBadge: "⭐ Nejlepší volba",
    addressLabel: "IP a port:",
    usersLabel: "👥 Připojení uživatelé:",
    stateLabel: "Stav:",
    connectBtn: "🔗 Připojit",
    usersCount: n => n === 1 ? "1 uživatel" : (n >= 2 && n <= 4 ? n + " uživatelé" : n + " uživatelů"),
    states: {
      ready: "✅ Připraveno",
      busy: "🏃 Robot se hýbe",
      loading: "⏳ Načítání…",
      starting: "⏳ Spouštění…",
      stopped: "⏹️ Zastaveno",
      closed: "🪟 Ukončeno",
      offline: "🔌 Offline"
    }
  },
  en: {
    pageTitle: "pyCub – Choose a robot",
    heading: "🤖 pyCub – Choose a robot",
    switchLanguage: "Čeština",
    autoTitle: "🚀 Quick connect",
    autoConnect: "🚀 Auto-connect",
    autoBest: "Recommended instance:",
    autoWaiting: "⏳ Starting instances, please wait…",
    instancesTitle: "🖥️ Running instances",
    instanceName: "Instance",
    bestBadge: "⭐ Best choice",
    addressLabel: "IP & port:",
    usersLabel: "👥 Connected users:",
    stateLabel: "Status:",
    connectBtn: "🔗 Connect",
    usersCount: n => n === 1 ? "1 user" : n + " users",
    states: {
      ready: "✅ Ready",
      busy: "🏃 Busy",
      loading: "⏳ Loading…",
      starting: "⏳ Starting…",
      stopped: "⏹️ Stopped",
      closed: "🪟 Closed",
      offline: "🔌 Offline"
    }
  }
};

let language = "cs";
let latestData = null;

function applyLanguage() {
  document.documentElement.lang = language;
  document.title = TEXT[language].pageTitle;
  for (const el of document.querySelectorAll("[data-i18n]"))
    el.textContent = TEXT[language][el.dataset.i18n];
  document.getElementById("language").textContent = TEXT[language].switchLanguage;
  if (latestData) render(latestData);
}

function toggleLanguage() {
  language = language === "cs" ? "en" : "cs";
  applyLanguage();
}

function targetUrl(inst) {
  const host = (location.hostname && location.hostname !== "0.0.0.0") ? location.hostname : inst.ip;
  return location.protocol + "//" + host + ":" + inst.port;
}

function displayAddress(inst) {
  const addrs = (inst.addresses && inst.addresses.length) ? inst.addresses : [inst.ip];
  if (location.hostname && location.hostname !== "localhost" && location.hostname !== "127.0.0.1") {
    return location.hostname + ":" + inst.port;
  }
  return addrs.map(ip => ip + ":" + inst.port).join(" / ");
}

function connectTo(inst) {
  window.location.href = targetUrl(inst);
}

async function autoConnect() {
  try {
    const data = await (await fetch("/api/instances")).json();
    if (data.best) {
      connectTo(data.best);
      return;
    }
  } catch (e) {}
  if (latestData && latestData.best) {
    connectTo(latestData.best);
  }
}

function render(data) {
  const t = TEXT[language];
  const autoBtn = document.getElementById("auto-btn");
  const autoHint = document.getElementById("auto-hint");

  if (data.best) {
    autoBtn.disabled = false;
    autoHint.textContent = t.autoBest + " " + t.instanceName + " " + data.best.id +
      " (" + displayAddress(data.best) + " – " + t.usersCount(data.best.users) + ")";
  } else {
    autoBtn.disabled = true;
    autoHint.textContent = t.autoWaiting;
  }

  const box = document.getElementById("instances");
  box.innerHTML = "";
  for (const inst of data.instances) {
    const isBest = data.best && data.best.id === inst.id;
    const card = document.createElement("div");
    card.className = "card" + (isBest ? " best" : "") + (!inst.online ? " offline" : "");

    const header = document.createElement("div");
    header.className = "card-header";
    const title = document.createElement("span");
    title.className = "card-title";
    title.textContent = "🤖 " + t.instanceName + " " + inst.id;
    header.appendChild(title);
    if (isBest) {
      const badge = document.createElement("span");
      badge.className = "badge best";
      badge.textContent = t.bestBadge;
      header.appendChild(badge);
    }
    card.appendChild(header);

    const addr = document.createElement("div");
    addr.className = "addr";
    addr.textContent = displayAddress(inst);
    card.appendChild(addr);

    const usersRow = document.createElement("div");
    usersRow.className = "meta";
    const usersLabel = document.createElement("span");
    usersLabel.textContent = t.usersLabel;
    const usersPill = document.createElement("span");
    usersPill.className = "users-pill " + (inst.users === 0 ? "free" : "occupied");
    usersPill.textContent = String(inst.users);
    usersRow.appendChild(usersLabel);
    usersRow.appendChild(usersPill);
    card.appendChild(usersRow);

    const stateRow = document.createElement("div");
    stateRow.className = "meta";
    const stateLabel = document.createElement("span");
    stateLabel.textContent = t.stateLabel;
    const stateVal = document.createElement("span");
    stateVal.textContent = t.states[inst.state] || inst.state;
    stateRow.appendChild(stateLabel);
    stateRow.appendChild(stateVal);
    card.appendChild(stateRow);

    const btn = document.createElement("button");
    btn.className = "connect-btn";
    btn.textContent = t.connectBtn;
    btn.disabled = !inst.online;
    btn.onclick = () => connectTo(inst);
    card.appendChild(btn);

    box.appendChild(card);
  }
}

async function refresh() {
  try {
    latestData = await (await fetch("/api/instances")).json();
    render(latestData);
  } catch (e) {}
}

applyLanguage();
refresh();
setInterval(refresh, 1000);
</script></body></html>
"""


def create_landing_app(web_ports: List[int], multi_proc: Optional[subprocess.Popen] = None) -> Flask:
    app = Flask(__name__)
    executor = ThreadPoolExecutor(max_workers=max(1, len(web_ports)))

    def resolve_addresses() -> tuple:
        addresses = local_addresses()
        req_host = (request.host or "").split(":")[0]
        if req_host and req_host not in ("localhost", "127.0.0.1", "0.0.0.0", "::1", "[::1]"):
            primary_ip = req_host
        elif addresses:
            primary_ip = addresses[0]
        else:
            primary_ip = "127.0.0.1"
        return primary_ip, (addresses or [primary_ip])

    def query_all() -> dict:
        multi_alive = multi_proc is None or (multi_proc.poll() is None)
        primary_ip, all_addresses = resolve_addresses()
        futures = [
            executor.submit(fetch_instance_status, i, port, multi_alive)
            for i, port in enumerate(web_ports, 1)
        ]
        instances = []
        for fut in futures:
            info = fut.result()
            info["ip"] = primary_ip
            info["addresses"] = all_addresses
            info["url"] = f"http://{primary_ip}:{info['port']}"
            instances.append(info)
        best = choose_best_instance(instances)
        return {"instances": instances, "best": best}

    @app.get("/")
    def index():
        return PAGE

    @app.get("/api/instances")
    def instances():
        return jsonify(query_all())

    @app.get("/auto")
    def auto_redirect():
        data = query_all()
        best = data["best"]
        if not best:
            return redirect("/")
        req_host = (request.host or "").split(":")[0]
        target_host = req_host if req_host and req_host != "0.0.0.0" else best["ip"]
        return redirect(f"http://{target_host}:{best['port']}")

    return app


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Start kid_ui_multi.py and serve a landing page showing all running instances"
    )
    parser.add_argument("-n", "--count", type=int, default=5, help="number of instances (default: 5)")
    parser.add_argument("--host", default="0.0.0.0",
                        help="address for incoming connections (use 0.0.0.0 for Wi-Fi/LAN; unencrypted)")
    parser.add_argument("--port", type=int, default=5000, help="web port for the landing page (default: 5000)")
    parser.add_argument("--instance-port", type=int, default=5001,
                        help="web port for the first kid_ui.py instance (default: 5001)")
    parser.add_argument("--stream-port", type=int, default=6001,
                        help="video port for the first kid_ui.py instance (default: 6001)")
    parser.add_argument("--world", default="ball", help="world to load at startup")
    parser.add_argument("--log-dir", default=None, help="directory for instance logs (default: no logs)")
    parser.add_argument("--headless", action="store_true",
                        help="run instances without displaying the simulation (for testing)")
    parser.add_argument("--no-browser", action="store_true", help="do not open a browser automatically")
    args = parser.parse_args()

    if args.count < 1:
        parser.error("-n must be at least 1")
    if not 1 <= args.port <= 65535:
        parser.error("--port must be between 1 and 65535")

    web_ports = [args.instance_port + i for i in range(args.count)]
    stream_ports = [args.stream_port + i for i in range(args.count)]
    if set(web_ports) & set(stream_ports):
        parser.error("web and video port ranges overlap")
    if args.port in set(web_ports) | set(stream_ports):
        parser.error("--port overlaps with instance web or video ports")
    if max(web_ports + stream_ports) > 65535:
        parser.error("ports exceed 65535")
    busy = [p for p in [args.port] + web_ports + stream_ports if not port_free(p)]
    if busy:
        parser.error(f"ports are already in use: {busy}")

    multi_cmd = [
        sys.executable, os.path.join(FILE_DIR, "kid_ui_multi.py"),
        "-n", str(args.count),
        "--port", str(args.instance_port),
        "--stream-port", str(args.stream_port),
        "--world", args.world,
    ]
    if args.log_dir:
        multi_cmd += ["--log-dir", args.log_dir]
    if args.headless:
        multi_cmd.append("--headless")

    multi_proc = subprocess.Popen(multi_cmd, start_new_session=True)
    atexit.register(lambda: stop_all([multi_proc]))

    addresses = local_addresses() or ["127.0.0.1"]
    print("Landing page: " + "  ".join(f"http://{ip}:{args.port}" for ip in addresses))
    sys.stdout.flush()

    app = create_landing_app(web_ports, multi_proc)
    if not args.no_browser:
        threading.Timer(1.0, lambda: webbrowser.open(f"http://localhost:{args.port}")).start()
    try:
        app.run(host=args.host, port=args.port, threaded=True)
    finally:
        stop_all([multi_proc])


if __name__ == "__main__":
    main()
