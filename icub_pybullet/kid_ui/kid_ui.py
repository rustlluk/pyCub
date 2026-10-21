"""
A simple web control panel for pyCub that is easy for children to use.

Starts a small web server (default address http://localhost:5000) with large buttons
such as "Raise the left arm" and "Restart world". The simulation runs in its own
process and is displayed directly on the page (Open3D WebRTC), so loading or
restarting a world simply starts a new simulation.

Usage:
    python3 kid_ui.py [--host 0.0.0.0] [--port 5000] [--stream-port 6000]
                      [--world ball] [--separate-window] [--headless]

:Author: pyCub contributors
"""
import argparse
import atexit
import multiprocessing as mp
import os
import queue
import socket
import sys
import tempfile
import threading
import time
import webbrowser
from typing import Callable, Dict, Generator, Optional

import yaml
from flask import Flask, jsonify, request

FILE_DIR = os.path.dirname(os.path.abspath(__file__))

# id -> (label, config file in the "configs" directory)
WORLDS = {
    "empty": (("Prázdný pokoj", "Empty room"), "default.yaml"),
    "ball": (("Stůl s míčkem", "Table with a ball"), "with_ball.yaml"),
}


# ---------------------------------------------------------------------------------------------------------------------
# Actions. Each one is a generator that yields while the robot is moving, so the simulation keeps running in between.
# ---------------------------------------------------------------------------------------------------------------------
def go(client, targets: Dict[str, float], velocity: float = 1.0) -> Generator:
    """Move joints to the target positions (radians) and yield until done (or timed out)"""
    names = list(targets)
    client.move_position(names, [targets[n] for n in names], wait=False, velocity=velocity, timeout=4,
                         check_collision=False)
    # Contact checking is off, otherwise a robot touching the table would refuse all further motions
    while not client.motion_done(names, check_collision=False):
        yield


def jog_wait(client, joints: set) -> Generator:
    """Yield until all joints moved by step buttons reach their targets"""
    while not client.motion_done(list(joints), check_collision=False):
        yield
    joints.clear()


def arm(side: str, **joints: float) -> Dict[str, float]:
    return {f"{side}_{name}": value for name, value in joints.items()}


def merge(*dicts: Dict[str, float]) -> Dict[str, float]:
    out = {}
    for d in dicts:
        out.update(d)
    return out


ARM_UP = dict(shoulder_pitch=-1.6, shoulder_roll=0.3, shoulder_yaw=0.0, elbow=0.6)
ARM_DOWN = dict(shoulder_pitch=-0.6, shoulder_roll=0.5, shoulder_yaw=0.0, elbow=0.9)


def make_arm_actions(side: str) -> Dict[str, Callable]:
    def wave(client):
        yield from go(client, arm(side, shoulder_pitch=-1.2, shoulder_roll=1.2, shoulder_yaw=0.0, elbow=1.2))
        for _ in range(3):
            yield from go(client, arm(side, elbow=0.6), velocity=4)
            yield from go(client, arm(side, elbow=1.6), velocity=4)
        yield from go(client, arm(side, **ARM_DOWN))

    return {f"{side}_arm_wave": wave}


def both_arms_up(client):
    yield from go(client, merge(arm("l", **ARM_UP), arm("r", **ARM_UP)))


def both_arms_down(client):
    yield from go(client, merge(arm("l", **ARM_DOWN), arm("r", **ARM_DOWN)))


def look(**joints: float) -> Callable:
    def action(client):
        yield from go(client, joints)
    return action


def reset_pose(client):
    """Return all joints to the angles defined in the config"""
    import numpy as np
    initial = client.config.initial_joint_angles
    targets = {j.name: float(np.deg2rad(getattr(initial, j.name))) if hasattr(initial, j.name) else 0.0
               for j in client.joints if "_hand_" not in j.name and "eye" not in j.name}
    yield from go(client, targets)


ACTIONS: Dict[str, Callable] = {
    **make_arm_actions("l"), **make_arm_actions("r"),
    "both_arms_up": both_arms_up, "both_arms_down": both_arms_down,
    "look_center": look(neck_yaw=0.0, neck_pitch=0.0, neck_roll=0.0),
    "stand_straight": look(torso_yaw=0.0, torso_pitch=0.0, torso_roll=0.0),
    "reset_pose": reset_pose,
}

# Step buttons: id -> (joint, sign). Pressing "+" moves the joint by +sign*step degrees, "-" by -sign*step degrees.
JOGS = {}


def add_jog(jog_id: str, joint: str, sign: int) -> str:
    JOGS[jog_id] = (joint, sign)
    return jog_id


def arm_rows(side: str):
    # (label, [(jog id, button label), ...]); the first button moves the joint in the "positive" direction
    return [
        (("Rameno nahoru / dolů", "Shoulder up / down"),
         add_jog(f"{side}_shoulder_pitch", f"{side}_shoulder_pitch", -1),
         ("⬆️ Nahoru", "⬆️ Up"), ("⬇️ Dolů", "⬇️ Down")),
        (("Rameno do strany / k tělu", "Shoulder out / in"),
         add_jog(f"{side}_shoulder_roll", f"{side}_shoulder_roll", 1),
         ("↔️ Do strany", "↔️ Out"), ("↩️ K tělu", "↩️ In")),
        (("Otočit paži", "Rotate arm"),
         add_jog(f"{side}_shoulder_yaw", f"{side}_shoulder_yaw", 1),
         ("↻ Doprava", "↻ Right"), ("↺ Doleva", "↺ Left")),
        (("Loket", "Elbow"), add_jog(f"{side}_elbow", f"{side}_elbow", 1),
         ("💪 Ohnout", "💪 Bend"), ("📏 Natáhnout", "📏 Straighten")),
        (("Zápěstí", "Wrist"), add_jog(f"{side}_wrist_prosup", f"{side}_wrist_prosup", 1),
         ("↻ Doprava", "↻ Right"), ("↺ Doleva", "↺ Left")),
    ]


def head_rows():
    return [
        (("Hlava vlevo / vpravo", "Head left / right"), add_jog("neck_yaw", "neck_yaw", 1),
         ("👈 Doleva", "👈 Left"), ("👉 Doprava", "👉 Right")),
        (("Hlava nahoru / dolů", "Head up / down"), add_jog("neck_pitch", "neck_pitch", -1),
         ("👆 Nahoru", "👆 Up"), ("👇 Dolů", "👇 Down")),
        (("Naklonit hlavu", "Tilt head"), add_jog("neck_roll", "neck_roll", 1),
         ("↪️ Doleva", "↪️ Left"), ("↩️ Doprava", "↩️ Right")),
    ]


def body_rows():
    return [
        (("Otáčení těla", "Turn body"), add_jog("torso_yaw", "torso_yaw", 1),
         ("↪️ Doleva", "↪️ Left"), ("↩️ Doprava", "↩️ Right")),
        (("Předklon", "Bend forward"), add_jog("torso_pitch", "torso_pitch", 1),
         ("🙇 Dopředu", "🙇 Forward"), ("🧍 Dozadu", "🧍 Back")),
        (("Úklon do strany", "Bend sideways"), add_jog("torso_roll", "torso_roll", 1),
         ("↪️ Doleva", "↪️ Left"), ("↩️ Doprava", "↩️ Right")),
    ]


# Layout of the page. "rows" are step buttons (label, jog id, + label, - label); "buttons" are ready-made actions
SECTIONS = [
    {"title": ("Levá ruka", "Left arm"), "rows": arm_rows("l"),
     "buttons": [("l_arm_wave", ("👋 Zamávat", "👋 Wave"))]},
    {"title": ("Pravá ruka", "Right arm"), "rows": arm_rows("r"),
     "buttons": [("r_arm_wave", ("👋 Zamávat", "👋 Wave"))]},
    {"title": ("Obě ruce", "Both arms"), "rows": [],
     "buttons": [("both_arms_up", ("🙌 Ruce nahoru", "🙌 Arms up")),
                 ("both_arms_down", ("🙇 Ruce dolů", "🙇 Arms down"))]},
    {"title": ("Hlava", "Head"), "rows": head_rows(),
     "buttons": [("look_center", ("🎯 Dívat se rovně", "🎯 Look straight ahead"))]},
    {"title": ("Tělo", "Body"), "rows": body_rows(),
     "buttons": [("stand_straight", ("🧍 Narovnat se", "🧍 Stand straight"))]},
]


# ---------------------------------------------------------------------------------------------------------------------
# Simulation process
# ---------------------------------------------------------------------------------------------------------------------
def make_config(world: str, web_gui: bool, headless: bool) -> str:
    """Copy the world's config into a temp file with the GUI settings required by this UI"""
    with open(os.path.join(FILE_DIR, "configs", WORLDS[world][1]), "r") as f:
        cfg = yaml.safe_load(f)
    cfg["gui"] = {"standard": not (web_gui or headless), "web": web_gui and not headless}
    cfg["debug"] = False
    fd, path = tempfile.mkstemp(suffix=".yaml", prefix="pycub_kid_")
    with os.fdopen(fd, "w") as f:
        yaml.safe_dump(cfg, f)
    return path


def free_port() -> int:
    with socket.socket() as sock:
        sock.bind(("127.0.0.1", 0))
        return sock.getsockname()[1]


def sim_process(world: str, web_gui: bool, headless: bool, commands: "mp.Queue", status: "mp.Queue",
                stream_port: int, stream_host: str, ball_info=None) -> None:
    """Runs the simulation, executing actions sent through the commands queue"""
    # every run streams on its own port, so a previous (still closing) simulation or another instance cannot collide
    os.environ["WEBRTC_PORT"] = str(stream_port)
    os.environ["WEBRTC_IP"] = stream_host
    sys.path.insert(0, os.path.dirname(FILE_DIR))
    from icub_pybullet.pycub import pyCub

    if web_gui:
        # the simulation is shown inside our page, so do not open a separate browser tab
        webbrowser.open = lambda *a, **k: False
    cfg_path = make_config(world, web_gui, headless)
    try:
        status.put({"state": "loading"})
        client = pyCub(config=cfg_path)
    finally:
        os.remove(cfg_path)
    status.put({"state": "ready"})

    import numpy as np
    current: Optional[Generator] = None
    current_name = None
    jogging = set()
    ball_id = next((o[0] for o in client.other_objects if o[1] == "ball"), None)
    table_id = next((o[0] for o in client.other_objects if o[1] == "table"), None)
    dt = client.getPhysicsEngineParameters()["fixedTimeStep"]
    last_contact_step = None
    after_5s = float("nan")
    moving_time = float("nan")
    while client.is_alive():
        try:
            while True:
                name = commands.get_nowait()
                if name == "stop":
                    client.stop_robot()
                    current, current_name = None, None
                elif isinstance(name, tuple):
                    # step command: ("jog", joint, delta in degrees, speed in rad/s); repeated presses add up
                    _, joint, delta, speed = name
                    if current_name != "jog":
                        client.stop_robot()
                        jogging.clear()
                    robot_id, joint_id = client.find_joint_id(joint)
                    state = client.joints[joint_id]
                    base = state.set_point if isinstance(state.set_point, float) else client.get_joint_state(joint)[0]
                    target = float(np.clip(base + np.deg2rad(delta), state.lower_limit, state.upper_limit))
                    client.move_position(joint, target, wait=False, velocity=speed, timeout=8, check_collision=False)
                    jogging.add(joint)
                    if current_name != "jog":
                        current_name = "jog"
                        current = jog_wait(client, jogging)
                        status.put({"state": "busy", "action": "jog"})
                elif name in ACTIONS:
                    if current is not None:
                        client.stop_robot()
                    current, current_name = ACTIONS[name](client), name
                    status.put({"state": "busy", "action": name})
        except queue.Empty:
            pass

        if current is not None:
            try:
                next(current)
            except StopIteration:
                current, current_name = None, None
                status.put({"state": "ready"})
        client.update_simulation()
        if ball_info is not None:
            dist = float("nan")
            if ball_id is not None and table_id is not None:
                pts = client.getClosestPoints(ball_id, table_id, 10.0)
                dist = min(p[8] for p in pts) if pts else float("nan")
                steps = client.steps_done
                if client.getContactPoints(client.robot, ball_id):
                    last_contact_step, after_5s, moving_time = steps, float("nan"), 0.0
                elif last_contact_step is not None and np.isnan(after_5s) and (steps - last_contact_step) * dt >= 3:
                    after_5s = dist
                if last_contact_step is not None and not client.getContactPoints(client.robot, ball_id):
                    # the clock runs from the last touch until the ball comes to rest
                    lin, ang = client.getBaseVelocity(ball_id)
                    if np.linalg.norm(lin) > 0.01 or np.linalg.norm(ang) > 0.1:
                        moving_time = (steps - last_contact_step) * dt
            ball_info[0], ball_info[1] = float(ball_id is not None), dist
            ball_info[2] = after_5s
            ball_info[4] = moving_time
            ball_info[3] = float(last_contact_step is not None)
        if headless and client.default_step is None:
            time.sleep(0.002)
    status.put({"state": "closed"})


class Simulation:
    """Supervisor that (re)starts the simulation process and relays commands to it"""

    def __init__(self, web_gui: bool, headless: bool, stream_host: str, stream_port: Optional[int] = None):
        self.web_gui = web_gui
        self.headless = headless
        self.stream_host = stream_host
        self.configured_stream_port = stream_port
        self.ctx = mp.get_context("spawn")
        self.process = None
        self.world = None
        self.state = {"state": "stopped"}
        self.lock = threading.Lock()
        self.commands = self.status = None
        self.ball_info = self.ctx.Array("d", [0.0, float("nan"), float("nan"), 0.0, float("nan")])
        self.stream_port = None
        self.run = 0  # increases with every started simulation; the page uses it to reconnect the video

    def start(self, world: str) -> None:
        with self.lock:
            self._stop()
            self.world = world
            self.run += 1
            self.stream_port = self.configured_stream_port or free_port()
            self.commands, self.status = self.ctx.Queue(), self.ctx.Queue()
            self.state = {"state": "loading"}
            self.ball_info[:] = [0.0, float("nan"), float("nan"), 0.0, float("nan")]
            self.process = self.ctx.Process(target=sim_process, daemon=True,
                                            args=(world, self.web_gui, self.headless, self.commands, self.status,
                                                  self.stream_port, self.stream_host, self.ball_info))
            self.process.start()

    def _stop(self) -> None:
        if self.process is not None and self.process.is_alive():
            self.process.terminate()
            self.process.join(5)
            if self.process.is_alive():
                self.process.kill()
                self.process.join(5)
        self.process = None

    def stop(self) -> None:
        with self.lock:
            self._stop()
            self.state = {"state": "stopped"}

    def send(self, command: str) -> bool:
        with self.lock:
            if self.process is None or not self.process.is_alive() or self.state["state"] == "loading":
                return False
            self.commands.put(command)
            return True

    def get_state(self) -> dict:
        with self.lock:
            if self.status is not None:
                try:
                    while True:
                        self.state = self.status.get_nowait()
                except queue.Empty:
                    pass
            if self.process is not None and not self.process.is_alive() and self.state["state"] != "closed":
                self.state = {"state": "closed"}
            present, dist, after, touched, moving = self.ball_info[:]
            ball = None
            if present:
                nn = lambda v: None if v != v else round(v * 100, 1)  # cm
                ball = {"distance": nn(dist), "after": nn(after), "touched": bool(touched),
                        "moving": None if moving != moving else round(moving, 2)}
            return dict(self.state, world=self.world, run=self.run, stream_port=self.stream_port, ball=ball)


# ---------------------------------------------------------------------------------------------------------------------
# Web server
# ---------------------------------------------------------------------------------------------------------------------
PAGE = """<!doctype html>
<html lang="cs"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width, initial-scale=1">
<title>pyCub – Ovládání robota</title>
<style>
  * { box-sizing: border-box; }
  body { font-family: "Comic Sans MS", "Trebuchet MS", sans-serif; background: #eef6ff; margin: 0; padding: 10px; }
  h1 { color: #1b4f8a; margin: 0; font-size: 1.4em; }
  #heading { display: flex; justify-content: space-between; align-items: center; gap: 8px; }
  #language { flex: 0 0 auto; padding: 6px 10px; font-size: 0.85em; }
  #status { font-size: 1em; margin: 2px 0 6px; min-height: 1.3em; }
  #layout { display: flex; gap: 12px; align-items: flex-start; }
  #left { flex: 0 0 46%; position: sticky; top: 10px; max-height: calc(100vh - 20px); overflow: auto; }
  #right { flex: 1 1 0; min-width: 0; }
  #sections { columns: 290px; column-gap: 10px; }
  section { background: #fff; border-radius: 12px; padding: 8px 10px; margin: 0 0 10px;
            box-shadow: 0 2px 6px rgba(0,0,0,.15); }
  #sections section { break-inside: avoid; }
  h2 { margin: 0 0 6px; color: #1b4f8a; font-size: 1.05em; }
  .row { display: flex; flex-wrap: wrap; gap: 6px; }
  button { font: inherit; font-size: 0.95em; padding: 8px 10px; border: 0; border-radius: 10px; cursor: pointer;
           background: #4a9df8; color: #fff; flex: 1 1 110px; box-shadow: 0 3px #2d6fb8; }
  button:active { transform: translateY(2px); box-shadow: 0 1px #2d6fb8; }
  button:disabled { background: #aaa; box-shadow: 0 3px #777; cursor: not-allowed; }
  button.world { background: #34a853; box-shadow: 0 3px #1f7a37; }
  button.world.active { outline: 3px solid #ffd54f; }
  button.restart { background: #f9a825; box-shadow: 0 3px #b27700; }
  button.stop { background: #e53935; box-shadow: 0 3px #9c1f1c; }
  .step { display: flex; align-items: center; gap: 6px; flex-wrap: wrap; }
  .step button { flex: 0 0 auto; padding: 6px 12px; background: #8e6cd8; box-shadow: 0 3px #5a3fa0; }
  .step button.on { outline: 3px solid #ffd54f; }
  .step input { font: inherit; width: 4em; padding: 5px; border-radius: 8px; border: 2px solid #8e6cd8; }
  .jog { display: flex; align-items: center; gap: 6px; margin: 4px 0; }
  .jog .name { flex: 0 0 105px; color: #1b4f8a; font-weight: bold; font-size: 0.85em; line-height: 1.1; }
  .jog button { flex: 1 1 0; padding: 7px 4px; }
  #right .row { margin-top: 6px; }
  #view video { display: block; width: 100%; max-height: calc(100vh - 290px); object-fit: contain; background: #fff;
                border-radius: 10px;
                touch-action: none; }
  @media (max-width: 900px) { #layout { flex-direction: column; } #left { position: static; flex-basis: auto;
                                                                          width: 100%; max-height: none; } }
</style></head><body>
<div id="layout">
<div id="left">
  <section id="view" style="display:none"><video id="sim" autoplay muted playsinline></video></section>
  <section><div class="row" id="worlds"></div>
    <div class="row">
      <button class="restart" data-i18n="restart" onclick="post('/api/restart')"></button>
      <button class="stop" data-i18n="stop" onclick="post('/api/action/stop')"></button>
      <button class="restart" data-i18n="resetPose" onclick="post('/api/action/reset_pose')"></button>
    </div></section>
</div>
<div id="right">
  <div id="heading"><h1 data-i18n="heading"></h1><button id="language" onclick="toggleLanguage()"></button></div>
  <div id="status">…</div>
  <div id="ball" style="display:none; background:#fff; border-radius:12px; padding:6px 10px; margin-bottom:8px"></div>
  <div id="sections"></div>
  <section id="settings"><div class="step" id="steps"><b data-i18n="step"></b></div>
    <div class="step" id="speeds" style="margin-top:6px"><b data-i18n="speed"></b></div></section></div>
</div>
<script>
const CFG = __CONFIG__;
const TEXT = {
  cs: {
    pageTitle: "pyCub – Ovládání robota", heading: "🤖 Ovládání robota",
    restart: "🔄 Restartovat svět", stop: "✋ Zastavit", resetPose: "🧍 Základní postoj",
    step: "Krok:", speed: "Rychlost:", switchLanguage: "English",
    loading: "⏳ Načítám svět, chvilku strpení…", ready: "✅ Připraveno! Zmáčkni tlačítko.",
    busy: "🏃 Robot se hýbe…", closed: "🪟 Simulace skončila. Zmáčkni „Restartovat svět“.",
    stopped: "Vyber svět, ve kterém to začne.",
    slow: "🐢 Pomalu", medium: "🚶 Středně", fast: "🐇 Rychle",
    ballDistance: "⚽ Vzdálenost míčku od stolu:",
    ballAfter: "⏱️ 3 s po posledním dotyku robota:",
    ballElapsed: "🏃 Čas od posledního dotyku (míček se hýbe):",
    waiting: "čekám…", untouched: "míček se ještě nedotkl robota"
  },
  en: {
    pageTitle: "pyCub – Robot controls", heading: "🤖 Robot controls",
    restart: "🔄 Restart world", stop: "✋ Stop", resetPose: "🧍 Reset posture",
    step: "Step:", speed: "Speed:", switchLanguage: "Čeština",
    loading: "⏳ Loading the world, please wait…", ready: "✅ Ready! Press a button.",
    busy: "🏃 The robot is moving…", closed: "🪟 The simulation has ended. Press “Restart world”.",
    stopped: "Choose a world to get started.",
    slow: "🐢 Slow", medium: "🚶 Medium", fast: "🐇 Fast",
    ballDistance: "⚽ Ball distance from the table:",
    ballAfter: "⏱️ 3 s after the robot's last touch:",
    ballElapsed: "🏃 Time since the last touch (while the ball is moving):",
    waiting: "waiting…", untouched: "the ball has not touched the robot yet"
  }
};
let language = "cs";
function setPair(element, pair, prefix = "") {
  element.dataset.cs = pair[0];
  element.dataset.en = pair[1];
  element.dataset.prefix = prefix;
  element.textContent = prefix + pair[language === "en" ? 1 : 0];
}
function applyLanguage() {
  document.documentElement.lang = language;
  document.title = TEXT[language].pageTitle;
  for (const element of document.querySelectorAll("[data-i18n]"))
    element.textContent = TEXT[language][element.dataset.i18n];
  for (const element of document.querySelectorAll("[data-cs][data-en]"))
    element.textContent = element.dataset.prefix + element.dataset[language];
  document.getElementById("language").textContent = TEXT[language].switchLanguage;
  refresh();
}
function toggleLanguage() {
  language = language === "cs" ? "en" : "cs";
  applyLanguage();
}
async function post(url, body) {
  await fetch(url, {method: "POST", headers: {"Content-Type": "application/json"}, body: JSON.stringify(body || {})});
  refresh();
}
function stepDegrees() {
  const v = parseFloat(document.getElementById("stepInput").value);
  return (v > 0 && v <= 90) ? v : 10;
}
const SPEEDS = [["slow", 0.6], ["medium", 1.5], ["fast", 4]];
let speed = 1.5;
function buildSpeeds() {
  const box = document.getElementById("speeds");
  for (const [key, v] of SPEEDS) {
    const b = document.createElement("button");
    b.textContent = TEXT[language][key]; b.dataset.i18n = key; b.dataset.speed = v;
    b.onclick = () => { speed = v; mark(); };
    box.appendChild(b);
  }
  const mark = () => {
    for (const b of box.querySelectorAll("button")) b.classList.toggle("on", parseFloat(b.dataset.speed) === speed);
  };
  mark();
}
function buildSteps() {
  const box = document.getElementById("steps");
  const input = document.createElement("input");
  input.id = "stepInput"; input.type = "number"; input.min = 1; input.max = 90; input.value = 10;
  const mark = () => {
    for (const b of box.querySelectorAll("button")) b.classList.toggle("on", parseFloat(b.dataset.deg) === stepDegrees());
  };
  for (const deg of [5, 10, 20, 45]) {
    const b = document.createElement("button");
    b.textContent = deg + "°"; b.dataset.deg = deg;
    b.onclick = () => { input.value = deg; mark(); };
    box.appendChild(b);
  }
  input.oninput = mark;
  box.appendChild(input);
  box.appendChild(document.createTextNode("°"));
  mark();
}
function build() {
  const w = document.getElementById("worlds");
  for (const [id, label] of CFG.worlds) {
    const b = document.createElement("button");
    b.className = "world"; b.id = "world-" + id; setPair(b, label, "🌍 ");
    b.onclick = () => post("/api/world", {world: id});
    w.appendChild(b);
  }
  buildSteps();
  buildSpeeds();
  const s = document.getElementById("sections");
  for (const sc of CFG.sections) {
    const sec = document.createElement("section");
    sec.innerHTML = "<h2></h2>";
    setPair(sec.querySelector("h2"), sc.title);
    for (const [name, id, plus, minus] of sc.rows) {
      const row = document.createElement("div");
      row.className = "jog";
      row.innerHTML = "<span class='name'></span>";
      setPair(row.querySelector(".name"), name);
      for (const [label, dir] of [[plus, 1], [minus, -1]]) {
        const b = document.createElement("button");
        b.className = "action"; setPair(b, label);
        b.onclick = () => post("/api/jog", {id: id, direction: dir, degrees: stepDegrees(), speed: speed});
        row.appendChild(b);
      }
      sec.appendChild(row);
    }
    if (sc.buttons.length) {
      const r = document.createElement("div");
      r.className = "row"; r.style.marginTop = "8px";
      for (const [id, label] of sc.buttons) {
        const b = document.createElement("button");
        b.className = "action"; setPair(b, label);
        b.onclick = () => post("/api/action/" + id);
        r.appendChild(b);
      }
      sec.appendChild(r);
    }
    s.appendChild(sec);
  }
  if (CFG.embed) document.getElementById("view").style.display = "";
}

// Embedded view: the Open3D WebRTC server streams the simulation window into the <video> element
let streamer = null, streamedRun = null, scriptsLoaded = null;
function loadScript(src) {
  return new Promise((resolve, reject) => {
    const el = document.createElement("script");
    el.src = src; el.onload = resolve; el.onerror = reject;
    document.head.appendChild(el);
  });
}
// The stream library mishandles touch (no button, crashes on touchend), so touch is translated here:
// one finger rotates the view, two fingers (pinch) zoom it.
(function () {
  const video = document.getElementById("sim");
  let mode = null, lastDist = 0;
  const pos = t => { const r = video.getBoundingClientRect();
                     return [Math.round(t.clientX - r.left), Math.round(t.clientY - r.top)]; };
  const send = (type, t, extra) => {
    if (!streamer) return;
    const [x, y] = pos(t);
    streamer.sendJsonData(Object.assign({window_uid: "window_0", class_name: "MouseEvent", type, x, y,
                                         modifiers: 0}, extra));
  };
  const button = {button: {button: "LEFT", count: 1}};
  const spread = e => Math.hypot(e.touches[0].clientX - e.touches[1].clientX,
                                 e.touches[0].clientY - e.touches[1].clientY);
  video.addEventListener("touchstart", e => {
    e.preventDefault(); e.stopImmediatePropagation();
    if (e.touches.length === 1) { mode = "rotate"; send("BUTTON_DOWN", e.touches[0], button); }
    else if (e.touches.length === 2) {
      if (mode === "rotate") send("BUTTON_UP", e.touches[0], button);
      mode = "zoom"; lastDist = spread(e);
    }
  }, {capture: true, passive: false});
  video.addEventListener("touchmove", e => {
    e.preventDefault(); e.stopImmediatePropagation();
    if (mode === "rotate" && e.touches.length === 1) send("DRAG", e.touches[0], {move: {buttons: 1}});
    else if (mode === "zoom" && e.touches.length === 2) {
      const d = spread(e);
      if (Math.abs(d - lastDist) > 8) {
        send("WHEEL", e.touches[0], {wheel: {dx: 0, dy: d > lastDist ? 1 : -1, isTrackpad: 0}});
        lastDist = d;
      }
    }
  }, {capture: true, passive: false});
  const end = e => {
    e.preventDefault(); e.stopImmediatePropagation();
    if (mode === "rotate") send("BUTTON_UP", e.changedTouches[0], button);
    if (e.touches.length === 0) mode = null; else mode = "none";
  };
  video.addEventListener("touchend", end, {capture: true, passive: false});
  video.addEventListener("touchcancel", end, {capture: true, passive: false});
})();
async function connectView(run, port) {
  streamedRun = run;
  const STREAM_URL = location.protocol + "//" + location.hostname + ":" + port;
  if (streamer) { try { streamer.disconnect(); } catch (e) {} streamer = null; }
  try {
    // the stream server only exists while a simulation runs, so the scripts are loaded lazily
    scriptsLoaded = scriptsLoaded || loadScript(STREAM_URL + "/libs/adapter.min.js")
        .then(() => loadScript(STREAM_URL + "/webrtcstreamer.js"));
    await scriptsLoaded;
    streamer = new WebRtcStreamer("sim", STREAM_URL);
    streamer.connect("window_0", null, "rtptransport=tcp&timeout=60");
  } catch (e) {
    scriptsLoaded = null; streamedRun = null;  // try again on the next refresh
  }
}
async function refresh() {
  const st = await (await fetch("/api/status")).json();
  document.getElementById("status").textContent = TEXT[language][st.state] || st.state;
  const ballEl = document.getElementById("ball");
  ballEl.style.display = st.ball ? "block" : "none";
  if (st.ball) {
    const f = v => v === null ? "–" : v + " cm";
    const text = TEXT[language];
    ballEl.innerHTML = text.ballDistance + " <b>" + f(st.ball.distance) + "</b><br>" +
      text.ballAfter + " <b>" + (st.ball.after !== null ? f(st.ball.after)
        : st.ball.touched ? text.waiting : text.untouched) + "</b><br>" +
      text.ballElapsed + " <b>" + (st.ball.moving !== null ? st.ball.moving + " s" : "–") + "</b>";
  }
  const running = st.state === "ready" || st.state === "busy";
  for (const b of document.querySelectorAll("button.action")) b.disabled = !running;
  for (const [id] of CFG.worlds)
    document.getElementById("world-" + id).classList.toggle("active", st.world === id);
  if (CFG.embed && running && streamedRun !== st.run) connectView(st.run, st.stream_port);
}
build(); applyLanguage(); setInterval(refresh, 700);
</script></body></html>
"""


def create_app(sim: Simulation, embed: bool) -> Flask:
    import json
    app = Flask(__name__)
    config = {"worlds": [[k, v[0]] for k, v in WORLDS.items()], "sections": SECTIONS,
              "embed": embed}

    @app.get("/")
    def index():
        return PAGE.replace("__CONFIG__", json.dumps(config))

    @app.get("/api/status")
    def status():
        return jsonify(sim.get_state())

    @app.post("/api/world")
    def load_world():
        world = (request.get_json(silent=True) or {}).get("world")
        if world not in WORLDS:
            return jsonify(error="unknown world"), 400
        sim.start(world)
        return jsonify(ok=True)

    @app.post("/api/restart")
    def restart():
        sim.start(sim.world or next(iter(WORLDS)))
        return jsonify(ok=True)

    @app.post("/api/action/<name>")
    def action(name):
        if name != "stop" and name not in ACTIONS:
            return jsonify(error="unknown action"), 400
        return jsonify(ok=sim.send(name))

    @app.post("/api/jog")
    def jog():
        data = request.get_json(silent=True) or {}
        jog_id, direction = data.get("id"), data.get("direction")
        try:
            degrees = float(data.get("degrees"))
        except (TypeError, ValueError):
            return jsonify(error="invalid angle"), 400
        try:
            speed = float(data.get("speed", 1))
        except (TypeError, ValueError):
            return jsonify(error="invalid speed"), 400
        if jog_id not in JOGS or direction not in (-1, 1) or not 0 < degrees <= 90 or not 0 < speed <= 5:
            return jsonify(error="invalid movement"), 400
        joint, sign = JOGS[jog_id]
        return jsonify(ok=sim.send(("jog", joint, sign * direction * degrees, speed)))

    return app


def main() -> None:
    parser = argparse.ArgumentParser(description="Web control panel for pyCub")
    parser.add_argument("--host", default="0.0.0.0",
                        help="address for incoming connections (use 0.0.0.0 for Wi-Fi/LAN; unencrypted)")
    parser.add_argument("--port", type=int, default=5000)
    parser.add_argument("--stream-port", type=int, default=None,
                        help="fixed video stream port (default: selected automatically)")
    parser.add_argument("--world", choices=list(WORLDS), default="ball", help="world to load at startup")
    parser.add_argument("--separate-window", action="store_true",
                        help="show the robot in a separate window (Open3D) instead of on the page")
    parser.add_argument("--headless", action="store_true", help="run without displaying the simulation (for testing)")
    parser.add_argument("--no-browser", action="store_true", help="do not open a browser automatically")
    args = parser.parse_args()

    if not 1 <= args.port <= 65535:
        parser.error("--port must be between 1 and 65535")
    if args.stream_port is not None and not 1 <= args.stream_port <= 65535:
        parser.error("--stream-port must be between 1 and 65535")
    if args.stream_port == args.port:
        parser.error("--port and --stream-port must be different")

    embed = not (args.separate_window or args.headless)
    sim = Simulation(embed, args.headless, args.host, args.stream_port)
    atexit.register(sim.stop)
    sim.start(args.world)
    app = create_app(sim, embed)
    if not args.no_browser:
        threading.Timer(1.0, lambda: webbrowser.open(f"http://localhost:{args.port}")).start()
    app.run(host=args.host, port=args.port, threaded=True)


if __name__ == "__main__":
    main()
