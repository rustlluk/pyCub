"""
High-level beginner-friendly Python API for pyCub ("Kid Programming" & "Scratch" Blockly runtime).

Allows students to control iCub with intuitive one-line commands without writing
simulation loops or radian conversions, both inside the standalone desktop app
(Modes 2 & 3) and when executed directly as a standard `.py` script.
"""
from __future__ import annotations

import math
import threading
import time
from typing import Callable, Dict, Optional

import numpy as np


class KidScriptStopped(BaseException):
    """Raised cooperatively to stop a running Kid Scratch / Kid Programming script immediately."""


def _norm_side(side: str) -> str:
    """Normalize English or Czech arm side input to ``'l'`` or ``'r'``."""
    s = str(side).strip().lower()
    if s in ("l", "left", "leva", "levá"):
        return "l"
    if s in ("r", "right", "prava", "pravá"):
        return "r"
    raise ValueError(f"Unknown arm side '{side}'. Use 'left' or 'right'.")


ARM_UP = dict(shoulder_pitch=-1.6, shoulder_roll=0.3, shoulder_yaw=0.0, elbow=0.6)
ARM_DOWN = dict(shoulder_pitch=-0.6, shoulder_roll=0.5, shoulder_yaw=0.0, elbow=0.9)


class KidRobot:
    """
    Controller binding `kid_api` functions either to a managed step-callback (inside the
    interactive simulation worker) or to a standalone `pyCub` client instance.
    """

    def __init__(
        self,
        client,
        step_fn: Optional[Callable[[], None]] = None,
        stop_event: Optional[threading.Event] = None,
        highlight_cb: Optional[Callable[[str], None]] = None,
        telemetry_getter: Optional[Callable[[], Dict[str, object]]] = None,
    ):
        """Initialize a KidRobot controller bound to ``client`` and optional UI callbacks."""
        self.client = client
        self._step_fn = step_fn
        self._stop_event = stop_event
        self._highlight_cb = highlight_cb
        self._telemetry_getter = telemetry_getter

    def check_stop(self) -> None:
        """Raise ``KidScriptStopped`` when cancellation is requested or the simulator closes."""
        if self._stop_event is not None and self._stop_event.is_set():
            raise KidScriptStopped()
        if not self.client.is_alive():
            raise KidScriptStopped()

    def _tick(self) -> None:
        """Advance one simulation step while honoring stop requests."""
        self.check_stop()
        if self._step_fn is not None:
            self._step_fn()
        else:
            self.client.update_simulation(0.01)

    def highlight_block(self, block_id: str) -> None:
        """Notify the Blockly editor which block is executing and yield briefly."""
        self.check_stop()
        if self._highlight_cb is not None:
            self._highlight_cb(str(block_id))
        # Tiny cooperative yield so tight Scratch `while True:` loops stay responsive and stoppable
        time.sleep(0.005)
        self.check_stop()

    def _go_radians(self, targets: Dict[str, float], velocity: float = 1.5, timeout: float = 4.5) -> None:
        """Command target joint angles in radians and tick until done or timed out."""
        self.check_stop()
        names = list(targets.keys())
        vals = [float(targets[n]) for n in names]
        self.client.move_position(
            names,
            vals,
            wait=False,
            velocity=float(velocity),
            timeout=float(timeout),
            check_collision=False,
        )
        t0 = time.time()
        while not self.client.motion_done(names, check_collision=False):
            if time.time() - t0 > timeout + 0.5:
                break
            self._tick()

    def move_joint(self, joint_name: str, target_degrees: float, speed: float = 1.5) -> None:
        """Move a named joint to an absolute angle in degrees."""
        self.check_stop()
        _, joint_id = self.client.find_joint_id(joint_name)
        j = self.client.joints[joint_id]
        rad = float(np.clip(np.deg2rad(float(target_degrees)), j.lower_limit, j.upper_limit))
        self._go_radians({joint_name: rad}, velocity=float(speed), timeout=5.0)

    def jog_joint(self, joint_name: str, delta_degrees: float, speed: float = 1.5) -> None:
        """Move a named joint relatively by `delta_degrees`."""
        self.check_stop()
        _, joint_id = self.client.find_joint_id(joint_name)
        j = self.client.joints[joint_id]
        cur = float(self.client.get_joint_state(joint_name)[0])
        target = float(np.clip(cur + np.deg2rad(float(delta_degrees)), j.lower_limit, j.upper_limit))
        self._go_radians({joint_name: target}, velocity=float(speed), timeout=5.0)

    def get_joint(self, joint_name: str) -> float:
        """Return the current angle of `joint_name` in degrees."""
        self.check_stop()
        rad = float(self.client.get_joint_state(joint_name)[0])
        return round(float(np.rad2deg(rad)), 2)

    # --- Preset Actions ---

    def wave(self, side: str = "right") -> None:
        """Wave with the `'right'` or `'left'` arm."""
        s = _norm_side(side)
        self._go_radians(
            {f"{s}_shoulder_pitch": -1.2, f"{s}_shoulder_roll": 1.2, f"{s}_shoulder_yaw": 0.0, f"{s}_elbow": 1.2},
            velocity=1.8,
        )
        for _ in range(3):
            self._go_radians({f"{s}_elbow": 0.6}, velocity=4.0)
            self._go_radians({f"{s}_elbow": 1.6}, velocity=4.0)
        self._go_radians({f"{s}_{k}": v for k, v in ARM_DOWN.items()}, velocity=1.8)

    def both_arms_up(self) -> None:
        """Raise both arms to their preset positions."""
        targets = {f"l_{k}": v for k, v in ARM_UP.items()}
        targets.update({f"r_{k}": v for k, v in ARM_UP.items()})
        self._go_radians(targets, velocity=1.8)

    def both_arms_down(self) -> None:
        """Lower both arms to their preset positions."""
        targets = {f"l_{k}": v for k, v in ARM_DOWN.items()}
        targets.update({f"r_{k}": v for k, v in ARM_DOWN.items()})
        self._go_radians(targets, velocity=1.8)

    def look_center(self) -> None:
        """Center the neck joints."""
        self._go_radians({"neck_yaw": 0.0, "neck_pitch": 0.0, "neck_roll": 0.0}, velocity=1.5)

    def stand_straight(self) -> None:
        """Return the torso to its neutral upright posture."""
        self._go_radians({"torso_yaw": 0.0, "torso_pitch": 0.0, "torso_roll": 0.0}, velocity=1.5)

    def reset_pose(self) -> None:
        """Return all body joints to their configured initial angles."""
        initial = self.client.config.initial_joint_angles
        targets = {
            j.name: float(np.deg2rad(getattr(initial, j.name))) if hasattr(initial, j.name) else 0.0
            for j in self.client.joints
            if "_hand_" not in j.name and "eye" not in j.name
        }
        self._go_radians(targets, velocity=1.8)

    def stop(self) -> None:
        """Stop all current joint movements."""
        self.check_stop()
        self.client.stop_robot()

    # --- Arm Step Helpers ---

    def shoulder_up(self, side: str = "right", degrees: float = 15.0, speed: float = 1.5) -> None:
        """Raise the selected shoulder by ``degrees``."""
        s = _norm_side(side)
        self.jog_joint(f"{s}_shoulder_pitch", -abs(float(degrees)), speed=speed)

    def shoulder_down(self, side: str = "right", degrees: float = 15.0, speed: float = 1.5) -> None:
        """Lower the selected shoulder by ``degrees``."""
        s = _norm_side(side)
        self.jog_joint(f"{s}_shoulder_pitch", abs(float(degrees)), speed=speed)

    def shoulder_out(self, side: str = "right", degrees: float = 15.0, speed: float = 1.5) -> None:
        """Move the selected arm away from the torso by ``degrees``."""
        s = _norm_side(side)
        self.jog_joint(f"{s}_shoulder_roll", abs(float(degrees)), speed=speed)

    def shoulder_in(self, side: str = "right", degrees: float = 15.0, speed: float = 1.5) -> None:
        """Move the selected arm toward the torso by ``degrees``."""
        s = _norm_side(side)
        self.jog_joint(f"{s}_shoulder_roll", -abs(float(degrees)), speed=speed)

    def rotate_arm(self, side: str = "right", degrees: float = 15.0, speed: float = 1.5) -> None:
        """Rotate the selected upper arm by ``degrees``."""
        s = _norm_side(side)
        self.jog_joint(f"{s}_shoulder_yaw", float(degrees), speed=speed)

    def bend_elbow(self, side: str = "right", degrees: float = 15.0, speed: float = 1.5) -> None:
        """Bend the selected elbow by ``degrees``."""
        s = _norm_side(side)
        self.jog_joint(f"{s}_elbow", abs(float(degrees)), speed=speed)

    def straighten_elbow(self, side: str = "right", degrees: float = 15.0, speed: float = 1.5) -> None:
        """Extend the selected elbow by ``degrees``."""
        s = _norm_side(side)
        self.jog_joint(f"{s}_elbow", -abs(float(degrees)), speed=speed)

    def rotate_wrist(self, side: str = "right", degrees: float = 15.0, speed: float = 1.5) -> None:
        """Rotate the selected wrist by ``degrees``."""
        s = _norm_side(side)
        self.jog_joint(f"{s}_wrist_prosup", float(degrees), speed=speed)

    # --- Head & Torso Helpers ---

    def turn_head(self, degrees: float = 15.0, speed: float = 1.5) -> None:
        """Turn the head about the neck yaw axis by ``degrees``."""
        self.jog_joint("neck_yaw", float(degrees), speed=speed)

    def look_up(self, degrees: float = 15.0, speed: float = 1.5) -> None:
        """Tilt the head upward by ``degrees``."""
        self.jog_joint("neck_pitch", abs(float(degrees)), speed=speed)

    def look_down(self, degrees: float = 15.0, speed: float = 1.5) -> None:
        """Tilt the head downward by ``degrees``."""
        self.jog_joint("neck_pitch", -abs(float(degrees)), speed=speed)

    def tilt_head(self, degrees: float = 15.0, speed: float = 1.5) -> None:
        """Tilt the head sideways by ``degrees``."""
        self.jog_joint("neck_roll", float(degrees), speed=speed)

    def turn_body(self, degrees: float = 15.0, speed: float = 1.5) -> None:
        """Rotate the torso about its yaw axis by ``degrees``."""
        self.jog_joint("torso_yaw", float(degrees), speed=speed)

    def bend_forward(self, degrees: float = 15.0, speed: float = 1.5) -> None:
        """Bend the torso forward by ``degrees``."""
        self.jog_joint("torso_pitch", abs(float(degrees)), speed=speed)

    def bend_back(self, degrees: float = 15.0, speed: float = 1.5) -> None:
        """Bend the torso backward by ``degrees``."""
        self.jog_joint("torso_pitch", -abs(float(degrees)), speed=speed)

    def bend_sideways(self, degrees: float = 15.0, speed: float = 1.5) -> None:
        """Tilt the torso sideways by ``degrees``."""
        self.jog_joint("torso_roll", float(degrees), speed=speed)

    def wait(self, seconds: float = 1.0) -> None:
        """Pause script execution for `seconds` while keeping physics running."""
        end_t = time.time() + max(0.0, float(seconds))
        while time.time() < end_t:
            self._tick()

    # --- Sensors / Ball Telemetry ---

    def _ball_telemetry(self) -> Dict[str, object]:
        """Return current ball distance, contact flag, and hit count."""
        if self._telemetry_getter is not None:
            return self._telemetry_getter() or {}
        ball_id = next((o[0] for o in self.client.other_objects if o[1] == "ball"), None)
        table_id = next((o[0] for o in self.client.other_objects if o[1] == "table"), None)
        if ball_id is None:
            return {"distance": 0.0, "touched": False, "hits": 0}
        dist_cm = 0.0
        if table_id is not None:
            pts = self.client.getClosestPoints(ball_id, table_id, 10.0)
            if pts:
                d = min(p[8] for p in pts)
                if not math.isnan(d):
                    dist_cm = round(d * 100.0, 1)
        touched = bool(self.client.getContactPoints(self.client.robot, ball_id))
        return {"distance": dist_cm, "touched": touched, "hits": int(touched)}

    def ball_distance(self) -> float:
        """Distance of the ball from the table in cm (0.0 if on table or no ball)."""
        self.check_stop()
        val = self._ball_telemetry().get("distance")
        return float(val) if val is not None else 0.0

    def ball_touched(self) -> bool:
        """True if the robot has touched the ball in the current run."""
        self.check_stop()
        return bool(self._ball_telemetry().get("touched", False))

    def ball_hits(self) -> int:
        """Number of times the robot hit the ball in the current run."""
        self.check_stop()
        return int(self._ball_telemetry().get("hits", 0))

    def skin_touched(self) -> bool:
        """True if tactile skin is enabled and currently registering contact."""
        self.check_stop()
        if not getattr(self.client.config.skin, "use", False):
            return False
        return any(np.any(act > 0) for act in self.client.skin_activations.values())


# --- Global Active Controller Binding (for concise `wave()`, `shoulder_up()`, etc.) ---

_ACTIVE_ROBOT: Optional[KidRobot] = None


def _bind_active_controller(controller: Optional[KidRobot]) -> None:
    """Bind or clear the module-level active ``KidRobot`` controller."""
    global _ACTIVE_ROBOT
    _ACTIVE_ROBOT = controller


def get_robot() -> KidRobot:
    """Return the active KidRobot controller (lazily starting `with_ball.yaml` if run standalone)."""
    global _ACTIVE_ROBOT
    if _ACTIVE_ROBOT is None:
        from icub_pybullet.pycub import pyCub

        client = pyCub(config="with_ball.yaml")
        _ACTIVE_ROBOT = KidRobot(client)
    return _ACTIVE_ROBOT


def _highlight_block(block_id: str) -> None:
    """Forward active Blockly block highlighting to the bound controller."""
    get_robot().highlight_block(block_id)


def wave(side: str = "right") -> None:
    """Wave using the left or right arm.

    :param side: Arm to use ("left" or "right").
    """
    get_robot().wave(side=side)


def left_wave() -> None:
    """Wave using the left arm."""
    get_robot().wave(side="left")


def right_wave() -> None:
    """Wave using the right arm."""
    get_robot().wave(side="right")


def both_arms_up() -> None:
    """Raise both arms to their preset positions."""
    get_robot().both_arms_up()


def both_arms_down() -> None:
    """Lower both arms to their preset positions."""
    get_robot().both_arms_down()


def look_center() -> None:
    """Center the neck joints."""
    get_robot().look_center()


def stand_straight() -> None:
    """Return the torso to its neutral position."""
    get_robot().stand_straight()


def reset_pose() -> None:
    """Return the robot to its starting pose."""
    get_robot().reset_pose()


def stop() -> None:
    """Stop current joint movements."""
    get_robot().stop()


def shoulder_up(side: str = "right", degrees: float = 15.0, speed: float = 1.5) -> None:
    """Raise the selected shoulder.

    :param side: Arm to use ("left" or "right").
    :param degrees: Change from the current angle, in degrees.
    :param speed: Maximum joint speed in radians per second.
    """
    get_robot().shoulder_up(side=side, degrees=degrees, speed=speed)


def shoulder_down(side: str = "right", degrees: float = 15.0, speed: float = 1.5) -> None:
    """Lower the selected shoulder.

    :param side: Arm to use ("left" or "right").
    :param degrees: Change from the current angle, in degrees.
    :param speed: Maximum joint speed in radians per second.
    """
    get_robot().shoulder_down(side=side, degrees=degrees, speed=speed)


def shoulder_out(side: str = "right", degrees: float = 15.0, speed: float = 1.5) -> None:
    """Move the selected arm away from the torso.

    :param side: Arm to use ("left" or "right").
    :param degrees: Change from the current angle, in degrees.
    :param speed: Maximum joint speed in radians per second.
    """
    get_robot().shoulder_out(side=side, degrees=degrees, speed=speed)


def shoulder_in(side: str = "right", degrees: float = 15.0, speed: float = 1.5) -> None:
    """Move the selected arm toward the torso.

    :param side: Arm to use ("left" or "right").
    :param degrees: Change from the current angle, in degrees.
    :param speed: Maximum joint speed in radians per second.
    """
    get_robot().shoulder_in(side=side, degrees=degrees, speed=speed)


def rotate_arm(side: str = "right", degrees: float = 15.0, speed: float = 1.5) -> None:
    """Rotate the selected upper arm.

    :param side: Arm to use ("left" or "right").
    :param degrees: Change from the current angle, in degrees.
    :param speed: Maximum joint speed in radians per second.
    """
    get_robot().rotate_arm(side=side, degrees=degrees, speed=speed)


def bend_elbow(side: str = "right", degrees: float = 15.0, speed: float = 1.5) -> None:
    """Bend the selected elbow.

    :param side: Arm to use ("left" or "right").
    :param degrees: Change from the current angle, in degrees.
    :param speed: Maximum joint speed in radians per second.
    """
    get_robot().bend_elbow(side=side, degrees=degrees, speed=speed)


def straighten_elbow(side: str = "right", degrees: float = 15.0, speed: float = 1.5) -> None:
    """Extend the selected elbow.

    :param side: Arm to use ("left" or "right").
    :param degrees: Change from the current angle, in degrees.
    :param speed: Maximum joint speed in radians per second.
    """
    get_robot().straighten_elbow(side=side, degrees=degrees, speed=speed)


def rotate_wrist(side: str = "right", degrees: float = 15.0, speed: float = 1.5) -> None:
    """Rotate the selected wrist.

    :param side: Arm to use ("left" or "right").
    :param degrees: Change from the current angle, in degrees.
    :param speed: Maximum joint speed in radians per second.
    """
    get_robot().rotate_wrist(side=side, degrees=degrees, speed=speed)


def turn_head(degrees: float = 15.0, speed: float = 1.5) -> None:
    """Turn the head about the neck yaw axis.

    :param degrees: Change from the current angle, in degrees.
    :param speed: Maximum joint speed in radians per second.
    """
    get_robot().turn_head(degrees=degrees, speed=speed)


def look_up(degrees: float = 15.0, speed: float = 1.5) -> None:
    """Tilt the head upward.

    :param degrees: Change from the current angle, in degrees.
    :param speed: Maximum joint speed in radians per second.
    """
    get_robot().look_up(degrees=degrees, speed=speed)


def look_down(degrees: float = 15.0, speed: float = 1.5) -> None:
    """Tilt the head downward.

    :param degrees: Change from the current angle, in degrees.
    :param speed: Maximum joint speed in radians per second.
    """
    get_robot().look_down(degrees=degrees, speed=speed)


def tilt_head(degrees: float = 15.0, speed: float = 1.5) -> None:
    """Tilt the head to the side.

    :param degrees: Change from the current angle, in degrees.
    :param speed: Maximum joint speed in radians per second.
    """
    get_robot().tilt_head(degrees=degrees, speed=speed)


def turn_body(degrees: float = 15.0, speed: float = 1.5) -> None:
    """Rotate the torso about its yaw axis.

    :param degrees: Change from the current angle, in degrees.
    :param speed: Maximum joint speed in radians per second.
    """
    get_robot().turn_body(degrees=degrees, speed=speed)


def bend_forward(degrees: float = 15.0, speed: float = 1.5) -> None:
    """Bend the torso forward.

    :param degrees: Change from the current angle, in degrees.
    :param speed: Maximum joint speed in radians per second.
    """
    get_robot().bend_forward(degrees=degrees, speed=speed)


def bend_back(degrees: float = 15.0, speed: float = 1.5) -> None:
    """Bend the torso backward.

    :param degrees: Change from the current angle, in degrees.
    :param speed: Maximum joint speed in radians per second.
    """
    get_robot().bend_back(degrees=degrees, speed=speed)


def bend_sideways(degrees: float = 15.0, speed: float = 1.5) -> None:
    """Tilt the torso to the side.

    :param degrees: Change from the current angle, in degrees.
    :param speed: Maximum joint speed in radians per second.
    """
    get_robot().bend_sideways(degrees=degrees, speed=speed)


def move_joint(joint_name: str, target_degrees: float, speed: float = 1.5) -> None:
    """Move a joint to an absolute angle.

    :param joint_name: Simulator joint name, such as neck_pitch.
    :param target_degrees: Absolute target angle in degrees.
    :param speed: Maximum joint speed in radians per second.
    """
    get_robot().move_joint(joint_name=joint_name, target_degrees=target_degrees, speed=speed)


def jog_joint(joint_name: str, delta_degrees: float, speed: float = 1.5) -> None:
    """Move a joint relative to its current angle.

    :param joint_name: Simulator joint name, such as neck_pitch.
    :param delta_degrees: Signed change in degrees.
    :param speed: Maximum joint speed in radians per second.
    """
    get_robot().jog_joint(joint_name=joint_name, delta_degrees=delta_degrees, speed=speed)


def get_joint(joint_name: str) -> float:
    """Read the current joint angle in degrees.

    :param joint_name: Simulator joint name, such as neck_pitch.
    """
    return get_robot().get_joint(joint_name=joint_name)


def wait(seconds: float = 1.0) -> None:
    """Pause the program for the given number of seconds while the simulation runs."""
    get_robot().wait(seconds=seconds)


def ball_distance() -> float:
    """Read the distance between the ball and the table in centimetres."""
    return get_robot().ball_distance()


def ball_touched() -> bool:
    """Read whether the robot has touched the ball."""
    return get_robot().ball_touched()


def ball_hits() -> int:
    """Read the ball hit count from the active controller."""
    return get_robot().ball_hits()


def skin_touched() -> bool:
    """Read whether any enabled skin sensor is registering contact."""
    return get_robot().skin_touched()


__all__ = [
    "KidRobot",
    "KidScriptStopped",
    "get_robot",
    "wave",
    "left_wave",
    "right_wave",
    "both_arms_up",
    "both_arms_down",
    "look_center",
    "stand_straight",
    "reset_pose",
    "stop",
    "shoulder_up",
    "shoulder_down",
    "shoulder_out",
    "shoulder_in",
    "rotate_arm",
    "bend_elbow",
    "straighten_elbow",
    "rotate_wrist",
    "turn_head",
    "look_up",
    "look_down",
    "tilt_head",
    "turn_body",
    "bend_forward",
    "bend_back",
    "bend_sideways",
    "move_joint",
    "jog_joint",
    "get_joint",
    "wait",
    "ball_distance",
    "ball_touched",
    "ball_hits",
    "skin_touched",
]

# Czech command names share the English functions and their active controller.
CZECH_COMMANDS = {
    "wave": "zamávej",
    "left_wave": "zamávej_levou",
    "right_wave": "zamávej_pravou",
    "both_arms_up": "zvedni_obě_ruce",
    "both_arms_down": "spusť_obě_ruce",
    "look_center": "podívej_se_před_sebe",
    "stand_straight": "narovnej_se",
    "reset_pose": "obnov_polohu",
    "stop": "zastav",
    "shoulder_up": "zvedni_rameno",
    "shoulder_down": "spusť_rameno",
    "shoulder_out": "upaž",
    "shoulder_in": "připaž",
    "rotate_arm": "otoč_paži",
    "bend_elbow": "ohni_loket",
    "straighten_elbow": "narovnej_loket",
    "rotate_wrist": "otoč_zápěstí",
    "turn_head": "otoč_hlavu",
    "look_up": "podívej_se_nahoru",
    "look_down": "podívej_se_dolů",
    "tilt_head": "nakloň_hlavu",
    "turn_body": "otoč_trup",
    "bend_forward": "předkloň_se",
    "bend_back": "zakloň_se",
    "bend_sideways": "ukloň_se",
    "move_joint": "nastav_kloub",
    "jog_joint": "posuň_kloub",
    "get_joint": "zjisti_úhel_kloubu",
    "wait": "počkej",
    "ball_distance": "vzdálenost_míčku",
    "ball_touched": "dotkl_se_míčku",
    "ball_hits": "počet_zásahů",
    "skin_touched": "dotyk_kůže",
}
for _english, _czech in CZECH_COMMANDS.items():
    globals()[_czech] = globals()[_english]
__all__.extend(CZECH_COMMANDS.values())
