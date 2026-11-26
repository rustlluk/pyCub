# Programming the robot

pyCub supports three main programming workflows depending on your experience level and whether you want Studio's integrated 3D interface or a standalone Python environment:

| Workflow | Entry API | Units | Simulation loop | Best for |
| --- | --- | --- | --- | --- |
| **Kid modes** (Scratch & Kid Programming in Studio) | `icub_pybullet.kid_api` | **Degrees** | Managed automatically | Beginners, classrooms, quick experiments |
| **Studio Full Mode** (Normal Mode IDE in Studio) | `icub_pybullet.pycub.pyCub` | **Radians** | Explicit (`wait=True` or `update_simulation()`) | Full robot control, cameras, skin, course exercises |
| **Normal / Standalone Python** (Terminal / PyCharm / VS Code) | `pyCub` or `kid_api` | Radians (`pyCub`) / Degrees (`kid_api`) | Explicit (`pyCub`) / Managed (`kid_api`) | Automated evaluation, headless runs, external Open3D/Web GUI |

*(Note: inside `.yaml` world files, `initial_joint_angles` are always written in **degrees**, regardless of mode.)*

---

## Kid modes (`icub_pybullet.kid_api`)

The Kid modes inside **pyCub Studio** — **Kid "Scratch" (Blocks)** and **Kid Programming (Python)** — both run on top of `icub_pybullet.kid_api`.

### How Kid execution works

- Studio keeps an interactive physics worker running continuously in the background for the currently selected world (`with_ball.yaml`, `default.yaml`, `skin_test.yaml`, or your custom `.yaml` world in `~/Documents/pyCub/worlds/`).
- You do **not** instantiate `pyCub(...)` and do **not** write a `while client.is_alive(): client.update_simulation()` loop.
- Every movement command blocks until the motion finishes (or reaches its timeout), stepping the simulation smoothly and checking the **Stop** button cooperatively so infinite loops can always be interrupted immediately.

### From Scratch blocks to Python

In **Kid "Scratch"**, snap movement, loop, logic, and sensor blocks together. Toggle **Python Preview** to watch the live Python translation as you edit blocks, save reusable projects as `.pycub_blocks`, or click **Open as Python in Kid Programming** to export the generated script directly into the text Python editor.

### First program in Kid Programming

Select a world containing a ball (such as *Table with a ball* / `with_ball.yaml`) and run:

```python
from icub_pybullet.kid_api import wave, shoulder_up, look_down, ball_touched, ball_distance

look_down(20)
wave("right")
shoulder_up("left", degrees=20, speed=1.5)
print("Touched:", ball_touched(), "Distance (cm):", ball_distance())
```

Studio automatically exposes all English and Czech `kid_api` commands for autocomplete and inline help, and both spellings share the exact same underlying controller. The equivalent Czech script is:

```python
from icub_pybullet.kid_api import podívej_se_dolů, zamávej, zvedni_rameno, dotkl_se_míčku, vzdálenost_míčku

podívej_se_dolů(20)
zamávej("pravá")
zvedni_rameno("levá", degrees=20, speed=1.5)
print("Dotyk:", dotkl_se_míčku(), "Vzdálenost (cm):", vzdálenost_míčku())
```

Keyword arguments always use the Python parameter names (`side`, `degrees`, `speed`, `target_degrees`, `delta_degrees`, `seconds`).

### Kid command groups

- **Preset actions:** `wave(side="right")`, `left_wave()`, `right_wave()`, `both_arms_up()`, `both_arms_down()`, `look_center()`, `stand_straight()`, `reset_pose()`, `stop()`.
- **Relative arm moves (degrees):** `shoulder_up`, `shoulder_down`, `shoulder_out`, `shoulder_in`, `rotate_arm`, `bend_elbow`, `straighten_elbow`, `rotate_wrist`. All accept `side` (`"left"` / `"right"` or `"levá"` / `"pravá"`), `degrees=15.0`, and `speed=1.5` (rad/s).
- **Relative head & torso moves (degrees):** `turn_head`, `look_up`, `look_down`, `tilt_head`, `turn_body`, `bend_forward`, `bend_back`, `bend_sideways`.
- **Direct joint access & timing:** `move_joint(joint_name, target_degrees, speed=1.5)` for an absolute target angle, `jog_joint(joint_name, delta_degrees, speed=1.5)` for a signed relative offset, `get_joint(joint_name)` to read the current angle in degrees, and `wait(seconds=1.0)` to pause while physics keeps running.
- **Sensors & telemetry:** `ball_distance()` (distance between ball and table in centimetres), `ball_touched()` (`bool`), `ball_hits()` (`int`), and `skin_touched()` (`bool`, active when `skin.use: True`).

Click any entry in the **Command Palette** on the left to insert a working snippet at the cursor, or load a starter template from the toolbar. Syntax and runtime errors highlight the offending line in red and display a beginner-friendly hint in the console.

---

## Studio Full Mode (Normal Mode IDE)

**Normal Mode** in **pyCub Studio** gives you the complete `icub_pybullet.pycub.pyCub` simulator API together with an embedded hardware-accelerated OpenGL 3D viewport, live console, eye cameras, and course exercise tools.

### Running scripts in Studio Normal Mode

- Edit `.py` controllers and `.yaml` world configs across multiple tabs and press **F5** (or click **Run Script**) to execute the active tab.
- Each run starts in a clean isolated worker subprocess (`SimProcessSupervisor`) whose rendering streams directly into Studio's embedded 3D viewport—no separate Open3D window is opened.
- To code in PyCharm or VS Code while using Studio's 3D viewport and camera panels, click **Run External .py…** and enable **Auto-run external file on save**, or launch Studio directly from the command line:

```sh
python -m studio.main --run-script path/to/your_controller.py
```

### Minimal `pyCub` controller structure

In Full / Normal mode, your script creates the `pyCub` client, selects a world `.yaml` config, commands motions in **radians**, and keeps the simulation alive at the end until you click **Stop**:

```python
from icub_pybullet.pycub import pyCub

client = pyCub(config="with_ball.yaml")

# Blocking joint motion (waits until target is reached or collision/timeout occurs)
client.move_position("neck_pitch", -0.25)

# Non-blocking joint motion: advance simulation manually in your own loop
client.move_position(["torso_pitch", "torso_yaw"], [0.30, -0.60], wait=False, velocity=2.0)
while not client.motion_done():
    client.update_simulation()

# Keep the 3D view responsive after your routine finishes
while client.is_alive():
    client.update_simulation()
```

### Joint, velocity, and Cartesian control

- **Position control (`move_position`):** Pass a single joint name/ID or equal-length lists of joints and target angles in **radians**. Optional parameters: `wait=True`, `velocity=1` (rad/s), `set_col_state=True`, `check_collision=True`, and `timeout=None` (seconds). With `wait=False`, drive the simulation yourself with `client.update_simulation()` until `client.motion_done()` returns `True`, or call `client.wait_motion_done()`.
- **Velocity control (`move_velocity`):** Command target angular speeds in **radians per second** for one joint or a list of joints.
- **Reading joint state & stopping:** `client.get_joint_state(joints=None)` returns current joint positions in radians (defaulting to all movable joints); `client.stop_robot(joints=None)` halts active position or velocity targets.
- **Cartesian control (`move_cartesian` & `Pose`):** Read the current end-effector pose with `pose = client.end_effector.get_position()`, adjust `pose.pos` (`[x, y, z]` in metres) or `pose.ori` (quaternion `[x, y, z, w]`), and call `client.move_cartesian(pose, wait=True, velocity=1, check_collision=True, timeout=None)`. PyBullet computes the inverse kinematics solution and commands the movable joints automatically.

```python
from icub_pybullet.pycub import pyCub, Pose

client = pyCub(config="with_ball.yaml")
cur = client.end_effector.get_position()
target = Pose(list(cur.pos), list(cur.ori))
target.pos[2] -= 0.15
target.pos[0] -= 0.10
client.move_cartesian(target, velocity=1.5)
```

- **Jacobians (`compute_jacobian`):** Compute the base-frame geometric Jacobian and active joint names for any kinematic chain (`"left_arm"`, `"right_arm"`, `"left_leg"`, `"right_leg"`, `"torso"`, `"head"`):

```python
J, active_joints = client.compute_jacobian("right_arm", end="r_hand")
```

### Eye cameras, tactile skin, and workspace tools

- **Eye cameras (`l_eye`, `r_eye`):** Toggle **Left Eye (`l_eye`)** or **Right Eye (`r_eye`)** in the Normal Mode toolbar, or set `eyes.l_eye: True` / `eyes.r_eye: True` in your `.yaml` config. In Python, call `client.get_camera_images(eyes)` for RGB arrays and `client.get_camera_depth_images(eyes)` for depth arrays. Use **Save RGB** / **Save Depth** to export PNGs to `~/Documents/pyCub/images/`.
- **Tactile skin:** Set `skin.use: True` in your config (see `skin_test.yaml`), step the simulation with `client.update_simulation()`, and inspect `client.skin_activations` and `client.activated_skin_points`.
- **Examples & Course Exercises:** Open the **Examples & Exercises** menu to load bundled scripts or copy Exercises `1–5` into `~/Documents/pyCub/exercises/`. When an exercise file is open, click **Run Exercise Tester** to run its automated verification suite and view the HTML report.
- **Extra packages & custom Python:** Use **Packages / Python** in the top bar to install PyPI wheels into `~/Documents/pyCub/extra_packages/`, attach `site-packages` from an existing virtual environment, or select an external Python executable for Normal Mode scripts.

---

## Normal / Standalone Python mode (outside Studio)

You can also run any pyCub script directly from a terminal, PyCharm, or VS Code using an environment where `icub_pybullet` is installed (`python3 -m pip install icub_pybullet` or `python3 -m pip install ./icub_pybullet`), without launching Studio at all.

### Visualizer backends (`gui` in `.yaml`)

When running outside Studio, `pyCub` reads the `gui` block of your `.yaml` config:

```yaml
gui:
  standard: True   # Native Open3D 3D window
  web: False       # Browser-based visualizer (slower fallback)
```

- **Open3D desktop window (`standard: True, web: False`):** Default interactive 3D window when running locally.
- **Browser visualizer (`standard: False, web: True`):** Serves an interactive view in your web browser—useful over remote sessions or if host OpenGL drivers conflict with Open3D.
- **Headless mode (`standard: False, web: False`):** No 3D GUI is created. Best for automated tests, reinforcement learning, batch evaluation, and CI.

*(Inside Studio Normal Mode, Studio automatically overrides these flags to route rendering into its embedded OpenGL viewport.)*

### Running `kid_api` scripts standalone

Scripts written for Kid Programming using `icub_pybullet.kid_api` also run directly as standard Python files:

```sh
python my_kid_script.py
```

On the first command call, `get_robot()` lazily creates `pyCub(config="with_ball.yaml")` and steps the simulation automatically.

### Config resolution and running exercises from CLI

When you pass a relative config name such as `pyCub(config="with_ball.yaml")`, pyCub searches in order:
1. Direct / absolute path or relative to the current working directory.
2. Relative to the calling script's directory (and a `configs/` subfolder next to it).
3. Your user workspace (`~/Documents/pyCub/worlds/`, `~/Documents/pyCub/configs/`, `~/Documents/pyCub/`).
4. Built-in `icub_pybullet/configs/` (`default.yaml`, `with_ball.yaml`, `skin_test.yaml`).

Custom `.yaml` configs can inherit built-in settings via `parent: with_ball.yaml` and override only the fields you need (such as `urdfs`, `eyes`, `skin`, or `end_effector`).

To run an exercise tester directly from the command line outside Studio:

```sh
python exercises/exercise_1/exercise_1_tester.py
```

---

## Inspecting calls in the editor

Both **Kid Programming** and **Normal Mode** in Studio include live Jedi-powered code inspection:

- **Hover documentation:** Hover the mouse over a function or class name to read its signature and docstring without moving the cursor. Hover help formats `:param:` entries and trailing inline parameter comments on separate lines.
- **Signature & parameter hints:** Pause with the cursor inside parentheses of a function call (or press **Ctrl+Shift+Space**) to view the active parameter and default values.
- **Go to Definition (`F12`):** Place the cursor on any symbol and press **F12** to view its source definition and docstring—working for both unsaved functions in your current buffer and `icub_pybullet` library code.

---

## Motion, kinematics, and sensor reference

### Quick comparison: Kid API vs Full `pyCub` API

| Task | Kid API (`icub_pybullet.kid_api`) | Full `pyCub` API (`client = pyCub(...)`) |
| --- | --- | --- |
| Target joint angle | `move_joint("neck_pitch", -15)` *(degrees)* | `client.move_position("neck_pitch", -0.26)` *(radians)* |
| Multi-joint move | Call sequentially or loop | `client.move_position(["torso_pitch", "torso_yaw"], [0.3, -0.5])` |
| Relative joint offset | `jog_joint("r_elbow", 20)` *(degrees)* | Read `get_joint_state(...)` + `move_position(...)` |
| Joint velocity | — | `client.move_velocity(joints, velocities)` *(rad/s)* |
| Cartesian IK move | — | `client.move_cartesian(pose)` *(`Pose(pos, ori)`)* |
| Read joint angle | `get_joint("neck_pitch")` *(degrees)* | `client.get_joint_state("neck_pitch")` *(radians)* |
| Camera RGB / Depth | — | `client.get_camera_images(eyes)` / `client.get_camera_depth_images(eyes)` |
| Ball telemetry | `ball_distance()`, `ball_touched()`, `ball_hits()` | Inspect `client.other_objects` + `getClosestPoints` / `getContactPoints` |
| Tactile skin | `skin_touched()` | `client.skin_activations`, `client.activated_skin_points` |
| Step simulation | Handled internally inside each command | `client.update_simulation()` / `client.wait_motion_done()` |

### Key iCub joints and kinematic chains

- **Torso:** `torso_pitch`, `torso_roll`, `torso_yaw`
- **Head & eyes:** `neck_pitch`, `neck_roll`, `neck_yaw`, `eyes_tilt`, `l_eye_pan_joint`, `r_eye_pan_joint`
- **Left / right arms (`l_` / `r_`):** `*_shoulder_pitch`, `*_shoulder_roll`, `*_shoulder_yaw`, `*_elbow`, `*_wrist_prosup`, `*_wrist_pitch`, `*_wrist_yaw`, plus finger joints `*_hand_*`
- **Left / right legs (`l_` / `r_`):** `*_hip_pitch`, `*_hip_roll`, `*_hip_yaw`, `*_knee`, `*_ankle_pitch`, `*_ankle_roll`
- **Kinematic chains (`CHAINS`):** `"left_arm"`, `"right_arm"`, `"left_leg"`, `"right_leg"`, `"torso"`, `"head"`

The bundled [Example controllers](examples.rst) and [API reference](autoapi/index.rst) document all classes, method signatures, and default parameters in detail.
