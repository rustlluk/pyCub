# Using Studio

Launch pyCub Studio and choose a mode from Home. The mode selector in the header
lets you switch activities. The language button switches between English and Czech.

## Getting started

Open the desktop application from its installer or portable distribution, or
install the Python packages into a Python 3.10–3.12 virtual environment and
launch `pycub-studio` from anywhere:

```sh
# From PyPI (either command installs simulator + Studio + handbook + exercises):
python -m pip install pycub-studio
# or: python -m pip install "icub_pybullet[studio]"

# Or from a local repository checkout:
python -m pip install ./icub_pybullet ./studio

pycub-studio
```

## Kid GUI

Choose a world with a ball, then try a preset movement. Use the joint buttons
to move one joint at a time. Adjust the step size for finer motion. The ball
readout shows whether the robot touched the ball and how far it moved.

## Scratch blocks

Snap movement, sensor and control blocks together and run the program. Watch
the highlighted block as the simulator steps through it. Use the Python
preview to see how a block translates to code. Save editable projects as
`.pycub_blocks`, or open the generated Python in Kid Programming.

## Kid Programming

Start with a template or click a command in the palette to insert an example.
Run the program and read its output in the console. Click Stop to interrupt a
loop. A reported error highlights the line in the editor. See the
[Python guide](programming.md) for call hints and definitions.

## Normal Mode

Open a Python file, or start with the script in the first tab. Press F5 to run
the current Python script. Use YAML tabs to edit world configurations. When
working in another editor, open an external script and enable run-on-save
to rerun it after changes. Console output belongs to the script worker.

Use the left and right eye views to inspect RGB and depth images. Use the
examples menu for a working controller, or select a course exercise to copy
its files to your workspace. Run the tester from the exercise tab and inspect
the results report.

## Classroom web sessions

Select the number of simulator instances and start the server in Classroom
Web GUI. Students on the same network can open the displayed landing URL or
scan the QR code. The dashboard shows connected users and instance status.
Open the logs to investigate a failed instance or restart it individually.

## Workspace and packages

Open **Workspace** from the header to locate saved scripts, blocks, examples,
exercises and images. Store custom YAML files in `worlds/`. A world can
inherit built-in settings with `parent: with_ball.yaml`. Put custom geometry in
`meshes/`. Use **Packages / Python** to install extra packages or select an
external interpreter for Normal Mode scripts.
