# Vision-driven pick-and-place on a Dobot Magician Lite

Colour-sorted pick-and-place on a 4-DOF desktop manipulator with a three-finger soft gripper.
The robot photographs the workspace from its home position, locates coloured blocks by colour,
converts their pixel coordinates into its own coordinate frame, and sorts them into stacks by colour.

## Demo

Download the demo video [Watch the demo](pick_and_place_demo.mp4) — 67 seconds, unedited.
The green mat is the calibration grid; the four red blocks at known grid positions
are the reference markers.

## How it works

**Calibration.** The arm takes one frame from its home position. Four red blocks sit at known
positions on the grid, and their pixel centroids are paired with their known robot coordinates to
build a perspective transform with `cv2.getPerspectiveTransform`. Every block detected afterwards is
pushed through `cv2.perspectiveTransform` to get its position in millimetres in the robot's frame.
This replaces the usual approach of manually teaching each block position, so the layout can change
between runs without touching the code.

**Detection.** Blocks are segmented in HSV. Blue and yellow each take a single hue range; red needs
two, because red wraps the 0/180 boundary in OpenCV's hue scale. Contours below a minimum area are
discarded as noise, and each surviving contour gives a centroid from its bounding box.

**Motion and stacking.** The camera gives x and y but no depth, so the picking height `z_pick` is
fixed and measured once. The arm travels at a higher `z_hover` so it never drags a held block through
the others, descends to pick, and rises again before moving. Same-colour blocks stack at a fixed
per-layer increment above the first, so each stack grows upward without the gripper colliding with
the block below.

**Control.** The arm is driven over a serial link using [pydobot](https://github.com/luismesas/pydobot).
The vendor DobotEDU/MagicBox SDK was tried first and abandoned: it targets an older Python than the
project used. Gripper open and close commands are followed by short settling delays, and the pneumatic
gripper is shut down on exit with a raw protocol message rather than left powered.

## Hardware and dependencies

- Dobot Magician Lite with a SoftGripping three-finger soft gripper
- A webcam positioned to see the full workspace
- Python with `opencv-python`, `numpy`, `pydobot`, `pyserial`

Set the serial port and the four reference-marker coordinates at the top of the script to match your
own setup before running.

## Attribution

This was a five-person group coursework project for **EG554V Kinematics and Dynamics of Industrial
Robot Arms**, University of Aberdeen, March 2026.

**My contribution**, and the content of this repository:

- the four-marker calibration scheme and the pixel-to-robot perspective transform
- colour-based block detection in OpenCV
- the hover, pick and stack height logic
- serial control of the arm via pydobot

**Not my work**, and not included here: the Denavit-Hartenberg table, the forward and inverse
kinematics, the Jacobian and singularity analysis, and the MATLAB simulation GUI. Those were done by
other members of the group and are theirs to publish.

The video is a recording of the whole group's demonstration and is included with that in mind.

## Licence

MIT, covering the code in this repository.
