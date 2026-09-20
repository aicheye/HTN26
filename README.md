# Spidey

A self-navigating quadruped on a tabletop arena. Tell it where to go, by voice, by WASD, or by clicking the map, and it plans the route and walks it. When walking can't get it there, a robot arm picks it up and carries it across.

Built at Hack The North 2026.

## Inspiration

Small robots are great at looking clever and bad at being useful. A tiny quadruped can wave and dance, but ask it to cross a table and it stumbles, drifts and walks into things. It has no idea where it is.

We wanted to see how far a cheap robot could get with **help from its surroundings**: an overhead camera for eyes, a laptop for a brain, and a robot arm for the moments walking isn't enough. And we wanted to control it the way you'd talk to a friend.

## What it does

- **It knows where it is.** An overhead camera tracks the robot and the arena with tags, so its position and heading are known in real time.
- **It plans and adapts.** It routes around obstacles, recovers when it gets stuck, and corrects for drift.
- **It calls for backup.** When there is no walkable path, such as a solid barrier across the table, a robot arm picks the robot up and carries it across.
- **You can just talk to it.** Say *"go past the blue barrier and then wave"*, *"turn right and walk 5 steps"* or *"go to the top left corner"*. A speech model turns that into a checked, step-by-step plan and the robot carries it out.
- **It shows you what it sees.** A live 2D and 3D map shows the robot, detected objects, the arm and the planned path. Manual driving and poses are there too, as extras.

## How we built it

**Robot.** A Sesame quadruped with an ESP32-S2 and PlatformIO firmware. It has four gaits (forward, back, left, right) and a library of poses, all controlled over WebSocket.

**Eyes.** A Raspberry Pi running QNX with a Camera Module 3. A C++ tracker finds ArUco tags on the robot, the arm and the four table corners, solves the camera pose every frame, and streams the robot's position at about 30 frames a second.

**Brain.** A Node.js bridge that:
- filters the robot's pose with a Kalman filter,
- plans routes with Dijkstra's on a 2 cm grid,
- drives the gaits with hysteresis, so the robot doesn't waste time switching between turning and walking,
- detects when the robot is stuck and backs off,
- measures the robot's real walking and turning speeds with the camera.

**Arm.** An SO-101 arm with a closed-form inverse-kinematics solver that we checked against forward kinematics, plus hard safety limits on every move. It works out how to grip the robot from the tag geometry. After each grip, the camera checks whether the robot moved with the gripper. If not, the arm shifts and retries.

**Object detection.** MobileSAM on the CPU, combined with colour and parallax cues from a moving camera, finds objects on the table. Their outlines and textures go to the map as obstacles.

**Interface.** A React and three.js app with 2D and 3D maps, telemetry, and a built-in simulator so we could develop and test with no hardware.

**Voice.** Whisper for transcription and a language model that returns structured JSON steps, both through Groq. Every step is validated before anything moves. Walks and turns are measured against the live camera position rather than a timer.

## Challenges we ran into

- **The robot has no sense of its own movement.** Its gaits are open-loop, so it drifts, veers and lags by about a third of a second. Everything had to be closed with camera feedback.
- **Noisy tracking.** A tag's apparent size gave a height that jumped by about 2 cm frame to frame, so we measured the robot's height once and intersected the camera ray with that plane. We also handled left-handed frames and a Pi with no clock.
- **A tiny arena.** The table is about 63 cm across. With the robot's clearance and a wall margin, two small obstacles can leave almost no corridor. A goal near a wall could be in bounds but unreachable, which sent the robot walking in loops until we added a wall margin and a no-progress watchdog.
- **Making the arm reach.** Held upright, the arm can't reach much above 12 cm, so it tilts the gripper only as far as needed and checks the whole lift, carry and lower path before it moves.
- **Trusting a language model with a robot.** We treat its output as untrusted. It must return strict JSON, the app refuses to move if tracking is stale, and it re-checks that the robot stays on the table and clear of obstacles. To get it to understand "the blue barrier" or "the corner on my left", we had to give it colours, sizes and each corner's position.
- **Flaky hardware and networks.** The servo bus dropped the odd packet, the Pi's `.local` name didn't resolve on every network, and four people built parts of one system on separate branches.

## Accomplishments that we're proud of

- A full loop across very different systems: a spoken sentence becomes a plan, the camera guides the robot along it, and when walking fails the arm steps in.
- A robot that gets itself out of trouble. It recovers from being stuck, fails cleanly instead of looping, and hands a blocked route to the arm.
- A hardware-free simulator and a large set of automated tests across the planner, navigator, voice pipeline and arm maths, so we could move fast without breaking the demo.
- Voice that does what people actually say: multi-step commands, distances in steps or centimetres, turns in degrees, and corners from your point of view. It also remembers the question it just asked, so a spoken follow-up makes sense.

## What we learned

- The hard part of robotics isn't the algorithm, it's the gap between the model and the hardware. Real robots drift, lag and lie, so measure and close the loop.
- Coordinate frames will hurt you. The camera, floor, arm and map each had their own conventions, and one flipped axis sends the robot the wrong way.
- Language models are good at understanding messy intent but shouldn't be the safety layer. Let them interpret, then verify everything in code.
- A simulator and tests are what let four people integrate quickly.

## What's next for Spidey

- Make the arm hand-off smoother and more reliable, and handle more than just carrying the robot.
- Support more than one robot and richer missions, like fetching or sorting.
- Add speech output, so the robot can ask for clarification and confirm what it's doing.
- Make it faster and more compact, with more of the processing on the robot or the Pi.

## Where things live

| Part | Where |
|---|---|
| Arm: IK solver, safety limits, tracker client, pick-up scripts | `so101_ik.py`, `so101_safe.py`, `sesame_tracker.py`, `sesame_pickup.py`; the one-command demo on the `arm` / `grasp` branches (`sh run.sh grip`) |
| Vision navigation (calibration, segmentation, planner, overlay) | `nav/` |
| Pi tracker, camera and object detection | `pi/`, `vision/` on `devel/sean` |
| Bridge, firmware, web app and voice | `bridge/`, `firmware/`, `frontend/` on their branches |
| Survey of the repo | `docs/INVENTORY.md` |

Arm quick start, from a fresh clone:

```
git clone -b grasp https://github.com/virkvarjun/HTN && cd HTN
sh run.sh test     # offline: IK tests and the pick-and-place chain, no hardware
sh run.sh grip     # live: IK to the Sesame's tag, grip, lift 10 cm, carry left, release, home
```
