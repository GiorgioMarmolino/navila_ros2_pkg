# navila_ros2_bridge

ROS 2 (Humble) bridge between the **NaVILA** Vision-Language-Action model and a wheeled mobile robot
(Clearpath Husky), enabling autonomous navigation driven by natural-language instructions.

The package takes a text instruction (e.g. *"go to the kitchen"*), queries NaVILA over a sequence of
camera observations, and translates the model's decisions into velocity commands executed in
**closed-loop** on odometry, with an optional anti-collision safety layer.

The implementation aims for **maximum fidelity** to the official
[`AnjieCheng/NaVILA`](https://github.com/AnjieCheng/NaVILA) inference, adapted for execution on a real
robot (continuous control, waiting for physical completion of each primitive, obstacle handling).

---

## Table of Contents

- [Architecture](#architecture)
- [Model](#model)
- [Nodes](#nodes)
- [Topics](#topics)
- [Parameters](#parameters)
- [Installation](#installation)
- [Usage](#usage)
- [Fidelity to the Official Repo](#fidelity-to-the-official-repo)
- [Troubleshooting](#troubleshooting)

---

## Architecture

```
                    /goal_instruction (String)
                              │
                              ▼
   /camera .../compressed ─► [ navila_node ] ─► /navila/action ─► [ action_node ]
                              ▲                  (String:                  │
                              │                   "forward 25 cm")         │ closed-loop
                              │                                            │ on /odom
              /navila/primitive_status ◄──────────────────────────────────┤
                  (String: done|aborted)                                   │
                                                                           ▼
                                                                  /cmd_vel_raw (Twist)
                                                                           │
                                                                           ▼
                                                              [ safety_layer_node ]
                                                                  (anti-collision)
                                                                           │
                                                                           ▼
                                                                  /cmd_vel ─► twist_mux
```

The system is an **event-driven loop synchronized with motion**, not a fixed-rate controller: one
model decision → execute one primitive → observe the result → next decision. This replicates the
step-synchronous nature of the NaVILA policy.

When the safety layer is disabled, `action_node` publishes directly to `/cmd_vel`.

---

## Model

- **Checkpoint:** `a8cheng/navila-llama3-8b-8f` (downloaded automatically from HuggingFace on first
  launch if not present in `model_path`).
- **Backbone:** SigLIP vision encoder + LLaMA3-8B.
- **Input:** a sequence of `num_video_frames` frames (default 8: 7 historical + 1 current).
- **Output:** free-form text containing the action and its magnitude, e.g. *"the next action is to
  move forward 75 cm"* or *"turn left 30 degrees"*.

The model is already trained and ready for inference: **no training is required**. The datasets
referenced in the official repo are only needed to reproduce or extend training.

---

## Nodes

### `navila_node`
The core of the package. Loads NaVILA, manages the frame history, runs inference, and implements the
decision loop and the action queue.

- Loads the model in a background thread (ROS spin is not blocked during loading).
- Maintains an observation history that advances **one frame per executed primitive**.
- Parses the model output, extracting action, value and unit; quantizes to the official primitives
  (forward in multiples of 25 cm, turn in multiples of 15°).
- Expands the magnitude into a queue of primitives: executes one, queues the rest, and **replays them
  without re-running inference** until the queue is empty (replicating `queue_actions`).
- Advances when it receives the primitive-completion status from `action_node`.

### `action_node`
Translates primitives into velocity commands and executes each motion in **closed-loop on odometry**.

- Receives commands in the format `"<action> <value> <unit>"` (e.g. `forward 25 cm`, `turn_left 15 deg`).
- Saves the start pose from `/odom`, measures actual progress (Euclidean distance for forward,
  normalized delta-yaw for turn) and stops when the target is reached.
- Command smoothing via acceleration ramp; anti-overshoot margin on the target.
- Failsafe: if progress is not reached within a deadline, the primitive is aborted.
- Publishes `/navila/primitive_status` with payload `done` (completed by measurement) or `aborted`
  (completed by failsafe / robot blocked).

### `safety_layer_node`
Anti-collision velocity filter between `action_node` and `twist_mux`. Optional (toggled from the launch file).

- Sectorizes the LiDAR (Velodyne VLP-16 via `pointcloud_to_laserscan`) into front / sides / rear and
  computes the minimum distance per sector.
- Hard front stop + progressive slowdown; rear protection while reversing; steering reduction in
  narrow corridors.
- Optional fusion with the ZED depth for low obstacles ahead.
- Fail-safe on LiDAR timeout and a watchdog on the incoming command.

---

## Topics

### `navila_node`
| Direction | Topic | Type | Notes |
|---|---|---|---|
| Sub | `/zed/rgb/color/rect/image/compressed` | `sensor_msgs/CompressedImage` | camera observations |
| Sub | `/goal_instruction` | `std_msgs/String` | navigation instruction (arms the loop) |
| Sub | `/navila/reset` | `std_msgs/Empty` | disarms the loop and clears the history |
| Sub | `/navila/primitive_status` | `std_msgs/String` | `done` / `aborted` from `action_node` |
| Pub | `/navila/action` | `std_msgs/String` | primitive, e.g. `"forward 25 cm"` |

### `action_node`
| Direction | Topic | Type | Notes |
|---|---|---|---|
| Sub | `/navila/action` | `std_msgs/String` | primitive to execute |
| Sub | `/odom` (or `/odometry/filtered`) | `nav_msgs/Odometry` | pose for closed-loop |
| Pub | `/cmd_vel_raw` or `/cmd_vel` | `geometry_msgs/Twist` | velocity command |
| Pub | `/navila/primitive_status` | `std_msgs/String` | `done` / `aborted` |

### `safety_layer_node`
| Direction | Topic | Type | Notes |
|---|---|---|---|
| Sub | `/cmd_vel_raw` | `geometry_msgs/Twist` | raw command |
| Sub | `/scan` | `sensor_msgs/LaserScan` | 2D LiDAR |
| Sub | `/zed/depth/depth_registered` | `sensor_msgs/Image` | depth (optional) |
| Pub | `/cmd_vel` | `geometry_msgs/Twist` | filtered command toward twist_mux |

---

## Parameters

Centralized in `navila_params.yaml`. The main ones:

### `navila_node`
| Parameter | Default | Description |
|---|---|---|
| `model_path` | `/models` | local path of the NaVILA checkpoint |
| `num_video_frames` | `8` | frames per inference (overridden by the checkpoint if present) |
| `max_history_frames` | `512` | maximum history depth |
| `image_topic` | `/zed/rgb/color/rect/image/compressed` | input camera topic |
| `input_color_order` | `bgr` | channel order after `imdecode`: `bgr` → convert to RGB, `rgb` → leave unchanged |
| `frame_wait_timeout_sec` | `1.0` | max wait for a fresh frame after motion |
| `frame_settle_sec` | `0.0` | settling margin for robot/camera before grabbing the frame |

### `action_node`
| Parameter | Default | Description |
|---|---|---|
| `cmd_vel_topic` | `/cmd_vel` | output (`/cmd_vel_raw` if safety is active) |
| `odom_topic` | `/odometry/filtered` | odometry source for closed-loop |
| `linear_x` | `0.4` | forward linear velocity (m/s) |
| `angular_z` | `0.35` | turn angular velocity (rad/s) |
| `max_acc_linear` / `max_acc_angular` | `1.0` / `2.0` | acceleration ramp limits |

### `safety_layer_node`
| Parameter | Default | Description |
|---|---|---|
| `enable_depth` | `true` | enable fusion with the ZED depth |
| `front_stop_dist` / `front_slow_dist` | — | **requires `stop < slow`** (see Troubleshooting) |
| `front_fov_deg` / `side_fov_deg` / `rear_fov_deg` | `60` / `15` / `10` | sector widths |
| `lidar_front_angle_deg` | `90` | offset of the robot's physical front in the scan array |

---

## Installation

Requirements: ROS 2 Humble, a CUDA GPU with NaVILA installed (for Jetson AGX Orin use the
torch/flash-attn wheels from the Jetson AI Lab index; for x86 use the standard wheels).

```bash
cd ~/ros_ws/src
git clone <repo-url> navila_ros2_bridge
cd ~/ros_ws
colcon build --packages-select navila_ros2_bridge
source install/setup.bash
```

The NaVILA checkpoint is downloaded on first launch if missing from `model_path`. Alternatively,
download it manually:

```bash
huggingface-cli download a8cheng/navila-llama3-8b-8f --local-dir /models
```

---

## Usage

### Launch the pipeline

```bash
# With safety layer
ros2 launch navila_ros2_bridge navila_bringup.launch.py enable_safety:=true

# Without safety (recommended for the first isolated tests)
ros2 launch navila_ros2_bridge navila_bringup.launch.py enable_safety:=false
```

Wait for the `NaVILA ready` log before proceeding.

### Send a goal

```bash
ros2 topic pub --once /goal_instruction std_msgs/msg/String "{data: 'go to the kitchen'}"
```

Use `--once`: every message resets the history and re-arms the loop, so repeated publishing would
prevent it from starting.

### Reset

```bash
ros2 topic pub --once /navila/reset std_msgs/msg/Empty "{}"
```

### Monitoring

```bash
ros2 topic echo /navila/action          --qos-reliability reliable
ros2 topic echo /navila/primitive_status --qos-reliability reliable
```

Debug frames with an overlay (action, raw output, goal) are saved to `/home/ros_ws/debug_frames`:
useful to verify what the model sees and decides.

### Expected log pattern

```
raw='...75 cm' → forward 25 cm ×3   (queued:2)
[queue] forward 25 cm   (remaining:1)
[queue] forward 25 cm   (remaining:0)
raw='...' → <new decision>
```

---

## Fidelity to the Official Repo

| Aspect | Official | This package |
|---|---|---|
| Generation | greedy, 32 tokens, tensor passed directly | identical |
| Prompt | `llama_3` template, 7 historical + 1 current | identical |
| Action parsing | patterns + magnitude + quantization | identical |
| Image preprocessing | `process_images` with the checkpoint config | identical (frames passed raw) |
| Frame sampling | `sample_and_pad_images` (black padding, `endpoint=False`, int) | faithful replica |
| Loop | step-synchronous, 1 primitive per step | event-driven gated by real motion |
| Action queue | `queue_actions`, replay without re-inference | faithful replica |
| History | 1 frame per executed primitive | identical (gated by `done`/`aborted`) |
| Execution | instantaneous `envs.step` (simulation) | closed-loop on odometry + failsafe (real) |

**Intentional difference:** on the real robot each primitive waits for physical completion before
proceeding, instead of the simulator's instantaneous step. The action sequence seen by the model
remains identical.

---

## Troubleshooting

**The robot doesn't move / always receives `stop`.**
Check the `raw='...'` output in the logs and the parser patterns: they must match what NaVILA
actually emits. Verify how often parsing falls back to the default — if always, it's a regex issue,
not the model.

**Wrong colors / poor inference.**
Open a frame in `/home/ros_ws/debug_frames`: if red and blue are swapped, the `input_color_order`
value is wrong for that source. The real camera and the simulation pipeline may require different
values; tune it by looking at the debug frame. Note: `cv2.imdecode` always returns BGR regardless of
the encoding declared on the topic.

**Model output identical on every cycle.**
Indicates a history that isn't changing: check frame sampling and the promotion of
`_last_decision_frame` into the history.

**The loop stops after the first decision (a single `raw=` then silence).**
`action_node` is not publishing `/navila/primitive_status`: without that signal the NaVILA node stays
waiting. Verify that `action_node` is running and that odometry is being received.

**Spurious `done` when the robot is blocked (wheels spinning in place).**
Encoder-only odometry integrates the slip and believes the robot is advancing. Use the IMU-fused pose
(`/odometry/filtered` from `robot_localization`) as `odom_topic`.

**The safety layer doesn't slow down progressively.**
Make sure `front_stop_dist < front_slow_dist`: if inverted, the progressive slowdown never triggers
and the scaling has a negative denominator.

**`ros2 topic echo` / `info -v` fail with `unknown tag 'rclpy.type_hash.TypeHash'`.**
Version mismatch between `ros2cli` and the ROS core in the container (does not affect runtime).
Workaround: add `--qos-reliability reliable`. Fix: realign the `ros-humble-*` packages.

**Mediocre navigation decisions in Gazebo.**
Visual domain gap: NaVILA is trained on photorealistic scenes (Habitat/Matterport), very different
from the simulator's rendering. Evaluate *pipeline* correctness (loop, queue, primitive execution)
separately from the *navigational quality* of the decisions.