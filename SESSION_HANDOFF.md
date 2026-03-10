# MARY — Session Handoff Document

**Team:** Skynet (ROB498 Capstone, Winter 2026)
**Date:** March 9, 2026
**Current Branch:** `T265_disp`

---

## Project Overview

MARY (Mobile Autonomous Rain sYstem) is an autonomous umbrella drone that follows a user overhead. The ROS2 workspace (`mary_ws/src/`) contains five packages:

| Package | Purpose |
|---------|---------|
| `mary_bringup` | Launch files and parameter configs |
| `mary_control` | Flight control nodes (stationkeeping, comm, mission manager, follower, altitude controller) |
| `mary_perception` | T265 pose processing, person tracker (incomplete) |
| `mary_hardware` | MAVROS launch, sensor drivers, PX4 config |
| `mary_msgs` | Custom messages (MissionStatus, TrackingStatus, PersonDetection) |

---

## Flight Test 2 — COMPLETED

**Exercise:** Stationkeeping — hover at 50cm for 30 seconds.

### What Works

- **Stationkeeping node** (`stationkeeping_node.py`): Full state machine (IDLE → LAUNCH → TEST → LAND | ABORT) with autonomous hover at configurable altitude.
- **Course interface:** Services `{drone_id}/comm/{launch,test,land,abort}` using `std_srvs/Trigger`.
- **T265 VIO pipeline:** `t265_pose_node.py` processes T265 odometry, transforms to ENU frame (180° rotation about Z), publishes to `/mary/localization/pose` at 30Hz.
- **Vision pose relay:** Stationkeeping node relays best pose (Vicon if fresh, else T265) to `/mavros/vision_pose/pose` for PX4 EKF2 fusion at 30Hz.
- **Position setpoints:** Published at 20Hz to `/mavros/setpoint_position/local` to keep OFFBOARD alive.
- **VICON support (Part I):** Identity transform (Vicon already ENU-aligned). Height calibration computes T265 Z offset from paired Vicon/T265 samples.
- **T265-only (Part II):** Works without Vicon, initial pose zeroed on startup.
- **Landing:** Smooth descent ramping Z at 0.15 m/s while holding X/Y/heading.
- **RC override detection:** Resets to IDLE if pilot takes over or disarms externally.

### Launch Command

```bash
# Part I (Vicon-aided)
ros2 launch mary_bringup flight_test_2.launch.py part:=1

# Part II (T265 only)
ros2 launch mary_bringup flight_test_2.launch.py part:=2

# With calibrated T265 offset
ros2 launch mary_bringup flight_test_2.launch.py part:=2 t265_z_offset:=0.1234
```

### Nodes Launched (FT2)

1. **MAVROS** — FCU bridge via `/dev/ttyACM0:921600`
2. **T265 camera** — Pose at 200Hz, fisheye at 30fps
3. **T265 pose node** — VIO processing, `publish_to_mavros=False` (relay handled by stationkeeping)
4. **Stationkeeping node** — Hover controller + course interface

---

## Problems Encountered & Solutions

### T265 Frame Transformation (Multiple Iterations)

**Problem:** T265 mounted bottom-facing with lenses backward produced incorrect position/orientation in MAVROS. Drone drifted or flew in wrong direction.

**Solution history (see git log):**
1. `f7f74f1` — Initial NED transformation attempt (incorrect)
2. `17483c5` — Corrected to ENU frame
3. `fdc937d` — Final fix: 180° rotation about Z axis confirmed working

**Final mapping:**
```
ENU +X (East)  = -T265 X
ENU +Y (North) = -T265 Y
ENU +Z (Up)    = +T265 Z
```

Rotation matrix in `t265_pose_node.py`:
```python
self._R_t265_to_enu = np.array([[-1, 0, 0], [0, -1, 0], [0, 0, 1]])
```

### Vicon Frame Alignment

**Problem:** Initially assumed Vicon needed NED→ENU rotation.

**Solution (`fdc937d`):** Vicon frame is already aligned with ENU. Set to identity transform. Added height calibration service to compute T265-to-Vicon Z offset.

### PX4 OFFBOARD Setpoint Stream

**Problem:** PX4 exits OFFBOARD mode if setpoints arrive at < 2Hz. During state transitions, setpoint gaps caused mode drops.

**Solution:** Continuous 20Hz setpoint publishing in ALL states (IDLE/LAUNCH/TEST/LAND). Even during IDLE, streams current pose to keep the pipeline warm. Only ABORT stops publishing (drone is disarmed anyway).

### Vision Pose Relay Conflict

**Problem:** Both `t265_pose_node` and `stationkeeping_node` could publish to `/mavros/vision_pose/pose`, causing duplicate/conflicting poses to PX4 EKF2.

**Solution:** Set `publish_to_mavros=False` on `t265_pose_node` in FT2 launch. Stationkeeping node handles relay, choosing between Vicon (if fresh within 0.5s) and T265.

### Landing Behavior

**Problem:** PX4 `AUTO.LAND` mode was unreliable in some configurations.

**Solution (`97d42d5`):** Stationkeeping node uses manual landing — ramps Z setpoint toward ground at `descent_speed` (0.15 m/s) while holding X/Y/heading. Manual disarm required after touchdown.

---

## Flight Test 3 — IMPLEMENTED

**Exercise:** Waypoint navigation — fly through 4 waypoints (40cm radius spheres) in order within 60s (90s max). Two parts: Vicon-aided (4%) and T265-only (4%).

### What Was Built

- **`waypoint_node.py`** — New node extending the proven stationkeeping pattern with waypoint navigation. Same state machine (IDLE→LAUNCH→TEST→LAND|ABORT), same vision pose relay, same setpoint infrastructure. During TEST, sequences through waypoints instead of holding position.
- **`flight_test_3.launch.py`** — Launch file mirroring FT2 but with waypoint_node.
- **Entry point** registered in `mary_control/setup.py`.

### FT2 Bug Fix Applied

**Problem:** In FT2, the `stationkeeping_node` used `super().__init__('stationkeeping_node')` as the ROS node name. The course skeleton requires the node name to be the drone ID (e.g. `rob498_drone_10`). The TA's ground control server may discover services by node name.

**Fix:** `waypoint_node.py` uses `super().__init__('rob498_drone_10')` matching the course skeleton convention. The stationkeeping_node still has the old name (not fixed — only used for FT2 which is complete).

### How It Works

1. **Waypoints received** via `rob498_drone_10/comm/waypoints` (PoseArray) — stored as Nx3 numpy array. If >4 poses, first is treated as current Vicon position (for Part II frame alignment).
2. **LAUNCH** — captures hover pose (current X/Y + 0.5m Z), streams setpoints, requests OFFBOARD + arm.
3. **TEST** — computes Vicon→T265 frame offset (Part II only), starts waypoint sequencing. Position setpoints are updated to target each waypoint in order.
4. **Reach detection** at 10 Hz — drone enters 40cm sphere → advance to next waypoint. Distance logged at ~2 Hz.
5. **All reached** → holds position at last waypoint (can land early or wait for LAND command).
6. **LAND** — smooth descent at 0.15 m/s holding current X/Y.

### Part II Frame Alignment

Both Vicon and T265 (after t265_pose_node) are ENU-oriented. The only difference is origin. At TEST start:
```
offset = vicon_pos − t265_pos
waypoint_t265 = waypoint_vicon − offset
```
Uses Vicon ref from PoseArray if available, else last Vicon subscription pose.

### Launch Commands

```bash
# Part I (Vicon-aided)
ros2 launch mary_bringup flight_test_3.launch.py part:=1

# Part II (T265 only)
ros2 launch mary_bringup flight_test_3.launch.py part:=2

# With calibrated T265 offset
ros2 launch mary_bringup flight_test_3.launch.py part:=2 t265_z_offset:=0.1234
```

### Nodes Launched (FT3)

1. **MAVROS** — FCU bridge via `/dev/ttyACM0:921600`
2. **T265 camera** — Pose at 200Hz, fisheye at 30fps
3. **T265 pose node** — VIO processing, `publish_to_mavros=False`
4. **Waypoint node** — Navigation controller + course interface

---

## MARY System (Person Following) — Status

The full MARY pipeline exists but is **untested and outdated**:

- **`mission_manager_node.py`** — State machine for full mission (IDLE → PREFLIGHT → ARMING → TAKEOFF → ACQUIRING → FOLLOWING → LANDING). Uses 2.5m altitude, monitors person detection.
- **`follower_node.py`** — P-control centering person in camera frame + feedforward velocity. Publishes velocity commands at 20Hz. Max 2.0 m/s horizontal, 1.0 m/s vertical.
- **`altitude_controller_node.py`** — PID altitude hold (Kp=1.5, Ki=0.1, Kd=0.5). Min 1.0m, max 5.0m.
- **`person_tracker_node.py`** — **INCOMPLETE.** Has subscriber/publisher structure but all processing functions are TODO stubs. Intended to use YOLOv8n.
- **`comm_node.py`** — Ground control interface with mission manager integration. Has waypoint subscription for FT3.
- **`mary_full.launch.py`** — Full system launch (all nodes).

**Note:** These MARY nodes are designed for the final capstone demo, not for flight tests. They need significant work (especially person tracker) before they're usable.

---

## Hardware Configuration

| Component | Detail |
|-----------|--------|
| Flight Controller | CubePilot Orange Cube+ running PX4 |
| Companion Computer | Jetson Nano |
| Tracking Camera | Intel RealSense T265 (mounted bottom, lenses backward) |
| Forward Camera | Sony IMX219 (CSI-2, not used in flight tests) |
| Battery | 4S LiPo, 2300 mAh |
| FCU Connection | `/dev/ttyACM0:921600` (USB serial) |
| GCS URL | `udp://@100.66.197.133:14550` |
| Drone ID | `rob498_drone_10` |
| Vicon Topic | `/vicon/ROB498_Drone/ROB498_Drone` |

---

## Repository Structure

```
ROB498-SkyNet/
├── mary_ws/src/
│   ├── mary_bringup/
│   │   ├── launch/
│   │   │   ├── flight_test_2.launch.py    # FT2 launcher (working)
│   │   │   ├── flight_test_3.launch.py    # FT3 launcher (new)
│   │   │   └── mary_full.launch.py        # Full MARY launcher (untested)
│   │   └── config/
│   │       └── mary_params.yaml           # Central parameters
│   ├── mary_control/mary_control/
│   │   ├── stationkeeping_node.py         # FT2 controller (working)
│   │   ├── waypoint_node.py              # FT3 controller (new)
│   │   ├── comm_node.py                   # Ground control (has FT3 waypoint sub)
│   │   ├── mission_manager_node.py        # MARY mission FSM (untested)
│   │   ├── follower_node.py               # Person following (untested)
│   │   └── altitude_controller_node.py    # PID altitude (untested)
│   ├── mary_perception/mary_perception/
│   │   ├── t265_pose_node.py              # T265 VIO (working)
│   │   └── person_tracker_node.py         # Person detection (INCOMPLETE)
│   ├── mary_hardware/
│   │   ├── launch/
│   │   │   ├── mavros.launch.py           # MAVROS FCU bridge
│   │   │   └── sensors.launch.py          # T265 + IMX219 drivers
│   │   └── config/
│   │       ├── px4_config.yaml            # MAVROS parameters
│   │       └── px4_pluginlists.yaml       # MAVROS plugin config
│   └── mary_msgs/msg/
│       ├── MissionStatus.msg
│       ├── TrackingStatus.msg
│       └── PersonDetection.msg
├── scripts/                               # Utility scripts
├── docs/                                  # Documentation
└── venv/                                  # Python virtual environment
```

---

## Git Branches

- **`main`** — Stable base
- **`T265_disp`** — Current working branch (all FT2 development)

---

## Key Decisions & Conventions

- **ROS2 Foxy** (downgraded from Humble — see commit `95fff74`)
- **Python build system** (switched from CMake — see commit `b5b8d92`)
- **drone_id = `rob498_drone_10`** — Used for all course service naming
- **ROS node name = `rob498_drone_10`** — Must match drone_id per course skeleton (FT2 had this wrong)
- **Position setpoints** preferred over velocity for stability in flight tests
- **T265 `publish_to_mavros=False`** in flight test launches to avoid relay conflicts
- **Manual disarm** after landing — no auto-disarm to avoid accidental disarm in flight
