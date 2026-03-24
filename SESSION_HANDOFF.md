# MARY — Session Handoff Document

**Team:** Skynet (ROB498 Capstone, Winter 2026)
**Date:** March 24, 2026
**Current Branch:** `T265_disp`

---

## Project Overview

MARY (Mobile Autonomous Rain sYstem) is an autonomous umbrella drone that follows a user overhead. The ROS2 workspace (`mary_ws/src/`) contains four packages:

| Package | Purpose |
|---------|---------|
| `mary_bringup` | Launch files |
| `mary_control` | Flight control nodes (stationkeeping, waypoint, follower) |
| `mary_perception` | T265 pose processing, stereo depth/tracking |
| `mary_hardware` | MAVROS launch, PX4 config |

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

## Flight Test 3 — COMPLETED (confirmed working 2026-03-16)

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

Skeleton cleanup done 2026-03-16. Removed unused artifacts: `mary_msgs` package, `comm_node.py`, `mission_manager_node.py`, `sensors.launch.py`, `mary_full.launch.py`, `mary_params.yaml`, `person_tracker_node.py`, `altitude_controller_node.py`.

### Implemented Nodes

- **`stereo_depth_node.py`** (mary_perception) — **CONFIRMED WORKING (2026-03-16).** Standalone VPI stereo depth publisher. Subscribes to T265 fisheye stereo pair via ROS topics, computes disparity via `libstereo.so` (VPI/CUDA on Jetson GPU). Publishes `/mary/depth/disparity` (mono16 Q10.5), `/mary/depth/depth` (32FC1 metres), `/mary/depth/debug` (colorized TURBO). Used for debugging/testing, not in the demo pipeline.

- **`stereo_tracker_node.py`** (mary_perception) — **CONFIRMED WORKING (2026-03-24).** Combines VPI GPU stereo disparity with blob detection for tracking an object (RC car / person) below the drone. Subscribes to T265 fisheye stereo pair, computes disparity via VPI/CUDA, performs foreground detection (ground plane estimation via median depth, foreground = pixels closer than ground by `height_threshold`), finds largest contour, computes 3D body-frame offset via centroid+depth, applies EMA filter. Publishes `/mary/tracking/target` (PointStamped, body frame), `/mary/tracking/status` (TRACKING/LOST), `/mary/tracking/debug` (colorized depth + overlay with tracking status, flight state, FPS, body offset, pose, setpoint). Features:
  - **Performance optimization:** CPU downsample 848x800 → 424x400 before rectification (4x cheaper remap), VPI output at 212x200. Achieves ~7-8 FPS on Jetson Nano.
  - **Video recording:** `record_video:=true` parameter saves debug frames to `logs/tracking_YYYYMMDD_HHMMSS.mkv` using H264 codec at actual measured FPS. Auto-prunes to keep only 3 most recent videos.
  - **Debug overlay:** Shows tracking status, flight state, FPS, body-frame offsets (fwd/rgt/dwn), current pose, current setpoint, pixel offset, arrow from image center to detection.
  - Axis signs configurable for T265 mounting calibration.

- **`follower_node.py`** (mary_control) — **BENCH-TESTED (2026-03-24), NOT FLIGHT-TESTED.** Rewritten from scratch based on `waypoint_node.py` patterns. State machine: IDLE → LAUNCH → FOLLOW ↔ HOVER → LAND | ABORT. Subscribes to `/mary/tracking/target` and `/mary/tracking/status`. Converts body-frame tracking offset to world-frame setpoint (no yaw rotation — drone flies with fixed heading, T265-to-ENU is identity). Negates offset so drone moves toward target. Rate-limits horizontal movement (1.5 m/s). On tracking loss, enters HOVER holding last known target; auto-lands after configurable timeout (default 60s in launch, 3s default param). Features:
  - **Pose source handoff (VICON → T265):** In part=1, takeoff uses VICON for reliable position. Once airborne and stable, hands off to T265 as primary pose source (Z offset calibrated from VICON). Two modes:
    - **Auto handoff** (`auto_handoff:=true`): After T265 Z offset is calibrated and drone has been in stable OFFBOARD hover for `handoff_stable_secs` (default 3s), automatically switches to T265.
    - **Manual handoff:** Call `ros2 service call /rob498_drone_10/comm/switch_pose std_srvs/srv/Trigger` from a terminal.
    - After handoff, VICON is kept as emergency fallback only (used if T265 pose drops out). Pose source resets to VICON on `_reset_to_idle`.
  - **`debug_follow` parameter:** Skips LAUNCH/arming, goes directly to FOLLOW for bench testing without MAVROS.
  - **`launch_t265` parameter** in `mary_demo.launch.py`: Set to `false` to launch without T265 driver (for VICON-only takeoff, start T265 manually later).
  - **T265 Z auto-offset:** When VICON is active and T265 pose first arrives, automatically computes Z offset to align T265 odometry with VICON world frame.
  - Same MAVROS integration as waypoint_node: vision pose relay, 20Hz setpoints, RC override detection, course service interface.

- **`mary_demo.launch.py`** (mary_bringup) — Launch file for full person-following demo. Launches MAVROS + (optional) T265 + t265_pose_node + stereo_tracker_node + follower_node. Supports `part:=1` (Vicon) and `part:=2` (T265 only). `launch_t265:=false` to defer T265 startup. `auto_handoff:=true` for automatic VICON→T265 handoff after stable hover.

### Problems Encountered & Solutions (Stereo Depth / Tracking)

**tf2 static transforms broken in Foxy:** The `TransformListener` buffer never receives static transforms from the realsense driver. `tf2_echo` works from CLI but the node's buffer always returns identity (baseline=0.0mm). Solution: hardcode T265 extrinsics from pyrealsense2 EEPROM (via `scripts/print_calibration.py`).

**tf2 extrinsics in wrong frame:** Even when tf2 worked, it reported extrinsics in optical frame convention (T=[0.064, -0.001, -0.001]) which differs from pyrealsense2 sensor frame (T=[-0.0643, 0.00007, -0.00015]). Using tf2 values produced a smooth gradient instead of real disparity. Solution: use pyrealsense2 values directly. Hardcoded in both `stereo_depth_node.py` and `stereo_tracker_node.py`.

**T265 Z published negative:** The `t265_pose_node` could publish negative Z values. Added `max(0.0, z)` clamp before publishing.

**Setpoint direction flipped:** Body-frame offset from tracker pointed drone→person, but follower added it to position (moving away from target). Fix: negate dx_body and dy_body when computing person world position.

**debug_follow race condition:** With `debug_follow:=true`, follower started in FOLLOW but `tracking_status` defaulted to 'LOST', causing immediate transition to HOVER. Fix: only transition FOLLOW→HOVER after at least one TRACKING status has been received (skip if `_ever_tracked` is False).

**QoS mismatch for rosbag recording:** T265 fisheye topics use BEST_EFFORT QoS, but `ros2 bag record` defaults to RELIABLE. Result: only ~10 frames captured in 45s. Fix: use QoS override file:
```bash
echo '/camera/fisheye1/image_raw:
  reliability: best_effort
/camera/fisheye2/image_raw:
  reliability: best_effort
/camera/fisheye1/camera_info:
  reliability: best_effort
/camera/fisheye2/camera_info:
  reliability: best_effort' > /tmp/qos.yaml

ros2 bag record --qos-profile-overrides-path /tmp/qos.yaml /camera/fisheye1/image_raw /camera/fisheye2/image_raw /camera/fisheye1/camera_info /camera/fisheye2/camera_info /mary/tracking/target /mary/tracking/status /mary/tracking/debug
```

**Video recording codec:** `h264_nvmpi` not available in system ffmpeg, `h264_omx` missing OMX libraries. Fell back to `cv2.VideoWriter` with H264 fourcc which works on Jetson's OpenCV build.

### Key Topics (Person Following Pipeline)

```
T265 fisheye L/R -> stereo_tracker_node -> /mary/tracking/target (PointStamped)
                         |                         |
                         +-> /mary/tracking/debug   v
                         +-> /mary/tracking/status  follower_node -> /mavros/setpoint_position/local
                                                         |
T265 pose -> t265_pose_node -> /mary/localization/pose --+
                                                         |
                                                         +-> /mavros/vision_pose/pose
```

### Bench Test Commands (no MAVROS/flight)

```bash
# Terminal 1: T265 camera
ros2 launch realsense2_camera rs_launch.py device_type:=t265 enable_fisheye1:=true enable_fisheye2:=true enable_pose:=true

# Terminal 2: T265 pose node
ros2 run mary_perception t265_pose_node --ros-args -p publish_to_mavros:=false

# Terminal 3: Stereo tracker (with video recording)
ros2 run mary_perception stereo_tracker_node --ros-args -p record_video:=true

# Terminal 4: Follower (debug mode, no MAVROS needed)
ros2 run mary_control follower_node --ros-args -p vicon_topic:="''" -p debug_follow:=true

# Terminal 5: View debug image
ros2 run rqt_image_view rqt_image_view /mary/tracking/debug

# Terminal 6: Watch setpoints
ros2 topic echo /mavros/setpoint_position/local
```

### Next Steps

1. **Flight test** the tracking + following pipeline (VICON takeoff → start T265 → enable following)
2. **Tune parameters** for flight conditions: `depth_min`/`depth_max`, `height_threshold`, `ema_alpha`, `min_blob_area` (may need lowering for 212x200 output)
3. **Calibrate axis signs** (`axis_sign_x`, `axis_sign_y`) during first flight — verify fwd/rgt directions match physical movement
4. **Test tracking recovery** — verify FOLLOW↔HOVER transitions work in flight
5. **Measure end-to-end latency** — target <330ms per proposal

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
│   │   └── launch/
│   │       ├── flight_test_2.launch.py    # FT2 launcher (working)
│   │       ├── flight_test_3.launch.py    # FT3 launcher (working)
│   │       └── mary_demo.launch.py        # Person following demo launcher
│   ├── mary_control/mary_control/
│   │   ├── stationkeeping_node.py         # FT2 controller (working)
│   │   ├── waypoint_node.py              # FT3 controller (working)
│   │   └── follower_node.py               # Person following controller
│   ├── mary_perception/mary_perception/
│   │   ├── t265_pose_node.py              # T265 VIO (working)
│   │   ├── stereo_tracker_node.py         # VPI stereo + blob tracking
│   │   ├── stereo_depth_node.py           # Standalone depth publisher (debug)
│   │   └── pose_logger_node.py            # Pose logging (working, optional)
│   └── mary_hardware/
│       ├── launch/
│       │   └── mavros.launch.py           # MAVROS FCU bridge
│       └── config/
│           ├── px4_config.yaml            # MAVROS parameters
│           └── px4_pluginlists.yaml       # MAVROS plugin config
├── scripts/                               # Utility scripts (realsense_test.py, VPI stereo, print_calibration.py)
├── logs/                                  # Auto-saved tracking debug videos (.mkv)
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
- **T265→ENU transform is identity** (180° Z rotation in t265_pose_node). Drone flies with fixed heading, so body→world yaw rotation is skipped in follower.
- **Hardcoded T265 extrinsics** — tf2 is broken in Foxy for this use case. Factory values from pyrealsense2 EEPROM hardcoded in stereo nodes.
- **Half-res remap optimization** — raw 848x800 downsampled to 424x400 before CPU rectification, VPI outputs at 212x200. Saves 4x remap cost.
- **`--symlink-install`** used for colcon build — Python changes take effect on node restart without rebuild.
