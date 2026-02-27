# Flight Test 2 — Full Procedure Flowcharts

## 1. Overall Test Structure

```mermaid
flowchart TD
    START(["Flight Test #2<br/>Stationkeeping<br/>Worth 6% total"]) --> PART1

    subgraph PART1["Part I — VICON Aided (3%)"]
        P1_DESC["VICON motion capture provides<br/>ground-truth pose to PX4 EKF2<br/>via /mavros/vision_pose/pose"]
    end

    subgraph PART2["Part II — On-Board Sensing Only (3%)"]
        P2_DESC["No VICON after TEST command<br/>T265 VIO provides pose<br/>or optical flow / downward camera"]
    end

    PART1 --> PART2
    PART2 --> SCORING

    subgraph SCORING["Scoring (per part)"]
        direction TB
        EQ["S = (tp/τ + 0.5 × ts/τ) × 3%"]
        EQ --> ZONES
        ZONES["tp = time in primary zone<br/>ts = time in secondary zone<br/>τ = 30s total<br/>Outside both zones = 0 points"]
    end

    style START fill:#90EE90
    style SCORING fill:#87CEEB
```

## 2. Scoring Zones

```mermaid
flowchart TD
    POSE(["Drone pose at TEST command<br/>= reference (zero) point"]) --> CHECK

    CHECK{"Check each axis<br/>every frame"} --> H_CHECK
    CHECK --> POS_CHECK
    CHECK --> ALT_CHECK

    H_CHECK{"Heading drift?"} -->|"≤ ±5°"| PRIMARY
    H_CHECK -->|"±5° to ±10°"| SECONDARY
    H_CHECK -->|"> ±10°"| OUTSIDE

    POS_CHECK{"XY drift (radial)?"} -->|"≤ 15 cm"| PRIMARY
    POS_CHECK -->|"15 cm to 25 cm"| SECONDARY
    POS_CHECK -->|"> 25 cm"| OUTSIDE

    ALT_CHECK{"Altitude?"} -->|"50 cm ± 10 cm<br/>(40–60 cm)"| PRIMARY
    ALT_CHECK -->|"50 cm ± 20 cm<br/>(30–70 cm)"| SECONDARY
    ALT_CHECK -->|"Outside 30–70 cm"| OUTSIDE

    PRIMARY["PRIMARY ZONE<br/>Full points for this time"]
    SECONDARY["SECONDARY ZONE<br/>Half points for this time"]
    OUTSIDE["OUTSIDE<br/>Zero points for this time"]

    NOTE["ANY single axis violation<br/>pushes you to the worse zone.<br/>e.g., heading OK + XY in secondary<br/>= secondary overall"]

    style PRIMARY fill:#98FB98
    style SECONDARY fill:#FFD700
    style OUTSIDE fill:#FF6B6B
```

## 3. Pre-Flight Setup Procedure

```mermaid
flowchart TD
    START(["Pre-Flight Setup"]) --> HW

    subgraph HW["Hardware Assembly"]
        HW1["Mount 3D-printed chassis<br/>+ Jetson Nano to frame"] --> HW2
        HW2["Connect serial cable<br/>Nano ↔ Cube Orange+<br/>(USB, 921600 baud)"] --> HW3
        HW3["Install T265 tracking camera<br/>(required for Part II)"] --> HW4
        HW4["TA attaches VICON marker<br/>holder to drone (Part I)"]
    end

    HW --> PX4

    subgraph PX4["PX4 Configuration (QGroundControl)"]
        PX4_1["Set EKF2 to accept<br/>external vision pose"] --> PX4_2
        PX4_2["Configure COM_RCL_EXCEPT<br/>to allow OFFBOARD"] --> PX4_3
        PX4_3["Enable companion computer<br/>on serial port (MAV_1_CONFIG)"] --> PX4_4
        PX4_4["Verify parameters:<br/>COM_ARM_WO_GPS = enabled<br/>EKF2_AID_MASK or EKF2_EV_CTRL"]
    end

    PX4 --> SW

    subgraph SW["Software Setup on Jetson"]
        SW1["source /opt/ros/foxy/setup.bash"] --> SW2
        SW2["source mary_ws/install/setup.bash"] --> SW3
        SW3["Verify MAVROS connects<br/>ros2 topic echo /mavros/state"]
    end

    SW --> READY(["Ready for Flight Test"])

    style START fill:#90EE90
    style READY fill:#98FB98
```

## 4. Part I — Full Flight Procedure (VICON)

```mermaid
flowchart TD
    START(["Part I Begins<br/>Drone on ground in Drone Zone"]) --> LAUNCH_NODES

    LAUNCH_NODES["SSH into Jetson, launch nodes:<br/>ros2 launch mary_bringup<br/>flight_test_2.launch.py"] --> VERIFY

    VERIFY{"Verify systems nominal?<br/>- /mavros/state shows connected<br/>- VICON pose streaming<br/>- stationkeeping_node: IDLE"} -->|No| DEBUG["Debug connection issues"]
    DEBUG --> VERIFY
    VERIFY -->|Yes| SIGNAL_TA["Signal to TA: ready for test"]

    SIGNAL_TA --> GCS_LAUNCH

    subgraph GCS_LAUNCH["Step 1: Ascend to Altitude"]
        GL1["Ground control server sends:<br/>ros2 service call<br/>/rob498_drone_10/comm/launch<br/>std_srvs/srv/Trigger"] --> GL2
        GL2["stationkeeping_node receives LAUNCH<br/>Captures hover_pose:<br/>X = current X, Y = current Y<br/>Z = 0.5m, heading = current"] --> GL3
        GL3["Setpoints stream at 20Hz<br/>for 1.5 seconds"] --> GL4
        GL4["Request OFFBOARD mode<br/>(retry every 2s until accepted)"] --> GL5
        GL5["Request arming<br/>(retry every 2s until accepted)"] --> GL6
        GL6["Motors spin up<br/>Drone ascends to 0.5m<br/>Holds hover_pose"]
    end

    GCS_LAUNCH --> STABLE{"Drone roughly stable<br/>at hover?"}
    STABLE -->|Not yet| WAIT_STABLE["Wait for drone to settle"]
    WAIT_STABLE --> STABLE
    STABLE -->|Yes| SIGNAL_READY["Signal TA: stable hover achieved"]

    SIGNAL_READY --> GCS_TEST

    subgraph GCS_TEST["Step 2: Stationkeeping Test (30s)"]
        GT1["Ground control server sends:<br/>ros2 service call<br/>/rob498_drone_10/comm/test<br/>std_srvs/srv/Trigger"] --> GT2
        GT2["CLOCK STARTS<br/>TA begins recording VICON CSV"] --> GT3
        GT3["Drone holds hover_pose<br/>for 30 seconds<br/>Heading, XY, altitude scored"] --> GT4
        GT4["30 seconds elapse<br/>TA stops recording"]
    end

    GCS_TEST --> GCS_LAND

    subgraph GCS_LAND["Step 3: Descend and Land"]
        GD1["Ground control server sends:<br/>ros2 service call<br/>/rob498_drone_10/comm/land<br/>std_srvs/srv/Trigger"] --> GD2
        GD2["stationkeeping_node: LAND<br/>Z ramps down at 0.15 m/s<br/>XY + heading held constant"] --> GD3
        GD3{"Altitude < 0.12m?"}
        GD3 -->|Yes| GD4["Auto disarm<br/>State → IDLE"]
        GD3 -->|No| GD5{"Elapsed > 15s?"}
        GD5 -->|No| GD2
        GD5 -->|Yes| GD6["Force disarm (safety)"]
    end

    GCS_LAND --> MANUAL_DEMO

    subgraph MANUAL_DEMO["Step 4: Manual Landing Demo (required, not scored)"]
        MD1["TA asks: put drone back<br/>into low-altitude hover"] --> MD2
        MD2["Call /comm/launch again<br/>Drone ascends to 0.5m hover"] --> MD3
        MD3["Team member flips RC<br/>mode switch away from OFFBOARD<br/>(e.g., to POSITION or STABILIZED)"] --> MD4
        MD4["Node detects mode change:<br/>_offboard_achieved = True but<br/>mode != OFFBOARD<br/>→ Logs 'RC override detected'<br/>→ Resets to IDLE<br/>→ Stops requesting OFFBOARD"] --> MD5
        MD5["RC pilot has full control<br/>Node publishes current pose as<br/>setpoint (PX4 ignores in RC mode)"] --> MD6
        MD6["RC pilot lands manually<br/>using sticks"] --> MD7
        MD7["RC pilot disarms<br/>Node detects external disarm<br/>→ Confirms IDLE state"]
    end

    MANUAL_DEMO --> DONE(["Part I Complete"])

    style START fill:#90EE90
    style DONE fill:#98FB98
    style GD6 fill:#FF6B6B
    style MD4 fill:#87CEEB
```

## 5. Part II — Full Flight Procedure (T265 Only)

```mermaid
flowchart TD
    START(["Part II Begins<br/>Drone on ground in Drone Zone"]) --> LAUNCH_NODES

    LAUNCH_NODES["SSH into Jetson, launch nodes:<br/>ros2 launch mary_bringup<br/>flight_test_2.launch.py<br/>vicon_topic:="] --> NOTE

    NOTE["Key difference: vicon_topic is EMPTY<br/>T265 VIO is sole pose source<br/>Local frame only — no global reference"]

    NOTE --> VERIFY

    VERIFY{"Verify systems nominal?<br/>- /mavros/state connected<br/>- T265 pose on /mary/localization/pose<br/>- stationkeeping_node: IDLE"} -->|No| DEBUG["Debug connection issues"]
    DEBUG --> VERIFY
    VERIFY -->|Yes| POSITION

    POSITION["Position drone carefully<br/>T265 has no global frame —<br/>hover will be relative to<br/>current position"] --> SIGNAL_TA["Signal to TA: ready"]

    SIGNAL_TA --> GCS_LAUNCH

    subgraph GCS_LAUNCH["Step 1: Ascend to Altitude"]
        GL1["Ground control sends:<br/>/rob498_drone_10/comm/launch"] --> GL2
        GL2["hover_pose captured from T265:<br/>X ≈ 0, Y ≈ 0 (local frame)<br/>Z = 0.5m, heading = current"] --> GL3
        GL3["Setpoints stream 1.5s →<br/>OFFBOARD → Arm → Ascend"]
    end

    GCS_LAUNCH --> STABLE{"Drone roughly stable?"}
    STABLE -->|Not yet| WAIT["Wait for settle"]
    WAIT --> STABLE
    STABLE -->|Yes| SIGNAL_READY["Signal TA: stable hover"]

    SIGNAL_READY --> GCS_TEST

    subgraph GCS_TEST["Step 2: Stationkeeping Test (30s)"]
        GT1["Ground control sends:<br/>/rob498_drone_10/comm/test"] --> GT2
        GT2["CLOCK STARTS<br/>TA records VICON CSV<br/>(for scoring only — drone<br/>does NOT see VICON data)"] --> GT3
        GT3["Drone holds position using<br/>T265 VIO only for 30 seconds"] --> GT4
        GT4["Challenge: VIO drift over time<br/>No external correction available"]
    end

    GCS_TEST --> GCS_LAND

    subgraph GCS_LAND["Step 3: Descend and Land"]
        GL_1["Ground control sends:<br/>/rob498_drone_10/comm/land"] --> GL_2
        GL_2["Z ramps down at 0.15 m/s<br/>using T265 altitude estimate"] --> GL_3
        GL_3{"T265 altitude < 0.12m?"} -->|Yes| GL_4["Disarm → IDLE"]
        GL_3 -->|No| GL_5{"Elapsed > 15s?"}
        GL_5 -->|No| GL_2
        GL_5 -->|Yes| GL_6["Force disarm"]
    end

    GCS_LAND --> MANUAL_DEMO

    subgraph MANUAL_DEMO["Step 4: Manual Landing Demo"]
        MD1["Call /comm/launch again<br/>Drone hovers at 0.5m"] --> MD2
        MD2["RC pilot flips mode switch<br/>Node detects RC override<br/>→ Resets to IDLE gracefully"] --> MD3
        MD3["RC pilot lands manually<br/>+ disarms via RC"]
    end

    MANUAL_DEMO --> DONE(["Part II Complete"])

    style START fill:#90EE90
    style DONE fill:#98FB98
    style GL_6 fill:#FF6B6B
    style GT4 fill:#FFD700
```

## 6. Emergency Stops — ABORT and RC Override

```mermaid
flowchart TD
    subgraph SOFTWARE_ABORT["Option A: /comm/abort (software)"]
        A1(["Any state:<br/>IDLE, LAUNCH, TEST, LAND"]) --> A2
        A2["Ground control sends:<br/>/rob498_drone_10/comm/abort"] --> A3
        A3["Immediate motor disarm<br/>No descent ramp — just stops"] --> A4
        A4["Drone drops from current altitude<br/>(use only for safety emergencies)"] --> A5
        A5["State → ABORT<br/>Setpoints stop publishing<br/>_offboard_achieved reset"] --> A6
        A6["Can re-launch via<br/>/comm/launch"]
    end

    subgraph RC_OVERRIDE["Option B: RC Override (hardware)"]
        R1(["Drone flying in OFFBOARD<br/>(_offboard_achieved = True)"]) --> R2
        R2["RC pilot flips mode switch<br/>(e.g., to POSITION or STABILIZED)"] --> R3
        R3["PX4 exits OFFBOARD mode<br/>MAVROS reports new mode"] --> R4
        R4["stationkeeping_node detects:<br/>mode != OFFBOARD while<br/>_offboard_achieved = True"] --> R5
        R5["Node logs 'RC override detected'<br/>Resets to IDLE<br/>STOPS requesting OFFBOARD<br/>(does NOT fight the RC)"] --> R6
        R6["RC pilot has full manual control<br/>Lands using sticks, disarms"] --> R7
        R7["Node detects external disarm<br/>Confirms IDLE state<br/>Ready for re-launch"]
    end

    style A2 fill:#FF6B6B
    style A4 fill:#FF6B6B
    style R2 fill:#FFD700
    style R5 fill:#87CEEB
```

## 7. Complete Timeline — Part I vs Part II Side-by-Side

```mermaid
flowchart TD
    subgraph PART_I["Part I — VICON"]
        direction TB
        I1["Launch: flight_test_2.launch.py<br/>(default vicon_topic)"] --> I2
        I2["Pose source: VICON<br/>Accurate, drift-free"] --> I3
        I3["/comm/launch → hover at 0.5m<br/>VICON XY = global position"] --> I4
        I4["/comm/test → 30s scoring<br/>VICON pose keeps drone locked"] --> I5
        I5["/comm/land → descend<br/>VICON altitude for disarm check"] --> I6
        I6["Manual landing demo"]
    end

    subgraph PART_II["Part II — T265 Only"]
        direction TB
        II1["Launch: flight_test_2.launch.py<br/>vicon_topic:=  (empty)"] --> II2
        II2["Pose source: T265 VIO<br/>Local frame, may drift"] --> II3
        II3["/comm/launch → hover at 0.5m<br/>T265 XY ≈ (0,0) local origin"] --> II4
        II4["/comm/test → 30s scoring<br/>T265 alone — drift accumulates"] --> II5
        II5["/comm/land → descend<br/>T265 altitude for disarm check"] --> II6
        II6["Manual landing demo"]
    end

    subgraph DIFF["Key Differences"]
        direction TB
        D1["Launch argument:<br/>vicon_topic set vs empty"]
        D2["Pose accuracy:<br/>sub-mm (VICON) vs cm-level drift (T265)"]
        D3["Global vs local frame:<br/>VICON = room coords<br/>T265 = relative to power-on"]
        D4["Landing altitude check:<br/>VICON Z = above floor<br/>T265 Z = above start position"]
    end

    style PART_I fill:#87CEEB
    style PART_II fill:#FFD700
```

## 8. ROS2 Commands Reference

```mermaid
flowchart LR
    subgraph LAUNCH_CMDS["Launch Commands"]
        direction TB
        L1["Part I:<br/>ros2 launch mary_bringup<br/>flight_test_2.launch.py"]
        L2["Part II:<br/>ros2 launch mary_bringup<br/>flight_test_2.launch.py<br/>vicon_topic:="]
    end

    subgraph SERVICE_CMDS["Service Commands (GCS sends these)"]
        direction TB
        S1["ros2 service call<br/>/rob498_drone_10/comm/launch<br/>std_srvs/srv/Trigger"]
        S2["ros2 service call<br/>/rob498_drone_10/comm/test<br/>std_srvs/srv/Trigger"]
        S3["ros2 service call<br/>/rob498_drone_10/comm/land<br/>std_srvs/srv/Trigger"]
        S4["ros2 service call<br/>/rob498_drone_10/comm/abort<br/>std_srvs/srv/Trigger"]
    end

    subgraph DEBUG_CMDS["Debug / Monitor"]
        direction TB
        D1["ros2 topic echo<br/>/mavros/state"]
        D2["ros2 topic echo<br/>/mary/comm/flight_state"]
        D3["ros2 topic echo<br/>/mavros/vision_pose/pose"]
        D4["ros2 topic hz<br/>/mavros/setpoint_position/local"]
    end
```

## Quick Reference — Test Parameters

| Parameter | Primary Zone | Secondary Zone |
|-----------|-------------|----------------|
| Heading | ±5° | ±10° |
| XY position (radial) | ≤ 15 cm | ≤ 25 cm |
| Altitude | 50 cm ± 10 cm | 50 cm ± 20 cm |
| Test duration | 30 seconds | 30 seconds |
| Scoring | `tp/τ × 3%` | `0.5 × ts/τ × 3%` |

| System Parameter | Value |
|-----------------|-------|
| Takeoff altitude | 0.5 m |
| Descent speed | 0.15 m/s |
| Disarm altitude | 0.12 m |
| Landing timeout | 15 s |
| Setpoint rate | 20 Hz |
| Vision pose rate | 30 Hz |
| OFFBOARD wait | 1.5 s |
| Max test sessions | 3 attempts |
