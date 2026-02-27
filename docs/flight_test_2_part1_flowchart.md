# Flight Test 2 Part I — Architecture & Algorithm Flowchart

## 1. System Architecture (Part I — VICON)

```mermaid
flowchart TB
    subgraph EXTERNAL["External Systems"]
        VICON["VICON Motion Capture<br/>/vicon/ROB498_Drone/ROB498_Drone<br/>(PoseStamped)"]
        GCS["Ground Control<br/>(ros2 service call)"]
    end

    subgraph HARDWARE["Hardware"]
        T265["Intel RealSense T265<br/>Pose: 200Hz | Fisheye: 30Hz"]
        FCU["Cube Orange+<br/>PX4 Autopilot"]
        MOTORS["4x ESC + Motors"]
    end

    subgraph JETSON["Jetson Nano — ROS2 Nodes"]
        subgraph PERCEPTION["Perception Layer"]
            T265_NODE["t265_pose_node<br/>30Hz pose processing"]
        end

        subgraph MAVROS_NODE["MAVROS"]
            MAVROS_BRIDGE["MAVROS Bridge<br/>MAVLink over USB<br/>921600 baud"]
        end

        subgraph CONTROL["Control Layer"]
            SK["stationkeeping_node"]
        end
    end

    T265 -->|"/camera/pose/sample<br/>(Odometry)"| T265_NODE
    T265_NODE -->|"/mary/localization/pose<br/>(PoseStamped)"| SK
    VICON -->|"/vicon/ROB498_Drone/...<br/>(PoseStamped)"| SK

    SK -->|"/mavros/vision_pose/pose<br/>(PoseStamped) 30Hz"| MAVROS_BRIDGE
    SK -->|"/mavros/setpoint_position/local<br/>(PoseStamped) 20Hz"| MAVROS_BRIDGE

    SK -->|"/mavros/set_mode<br/>(SetMode srv)"| MAVROS_BRIDGE
    SK -->|"/mavros/cmd/arming<br/>(CommandBool srv)"| MAVROS_BRIDGE

    MAVROS_BRIDGE -->|"/mavros/state<br/>(State)"| SK
    MAVROS_BRIDGE <-->|"MAVLink USB"| FCU
    FCU --> MOTORS

    GCS -->|"comm/launch<br/>comm/test<br/>comm/land<br/>comm/abort"| SK

    SK -->|"/mary/comm/flight_state<br/>(String)"| GCS
```

## 2. State Machine

```mermaid
stateDiagram-v2
    [*] --> IDLE

    IDLE --> LAUNCH : /comm/launch service called
    LAUNCH --> TEST : /comm/test service called
    TEST --> LAND : /comm/land service called
    LAUNCH --> LAND : /comm/land service called
    LAND --> IDLE : altitude < 0.12m (disarm) or timeout > 15s

    LAUNCH --> IDLE : RC override (mode != OFFBOARD)
    TEST --> IDLE : RC override (mode != OFFBOARD)
    LAND --> IDLE : RC override (mode != OFFBOARD)

    LAUNCH --> IDLE : External disarm detected
    TEST --> IDLE : External disarm detected
    LAND --> IDLE : External disarm detected

    IDLE --> ABORT : /comm/abort
    LAUNCH --> ABORT : /comm/abort
    TEST --> ABORT : /comm/abort
    LAND --> ABORT : /comm/abort
    ABORT --> IDLE : /comm/launch (re-launch)

    state IDLE {
        [*] --> Waiting
        Waiting : No setpoints published
        Waiting : Waiting for launch command
    }

    state LAUNCH {
        [*] --> StreamSetpoints
        StreamSetpoints : Publish hover_pose at 20Hz
        StreamSetpoints --> WaitOffboard : 1.5s elapsed
        WaitOffboard : Request OFFBOARD mode (retry every 2s)
        WaitOffboard --> WaitArm : mode == OFFBOARD
        WaitArm : Request arming (retry every 2s)
        WaitArm --> Hovering : armed == true
        Hovering : _offboard_achieved = True
        Hovering : Armed in OFFBOARD, holding position
    }

    state TEST {
        [*] --> Scoring
        Scoring : 30s scoring window
        Scoring : Continue holding same hover_pose
        Scoring : Reference position logged
    }

    state LAND {
        [*] --> Descending
        Descending : Z ramps down at 0.15 m/s
        Descending : X, Y, heading held constant
        Descending --> Disarm : altitude < 0.12m
        Descending --> ForceDisarm : timeout > 15s
    }

    note right of IDLE
        RC override only triggers after
        _offboard_achieved = True.
        Node will NOT fight the RC
        for control once pilot takes over.
    end note
```

## 3. Main Algorithm Flowchart

```mermaid
flowchart TD
    START(["Node Starts"]) --> INIT["Initialize:<br/>- flight_state = IDLE<br/>- hover_pose = None<br/>- Subscribe to VICON + T265 + MAVROS state<br/>- Create service servers<br/>- Start 3 timers"]

    INIT --> TIMERS

    subgraph TIMERS["Three Concurrent Timer Loops"]
        direction TB
        T1["Vision Pose Loop<br/>30Hz"]
        T2["Setpoint Loop<br/>20Hz"]
        T3["State Machine Loop<br/>10Hz"]
    end

    subgraph VP_LOOP["Vision Pose Loop (30Hz)"]
        VP1["Get VICON pose"] --> VP2{"VICON pose<br/>available?"}
        VP2 -->|Yes| VP3["Use VICON pose"]
        VP2 -->|No| VP4["Use T265 pose"]
        VP3 --> VP5["Publish to<br/>/mavros/vision_pose/pose"]
        VP4 --> VP5
        VP5 --> VP6(["PX4 knows where<br/>drone is"])
    end

    subgraph SP_LOOP["Setpoint Loop (20Hz)"]
        SP1{"flight_state?"} -->|IDLE / LAUNCH / TEST| SP2{"hover_pose<br/>set?"}
        SP2 -->|Yes| SP3["Publish hover_pose<br/>(fixed X, Y, Z=0.5m, heading)"]
        SP2 -->|No| SP4["Publish current pose<br/>(pre-launch stream)"]
        SP1 -->|LAND| SP5["Publish:<br/>X, Y, heading = hover_pose<br/>Z = start_alt - 0.15 * elapsed"]
        SP1 -->|ABORT| SP6["Stop publishing"]
        SP3 --> SP7["Publish to<br/>/mavros/setpoint_position/local"]
        SP4 --> SP7
        SP5 --> SP7
        SP7 --> SP8(["PX4 knows where<br/>to fly"])
    end

    subgraph SM_LOOP["State Machine Loop (10Hz)"]
        SM_RC{"_offboard_achieved<br/>AND flying state<br/>AND mode != OFFBOARD?"} -->|Yes| SM_RELEASE["RC override detected<br/>Reset to IDLE<br/>(stop fighting RC)"]
        SM_RC -->|No| SM_DISARM{"_offboard_achieved<br/>AND flying state<br/>AND not armed?"}
        SM_DISARM -->|Yes| SM_EXT_DISARM["External disarm detected<br/>Reset to IDLE"]
        SM_DISARM -->|No| SM1{"flight_state?"}
        SM1 -->|LAUNCH| SM_LAUNCH
        SM1 -->|LAND| SM_LAND
        SM1 -->|Other| SM_NOOP["No action"]

        subgraph SM_LAUNCH["_tick_launch"]
            L1{"elapsed<br/>< 1.5s?"} -->|Yes| L2["Wait<br/>(setpoints still streaming)"]
            L1 -->|No| L3{"mode ==<br/>OFFBOARD?"}
            L3 -->|No| L4["Request OFFBOARD<br/>(retry every 2s)"]
            L3 -->|Yes| L5{"armed?"}
            L5 -->|No| L6["Request arming<br/>(retry every 2s)"]
            L5 -->|Yes| L7["_offboard_achieved = True<br/>Log: Hovering,<br/>ready for TEST"]
        end

        subgraph SM_LAND["_tick_land"]
            D1["Read current altitude<br/>from VICON pose"] --> D2{"altitude<br/>< 0.12m?"}
            D2 -->|Yes| D3["Disarm<br/>Reset to IDLE"]
            D2 -->|No| D4{"elapsed<br/>> 15s?"}
            D4 -->|Yes| D5["Force disarm<br/>Reset to IDLE"]
            D4 -->|No| D6["Continue descending"]
        end
    end

    style START fill:#90EE90
    style VP6 fill:#87CEEB
    style SP8 fill:#87CEEB
    style L7 fill:#98FB98
    style D3 fill:#FFD700
    style D5 fill:#FF6B6B
    style SM_RELEASE fill:#FFD700
    style SM_EXT_DISARM fill:#FFD700
```

## 4. Launch Command Flowchart (What Happens When You Call /comm/launch)

```mermaid
flowchart TD
    CALL(["/comm/launch<br/>service called"]) --> CHECK{"flight_state<br/>== IDLE or ABORT?"}

    CHECK -->|No| REJECT["Return: Cannot launch"]
    CHECK -->|Yes| CAPTURE["Capture hover reference:<br/>hover_pose.x = VICON current X<br/>hover_pose.y = VICON current Y<br/>hover_pose.z = 0.5m (target)<br/>hover_pose.orientation = current heading"]

    CAPTURE --> SET_STATE["flight_state = LAUNCH<br/>launch_time = now"]
    SET_STATE --> RESPOND(["Return: Launching to 0.5m"])

    RESPOND --> LOOP["Timer loops continue running"]
    LOOP --> SETPOINTS["Setpoint loop publishes<br/>hover_pose at 20Hz"]
    LOOP --> VISION["Vision loop publishes<br/>VICON pose at 30Hz"]
    LOOP --> SM["State machine at 10Hz"]

    SM --> WAIT{"1.5s<br/>passed?"}
    WAIT -->|No| SM
    WAIT -->|Yes| REQ_OFF["Request OFFBOARD mode"]
    REQ_OFF --> OFF_OK{"PX4 accepted<br/>OFFBOARD?"}
    OFF_OK -->|No, retry in 2s| REQ_OFF
    OFF_OK -->|Yes| REQ_ARM["Request arming"]
    REQ_ARM --> ARM_OK{"PX4<br/>armed?"}
    ARM_OK -->|No, retry in 2s| REQ_ARM
    ARM_OK -->|Yes| FLYING(["Motors spin<br/>Drone ascends to 0.5m<br/>Holds position"])

    style CALL fill:#90EE90
    style FLYING fill:#98FB98
    style REJECT fill:#FF6B6B
```

## 5. Landing Flowchart

```mermaid
flowchart TD
    CALL(["/comm/land<br/>service called"]) --> RECORD["Record:<br/>land_start_alt = current Z<br/>land_start_time = now<br/>flight_state = LAND"]

    RECORD --> LOOP["Every 50ms (20Hz setpoint loop)"]

    LOOP --> CALC["target_z = land_start_alt - 0.15 * elapsed<br/>Publish setpoint:<br/>X = hover X, Y = hover Y<br/>Z = max(target_z, 0.03)<br/>heading = hover heading"]

    CALC --> SM["Every 100ms (10Hz state machine)"]
    SM --> READ["Read altitude from VICON"]
    READ --> CHECK{"altitude<br/>< 0.12m?"}
    CHECK -->|No| TIMEOUT{"elapsed<br/>> 15s?"}
    TIMEOUT -->|No| LOOP
    TIMEOUT -->|Yes| FORCE["Force disarm<br/>(safety timeout)"]
    CHECK -->|Yes| DISARM["Disarm motors"]

    DISARM --> RESET["Reset to IDLE<br/>hover_pose = None"]
    FORCE --> RESET
    RESET --> DONE(["Landed"])

    style CALL fill:#90EE90
    style DONE fill:#98FB98
    style FORCE fill:#FF6B6B
```

## 6. Data Flow Summary (Part I)

```mermaid
flowchart LR
    subgraph INPUT["Pose Sources"]
        VICON["VICON<br/>(ground truth)"]
    end

    subgraph STATIONKEEPING["stationkeeping_node"]
        direction TB
        POSE_SEL["_get_active_pose<br/>Returns VICON pose"]
        SETPOINT["_setpoint_loop<br/>Publishes hover target"]
        STATE_M["_state_machine_loop<br/>Manages OFFBOARD + arm"]
    end

    subgraph PX4["PX4 Flight Controller"]
        EKF["EKF2<br/>State Estimator"]
        POS_CTRL["Position<br/>Controller"]
        ATT_CTRL["Attitude<br/>Controller"]
        MIXER["Motor Mixer"]
    end

    VICON -->|"Where am I?"| POSE_SEL
    POSE_SEL -->|"/mavros/vision_pose/pose"| EKF
    SETPOINT -->|"/mavros/setpoint_position/local"| POS_CTRL
    STATE_M -->|"/mavros/set_mode<br/>/mavros/cmd/arming"| PX4

    EKF -->|"Current state"| POS_CTRL
    POS_CTRL -->|"Attitude cmd"| ATT_CTRL
    ATT_CTRL -->|"Thrust + torque"| MIXER
    MIXER --> MOTORS(["4 Motors"])

    style MOTORS fill:#98FB98
```

## Key Parameters (Part I)

| Parameter | Value | Purpose |
|-----------|-------|---------|
| `takeoff_altitude` | 0.5m | Exercise #2 hover height |
| `setpoint_rate` | 20Hz | Position commands to PX4 |
| `vision_pose_rate` | 30Hz | VICON pose relay to PX4 |
| `offboard_wait` | 1.5s | Setpoint stream before OFFBOARD request |
| `mode_request_interval` | 2.0s | Retry interval for OFFBOARD/arm |
| `descent_speed` | 0.15 m/s | Landing ramp rate |
| `land_disarm_altitude` | 0.12m | Disarm when below this |
| `land_timeout` | 15s | Force disarm safety net |
| `vicon_topic` | `/vicon/ROB498_Drone/ROB498_Drone` | VICON PoseStamped source |
