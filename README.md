# LAB4: 3R Kinematics

This project implements a **modular ROS2 control architecture** for a 3-DOF articulated robot arm. It includes **Inverse Kinematics (IK)**, **Resolved-Rate Motion Control**, **Teleoperation**, **Workspace-Aware Random Target Generation**, and **RViz visualization**.

This README explains the architecture, how to run the system, and how each component satisfies the LAB4 requirements.

---

## System Overview

### ROS2 Nodes and Functions

| Node | Function |
| :--- | :--- |
| **controller\_node** | Performs IK control, teleoperation control, auto-mode resolved-rate control, publishes end-effector pose |
| **scheduler\_node** | Handles mode switching and coordinates controller ↔ random node |
| **random\_node** | Generates workspace-valid random target poses |
| **teleop\_jog\_keyboard** | Publishes `/cmd_vel` for teleoperation |
| **workspace.py** | Computes theoretical workspace + visual verification |

### System Architecture

![System Architecture](/home/b/Documents/GitHub/FRA502-LAB-6631/system_architecture.png)

### Main Topics

* `/cmd_vel` — teleoperation command (**Twist**)
* `/target` — target pose to display in RViz
* `/end_effector` — real end-effector pose from TF
* `/controller_status` — **"TARGET\_REACHED"** event
* `/singularity_alert` — warning when approaching singularity
* `/current_state` — controller/scheduler state

### Main Services

* `/set_control_mode` – change robot mode (IK, AM, TO\_F, TO\_G)
* `/controller_server` – internal control commands (scheduler → controller)
* `/get_random_pose` – request a workspace-valid random pose

---

## Requirements Satisfaction (Summary)

### Part 1: Core Components

* **Workspace** computed using link lengths (`workspace.py`).
* **Random pose generator** with workspace checking and IK-feasibility (`random\_target.py`).
* **RViz visualization** with `/target` and `/end_effector` (`controller.py`).

### Part 2: Control Modes

#### **IK Mode (Inverse Kinematics)**
* ✓ Return success/failure + configuration solution.
* ✓ Use IK to verify reachability, then **resolved-rate control** moves the arm.

#### **Teleoperation Mode (TO\_F, TO\_G)**
* ✓ Control in **End-effector frame** or **global frame**.
* ✓ **Singularity detection** + emergency stop + topic notification.

#### **Auto Mode (AM)**
* ✓ Request random pose.
* ✓ Must reach target within **10 seconds**.
* ✓ Sends **"TARGET\_REACHED"** for next random pose.

### Part 3: Documentation
* Documentation (this README) + architecture + instructions to run.

---

## Installation and Running

### Requirements

* Ubuntu 22.04
* ROS2 Humble
* Python 3.10
* `roboticstoolbox-python`
* `spatialmath-python`

### Install Python dependencies

```bash
pip install roboticstoolbox-python spatialmath-python numpy
```

### Clone the project

```bash
git clone -b LAB4 https://github.com/bpbb/FRA502-LAB-6631.git
```

### Build the ROS2 workspace

```bash
cd LAB4
colcon build
source install/setup.bash
```

### Run the full system

```bash
ros2 launch lab4_controller robot_control.launch.py
```

```bash
ros2 run lab4_controller teleop_jog_keyboard.py
```

---

## Behavior of Each Mode


### IDLE Mode

**Who controls it:** scheduler  
**Default state:** `IDLE`

#### **Behavior**
- Robot maintains current joint state  
- Controller does not generate velocity commands  
- No IK, no teleop, no auto movement  
- Only publishes EE pose for RViz  

#### **Purpose in LAB4**
Acts as the **safe baseline** and fallback state after finishing tasks or encountering errors.

---

### IK Mode (Inverse Kinematics Mode)

#### **Purpose (LAB4 Requirement)**
- Accept a desired end-effector position  
- Attempt to solve IK  
- If **solvable → robot must move**  
- If **unsolvable → robot must NOT move**  

#### **User Command**
```bash
ros2 service call /set_control_mode interfaces/srv/SetControlMode "{mode_name: 'IK', target_pose: {...}}"
```

#### *Internal Behavior*

**Step 1 — Scheduler receives mode change**
- Extracts `(x, y, z)` from the `target_pose`  
- Calls: `req_ik() → inverse_kinematic()`

**Step 2 — IK Feasibility Check**
`inverse_kinematic()`:
- Checks workspace bounds:  
  - `distance > r_max` → **fail**  
  - `distance < r_min` → **fail**
- Uses Robotics Toolbox IK with multiple initial guesses  
- If **any** solution is valid → **IK success**

**Step 3 — Return results**
If IK succeeds:

```bash
response.success = True
response.configuration_solution = [q1, q2, q3]
```

If IK fails:
```bash
response.success = False
```
→ Robot must stay at its current configuration.

For example,

1) IK succeeds
```bash
ros2 service call /set_control_mode interfaces/srv/SetControlMode "{mode_name: 'IK',
  target_pose: {
    header: {frame_id: 'link_0'},
    pose: {position: {x: 0.3, y: 0.1, z: 0.3}}
  }}"
```
Result:
```
response:
interfaces.srv.SetControlMode_Response(success=True, message='IK solution found. Robot moving to target.', configuration_solution=sensor_msgs.msg.JointState(header=std_msgs.msg.Header(stamp=builtin_interfaces.msg.Time(sec=0, nanosec=0), frame_id=''), name=['joint_1', 'joint_2', 'joint_3'], position=[0.38503838262756407, -0.6622210993373332, 1.796273593445921], velocity=[], effort=[]))

```

2) IK fails
```bash
ros2 service call /set_control_mode interfaces/srv/SetControlMode \
"{mode_name: 'IK',
  target_pose: {
    header: {frame_id: 'link_0'},
    pose: {position: {x: 0.15, y: 0.1, z: 0.2}}
  }}"
```
Result:
```
response:
interfaces.srv.SetControlMode_Response(success=False, message='IK solution NOT found. Robot remains in current state.', configuration_solution=sensor_msgs.msg.JointState(header=std_msgs.msg.Header(stamp=builtin_interfaces.msg.Time(sec=0, nanosec=0), frame_id=''), name=[], position=[], velocity=[], effort=[]))
```

**Step 4 — Movement Execution (Resolved-Rate Control)**

IK is used **only for feasibility**.  
Actual motion is executed using **resolved-rate control**, which includes:

- Damped Least Squares (DLS)
- Null-space posture control
- Joint limit avoidance
- Velocity saturation

These ensure smooth end-effector motion toward the target while **avoiding singularities**.

---

### Teleoperation Mode

Two teleoperation modes:

- **TO_F** — end-effector frame  
- **TO_G** — global/world frame  

#### **Purpose (LAB4 Requirement)**
- Control robot using keyboard  
- Receive velocity commands via `/cmd_vel`  
- Switch between **global frame** and **EE frame** motion  
- Detect and stop near singularities  

#### **User Commands**

**End-effector frame:**
```bash
ros2 service call /set_control_mode "{mode_name: 'TO_F'}"
```

**Global frame:**
```bash
ros2 service call /set_control_mode "{mode_name: 'TO_G'}"
```

#### **Singularity Detection**

Before applying motion:

- Compute smallest singular value `s` of the Jacobian  
- If `s < threshold`:  
  - **Immediately stop movement**  
  - Log a warning  
  - Publish `/singularity_alert`

This fully satisfies the LAB4 **safety + reporting** requirements.

---

### Auto Mode (AM)

#### **Purpose (LAB Requirement)**
- Request a random pose from `/get_random_pose`  
- Move to the pose within **≤ 10 seconds**  
- When reached → request next pose  
- Repeat indefinitely  
- Stop automatically when user changes mode  

#### **User Command**
```bash
ros2 service call /set_control_mode "{mode_name: 'AM'}"
```

### **Random Node Behavior**

The random node uniformly samples `(x, y, z)` and ensures:

- Pose is inside the valid workspace  
  - `r_min ≤ r ≤ r_max`
- IK solution exists  
- FK path does **not** collide with the ground  
- Publishes `/target` for RViz  
- Returns `target_pose` to the scheduler  

---

## Workspace Computation
Run workspace visualization:
```bash
ros2 run lab4_controller workspace.py
```
![Robot Workspace](/home/b/Documents/GitHub/FRA502-LAB-6631/3R_workspace.png)


