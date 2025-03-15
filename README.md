# UR5 Manipulator Kinematics and Control in CoppeliaSim with MATLAB

This project focuses on developing forward and inverse kinematics, trajectory planning, and control algorithms for a UR5 robotic manipulator. It integrates MATLAB with the CoppeliaSim (formerly V-REP) simulator to validate the theoretical models and visualize the robot's motion. The work was completed as part of an undergraduate robotics course.

---

## Table of Contents
1. [Overview](#overview)
2. [Key Features](#key-features)
3. [Prerequisites](#prerequisites)
4. [Setup & Installation](#setup--installation)
5. [Implementation Details](#implementation-details)
6. [Results](#results)
7. [How to Run](#how-to-run)
8. [Troubleshooting](#troubleshooting)
9. [References](#references)

---

## Overview
The project involves:
- **Forward Kinematics**: Calculating the end-effector pose (position and orientation) from joint angles using Denavit-Hartenberg (DH) parameters.
- **Inverse Kinematics**: Deriving joint angles from a desired end-effector pose.
- **Trajectory Planning**: Generating smooth joint trajectories using cubic polynomials.
- **CoppeliaSim Integration**: Simulating and controlling the UR5 robot in real-time via MATLAB's remote API.

---

## Key Features
- **DH Parameters**: Defined for all 6 joints of the UR5 manipulator (see [DH Table](#denavit-hartenberg-table)).
- **Trajectory Planning**: Cubic polynomial interpolation for smooth motion between initial and final joint angles.
- **Error Analysis**: Quantifies discrepancies between calculated and simulated poses.
- **Visualization**: MATLAB plots for joint angles, end-effector positions, and orientation errors.
- **Sensor Integration**: Utilizes CoppeliaSim's vision sensor and proximity sensor for object detection.

---

## Prerequisites
1. **Software**:
   - MATLAB (tested on R2020b or later).
   - CoppeliaSim EDU (V-REP) [Download here](https://www.coppeliarobotics.com/).
   - MATLAB's `remoteApi` library (ensure bit-version matches MATLAB and CoppeliaSim).
2. **Hardware**: 
   - Compatible OS (Windows/Linux/macOS).
   - Ensure MATLAB and CoppeliaSim can run simultaneously.

---

## Setup & Installation
1. **CoppeliaSim Setup**:
   - Import the UR5 scene (includes the robot, target objects, and sensors).
   - Ensure the scene contains objects named `UR5_joint1` to `UR5_joint6`, `Base`, `Cuboid`, `Cup`, and `Vision_sensor`.
2. **MATLAB Configuration**:
   - Add CoppeliaSim's `remoteApi` folder to MATLAB's path.
   - Verify the `remoteApiProto.m` file is accessible.
3. **File Structure**:
   - Place `Trabalho.m` (main script) and `Parametros.m` (DH parameters) in the same directory.

---

## Implementation Details

### Denavit-Hartenberg Table
| Joint | \(a_n\) | \(\alpha_n\) (°) | \(d_n\) | \(\theta_n\) (°) |
|-------|---------|-------------------|---------|-------------------|
| 1     | 0       | 0                 | 0.1     | \(\theta_1^*\)    |
| 2     | 0       | 90                | 0       | \(\theta_2^*\)    |
| 3     | -0.42   | 0                 | 0       | \(\theta_3^*\)    |
| 4     | -0.40   | 0                 | 0.09    | \(\theta_4^*\)    |
| 5     | 0       | 90                | 0.09    | \(\theta_5^*\)    |
| 6     | 0       | -90               | 0.22    | \(\theta_6^*\)    |

### Trajectory Planning
A cubic polynomial ensures zero initial/final velocities:
\[
q(t) = q_o + 0.12(q_f - q_o)t^2 - 0.25(q_f - q_o)t^3
\]
- \(q_o\): Initial joint angle.
- \(q_f\): Final joint angle.
- Coefficients: \(a_0 = q_o\), \(a_1 = 0\), \(a_2 = 0.12(q_f - q_o)\), \(a_3 = -0.25(q_f - q_o)\).

### Kinematics
- **Forward Kinematics**: Computes \(T_0^6\) (end-effector transform matrix) using DH transformations.
- **Inverse Kinematics**: Solves joint angles geometrically for a desired \(T_0^6\). Key steps:
  1. Calculate \(\theta_1\) using the projection of the wrist center.
  2. Solve \(\theta_5\) and \(\theta_6\) from end-effector orientation.
  3. Derive \(\theta_2\), \(\theta_3\), and \(\theta_4\) using trigonometric relationships.

---

## Results
### Forward Kinematics (Sample Output)
- **Calculated Pose**: Position = `[-0.47674, 0.40345, 0.71247]`, Orientation = `[90°, -70°, 20°]`.
- **Simulated Pose**: Position = `[-0.49994, 0.41046, 0.69908]`, Orientation = `[90.1061°, -68.0488°, 19.5785°]`.
- **Errors**: Position = `[0.0232, 0.0070, 0.0134]`, Orientation = `[0.106°, 1.951°, 0.422°]`.

### Inverse Kinematics (Sample Output)
- **Target Pose**: Position = `[-0.2, -0.2, 0.5]`, Orientation = `[0°, 0°, 0°]`.
- **Achieved Pose**: Position = `[-0.21949, -0.22115, 0.5078]`, Orientation = `[2.5456°, 0.4973°, -1.9423°]`.
- **Joint Angles**: \(\theta_1 = 26.45°\), \(\theta_2 = -112.41°\), \(\theta_3 = 144.03°\), \(\theta_4 = -121.62°\), \(\theta_5 = 90.00°\), \(\theta_6 = 63.55°\).

---

## How to Run
1. Start CoppeliaSim and load the UR5 scene.
2. Run `Trabalho.m` in MATLAB:
   ```matlab
   >> Trabalho
