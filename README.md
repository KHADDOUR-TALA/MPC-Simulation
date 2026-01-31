# Trajectory Tracking of Vehicle Using PID and MPC Controllers

This repository presents a trajectory tracking simulation for an autonomous vehicle using **Model Predictive Control (MPC)**, implemented in MATLAB/Simulink. The MPC controller enables the vehicle to accurately follow a **reference trajectory** while accounting for system dynamics and constraints.

## Overview

**Controller:** Model Predictive Control (MPC)
**Vehicle Model:** Kinematic vehicle model
**Reference Trajectory:** Waypoint- and step-input-based trajectory
**Simulation Environment:** MATLAB / Simulink
### Vehicle Kinematic Model
<p align="center">
  <img src="imgs/kinematic_model.png" width="300">
</p>


## Project Files
- **Vehicle Model:** A kinematic model of an autonomous vehicle is used to simulate the motion based on control inputs.
- **Reference Path:** The reference trajectory used for testing the controllers is a unit circle for the PID controller and a step input trajectory for the MPC controller.
- **Controller Implementation:** MATLAB/Simulink is used for designing the controllers, simulating the system, and visualizing the results.

## Files Included
- **AutonomousSteeringSystem.slx** – Main Simulink model for MPC-based autonomous steering

- **Meldas_library.slx** – Reference MPC library adapted from MathWorks MPC tutorials

- **Params.mat** – Vehicle model parameters and MPC tuning values

- **imgs/** – Vehicle model diagrams and simulation results
## How to Run the Simulation

1. Open the `Meldas_library.slx` Simulink library to check and modify the MPC configuration.
   
2. Open the `Params.mat` file to load the necessary parameter values into the workspace for the Simulink model.
   
3. Open and run the Simulink model `AutonomousSteeringSystem.slx` to start the simulation.

## Simulation Results

- The MPC controller tracks the reference path (step input trajectory) more accurately.
- Lateral positions and yaw angles are compared between the reference and actual trajectory.

### Lateral Positions of Reference and MPC Tracking
<p align="center">
  <img src="imgs/MPC_tracking_lateral.png" width="400">
</p>

### Yaw Angles of Reference and MPC
<p align="center">
  <img src="imgs/MPC_yaw_angle_result.png" width="400">
</p>

## Conclusion
The Model Predictive Control approach demonstrates accurate and robust trajectory tracking by predicting future vehicle behavior and optimizing control inputs under system constraints. This makes MPC well-suited for autonomous vehicle steering and path-following applications.
