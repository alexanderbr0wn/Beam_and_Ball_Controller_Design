# Beam and Ball Controller Design

## Project Overview
This project simulates the **beam-and-ball system** to test and validate the performance of a Linear Quadratic Regulator (LQR) controller. It combines physical modeling in **Simscape Multibody** with dynamic control implemented in MATLAB. The system demonstrates stabilization of a ball on a beam by adjusting the beam's angle, with outputs visualized in both plots and 3D animations.

---

## Purpose
- **Simulation of Physical Behavior**: Accurately model the ball-and-beam dynamics, including physical constraints and motion.
- **Testing Controller Performance**: Validate LQR-based state-feedback control using key performance metrics such as settling time, percent overshoot, and error integrals.

---

## Software Used
1. **Simscape Multibody**:
   - Models the physical system, including the ball, beam, rigid transforms, and joints.
   - Provides 3D animations of the system's behavior.
2. **MATLAB**:
   - Defines physical parameters and system matrices for the model.
   - Implements the LQR controller and computes metrics to assess performance.

---

## System Components
### Key Physical Components
1. **Beam**:
   - Tilted to control the position of the ball.
   - Represented with rigid transforms and angular motion.
2. **Ball**:
   - Rolls on the beam based on gravity and beam tilt.
   - Modeled as a rigid body.
3. **Joints**:
   - **Revolute Joint**: Controls beam tilt.
   - **Prismatic Joint**: Allows linear motion of the ball along the beam.
4. **Rigid Transforms**:
   - Maintain the relative positioning of components.

### Subsystems
1. **System States**:
   - Tracks the states of the system: ball position (\(x\)), ball velocity (\(\dot{x}\)), beam angle (\(\theta\)), and angular velocity (\(\dot{\theta}\)).
2. **Ball**:
   - Simulates the ball's position and velocity.
3. **Beam**:
   - Simulates the beam's rotational dynamics.
4. **Integral Compensation**:
   - Implements integral control to eliminate steady-state error.

---

## Inputs and Outputs
### Inputs
- **Physical Parameters**:
  - Beam length, mass, moment of inertia, etc.
- **Controller Gains**:
  - LQR feedback gain matrix (\(K\)) and integral gain (\(K_i\)).
- **System Matrices**:
  - State-space representation (\(A, B, C, D\)).
- **Reference Input**:
  - Desired ball position.

### Outputs
- **State Trajectories**:
  - Ball position, velocity, beam angle, and angular velocity over time.
- **Performance Metrics**:
  - Settling time, percent overshoot, IAE, ISE, ITAE.
- **3D Visualization**:
  - Live animation of the ball-and-beam system in Mechanics Explorer.

---

## How to Use
### Prerequisites
- **MATLAB** (with Simscape Multibody Toolbox).
- Install all dependencies listed below.

### Steps
1. **Run the MATLAB Script**:
   - Open `beam_and_ball_parameters_and_simulation.m`.
   - Configure physical parameters and controller settings.
   - Run the script to compute controller gains and send inputs to the 3D model.
2. **Simulate the 3D Model**:
   - Open `beam_and_ball_3d_model.slx` in Simulink.
   - Run the simulation and observe:
     - 3D behavior in Mechanics Explorer.
     - Performance metrics in MATLAB.

---

## Dependencies
1. **Toolboxes**:
   - MATLAB (Core)
   - Simscape Multibody Toolbox
   - Control System Toolbox
2. **Files**:
   - `beam_and_ball_parameters_and_simulation.m` (MATLAB script).
   - `beam_and_ball_3d_model.slx` (Simulink model).

---

## Performance Metrics
- **Settling Time**: Time required for the ball to stabilize near the reference position.
- **Percent Overshoot**: Maximum deviation of ball position beyond the reference.
- **IAE (Integral of Absolute Error)**: Total absolute error over the simulation.
- **ISE (Integral of Squared Error)**: Penalizes large deviations more heavily.
- **ITAE (Integral of Time-weighted Absolute Error)**: Penalizes long-lasting errors.

---

## Known Limitations
- **Simplified Physics**:
  - Friction and air resistance are not modeled.
- **Parameter Sensitivity**:
  - System stability depends on accurate physical parameters and controller tuning.
- **Model Constraints**:
  - Large beam angles or extreme ball positions may exceed valid simulation limits.

---

## Future Work
1. **Extend Controller**:
   - Implement a Kalman filter for state estimation.
   - Add disturbance rejection (e.g., external forces on the ball).
2. **Improve Physics**:
   - Incorporate friction and more realistic ball dynamics.
3. **Dynamic References**:
   - Support tracking of time-varying reference positions.

---

## Example Output
1. **Plots**:
   - Ball position (\(x\)) and beam angle (\(\theta\)) over time.
2. **3D Animation**:
   - Visualization of the ball's movement on the beam in real time.

---

## Contact
For questions or feedback, contact:  
Alexander Brown  
alexanderjbrown1@gmail.com, ajb0083@uah.edu
