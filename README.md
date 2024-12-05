
# Beam and Ball Controller Design

## Table of Contents
1. [Project Overview](#project-overview)
2. [Directory Structure](#directory-structure)
3. [Documentation](#documentation)
4. [How to Use](#how-to-use)
   - [Cloning the Repository](#cloning-the-repository)
   - [Running the Simulation](#running-the-simulation)
5. [Performance Metrics](#performance-metrics)
6. [Known Limitations](#known-limitations)
7. [Future Work](#future-work)
8. [Contact](#contact)

---

## Project Overview
This project compares the performance of two control strategies—Linear Quadratic Regulator (LQR) and Pole Placement via Ackermann's formula—for stabilizing a ball on a beam. The performance is evaluated using sensitivity analysis and key metrics such as Integral of Absolute Error (IAE), Integral of Squared Error (ISE), and Integral of Time-weighted Absolute Error (ITAE). MATLAB simulations and 3D visualizations validate the results, demonstrating trade-offs between energy efficiency, transient response, and robustness.

---

## Directory Structure
Here is the repository layout:
```
.
|-- .gitignore        # Ignore file to exclude unnecessary files
|-- LICENSE           # License for the project
|-- README.md         # Documentation file (this file)
|
|-- documentation/    # Documentation resources
|   |-- presentation/ # Project presentation and associated figures
|   |-- references/   # Reference papers and tutorials
|   |-- report/       # Final report and figures
|
|-- matlab-simulink/  # MATLAB and Simulink project files
|   |-- models/       # Simulink models for the beam-and-ball system
|   |-- scripts/      # MATLAB scripts for parameter setup and simulations
|
```

## Documentation
The repository includes detailed documentation:
- **Presentation**: [`documentation/presentation/Project_Presentation.pdf`](./documentation/presentation/Project_Presentation.pdf)  
  Contains slides and associated figures.
- **References**: [`documentation/references/`](./documentation/references/)  
  Includes research papers and tutorials.
- **Final Report**: [`documentation/report/Final_Report.pdf`](./documentation/report/Final_Report.pdf)  
  Includes the final report and source files.

---

## How to Use

### Cloning the Repository
To use the project, first clone the repository to your local system:
```bash
git clone https://github.com/alexanderbr0wn/Beam_and_Ball_Controller_Design.git
cd Beam_and_Ball_Controller_Design
```

### Running the Simulation

#### Prerequisites
1. MATLAB with the following toolboxes installed:
   - Simscape Multibody Toolbox
   - Control System Toolbox
2. Ensure all necessary files are downloaded by cloning the repository.

#### Steps to Run the Simulation
1. Open MATLAB and navigate to the project directory:
   ```matlab
   cd 'C:\Users\YourName\Beam_and_Ball_Controller_Design\'
   ```
2. Run the MATLAB script to set up parameters:
   ```matlab
   run('matlab-simulink/scripts/beam_and_ball_param_and_sim.m')
   ```
3. Open the Simulink model:
   ```matlab
   open('matlab-simulink/models/beam_and_ball_3d_model.slx')
   ```
4. Run the simulation and observe:
   - 3D visualization in Mechanics Explorer.
   - Performance metrics in the MATLAB command window.

#### Validation Steps
1. **Run the simulation**: Follow the steps in the "How to Use" section to execute the simulation in MATLAB/Simulink.
2. **Performance Metrics**: 
   - Observe the performance metrics (IAE, ISE, ITAE) displayed in the MATLAB command window.
3. **3D Visualization**:
   - View the real-time response of the ball-and-beam system in the Simscape Mechanics Explorer.
4. **Compare Controllers**:
   - Toggle between the LQR and Pole Placement controllers by modifying the feedback gain vector `K` in the script.

---

## System Components

### Key Features of the Simulink Model
- **Controller Subsystems**:
  - The Simulink model includes two controller subsystems, each accepting a controller gain vector `K`:
    - **Pole Placement Controller (Ackermann's formula)**: Uses pole placement methodology to compute feedback gains.
    - **LQR Controller**: Implements state feedback gains calculated using the Linear Quadratic Regulator method.
  - You can toggle between the controllers by configuring the appropriate subsystem and gain vector.

- **Key Physical Components**:
  - **Beam**: Tilts to control the position of the ball. Represented with rigid transforms and angular motion.
  - **Ball**: Rolls on the beam based on gravity and beam tilt. Modeled as a rigid body.
  - **Joints**:
    - Revolute Joint: Controls beam tilt.
    - Prismatic Joint: Allows linear motion of the ball along the beam.
  - **Rigid Transforms**: Maintain the relative positioning of components.

- **Subsystems**:
  - **System States**: Tracks the states of the system: ball position (`x`), ball velocity (`x_dot`), beam angle (`theta`), and angular velocity (`theta_dot`).
  - **Integral Compensation**: Implements integral control to eliminate steady-state error.

---

## Performance Metrics
The simulation evaluates controller performance using the following metrics:
1. **IAE (Integral of Absolute Error)**: Evaluates total accumulated error.
   - **Pole Placement**: 0.13262
   - **LQR**: 0.13314
2. **ISE (Integral of Squared Error)**: Penalizes large deviations more heavily.
   - **Pole Placement**: 0.01937
   - **LQR**: 0.01975
3. **ITAE (Integral of Time-weighted Absolute Error)**: Penalizes long-lasting errors.
   - **Pole Placement**: 0.05834
   - **LQR**: 0.05626 (3.56% improvement over Pole Placement)

---

## Known Limitations
1. **Simplified Physics**:
   - Friction and air resistance are not modeled.
2. **Parameter Sensitivity**:
   - The Pole Placement controller showed higher sensitivity to changes in ball mass and beam length compared to the LQR controller.
   - Stability depends on accurate physical parameters and proper controller tuning.
3. **Model Constraints**:
   - Large beam angles or extreme ball positions may exceed simulation limits.
4. **Validation Environment**:
   - Simulations do not account for real-world factors like actuator delays, sensor noise, and hardware constraints.

---

## Future Work
1. **Adaptive and Robust Control**:
   - Implement adaptive control strategies, such as gain-scheduling or self-tuning controllers, to handle parameter variations dynamically.
   - Develop robust control methods to mitigate the effects of uncertainties and disturbances.
2. **Nonlinear Control Techniques**:
   - Explore sliding mode control or backstepping to address system nonlinearities.
3. **Hardware Implementation**:
   - Apply the proposed controllers to a physical system to validate their performance under real-world conditions, accounting for noise, delays, and actuator limitations.
4. **Enhanced Physics Modeling**:
   - Include friction, air resistance, and other real-world factors in the simulations.
5. **Energy Optimization**:
   - Design energy-efficient control strategies to reduce actuator wear and improve overall system efficiency.

---

## Contact
For questions or feedback, contact:  
Alexander Brown  
Email: alexanderjbrown1@gmail.com, ajb0083@uah.edu