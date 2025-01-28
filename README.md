# README: Multi-Robot Path Planning and Collision Avoidance Simulation

## Overview
This MATLAB program simulates a multi-robot environment where robots and humans navigate a shared space, avoiding collisions and static obstacles while achieving their respective goals. The simulation includes path planning, collision detection, obstacle avoidance, and visualization of robot movements in a 2D environment.

## Features
- **Multi-Robot Environment:** Simulates up to 3 robots with dynamic and static obstacles.
- **Path Planning:** Uses a Probabilistic Road Map (PRM) for pathfinding.
- **Collision Detection:** Detects collisions between robots and records collision times.
- **Obstacle Avoidance:** Implements velocity obstacle-based avoidance for safe navigation.
- **Visualization:** Real-time display of robot positions, trajectories, and obstacles.
- **Data Capture:** Saves simulation snapshots at specific times.

## Prerequisites
- MATLAB with the following toolboxes:
  - Robotics System Toolbox
  - Navigation Toolbox

## How to Use

### Step 1: Prepare the Environment
1. Place the `Map.mat` file in the working directory.
2. Ensure required toolboxes are installed.

### Step 2: Run the Simulation
1. Execute the script in MATLAB.
2. The simulation initializes the multi-robot environment and starts the robots.
3. Real-time updates are displayed in a figure window.

### Step 3: View Results
- Snapshots of the environment are saved in the `figure/Strategi2` folder.
- Simulation logs and collision detection messages are displayed in the MATLAB command window.

## Code Structure

### Sections
1. **Folder Creation:** Ensures a directory for saving figures.
2. **Simulation Parameters:** Defines time steps and robot parameters.
3. **Multi-Robot Environment:** Configures the simulation environment, robots, and sensors.
4. **Path Planning:** Plans paths for robots using PRM.
5. **Controllers:** Configures pure pursuit controllers for robot navigation.
6. **Collision Detection:** Checks for collisions between robots and logs events.
7. **Obstacle Avoidance:** Adjusts robot velocities to avoid obstacles dynamically.
8. **Visualization and Plotting:** Updates the environment and saves snapshots.

### Key Functions
- **velocity_obstacle:** Implements obstacle avoidance using velocity obstacles.
- **VFormationControl:** Ensures robots maintain a V-formation during movement.
- **plotAndAnalyzeRobotData:** Analyzes and plots simulation results.

## Outputs
- **Figures:** Saved in `figure/Strategi2` folder as `.fig` files.
- **Collision Log:** Displays collision events and times in the MATLAB command window.
- **Data Analysis:** Plots of velocity, error, and trajectory are generated for further analysis.

## Customization
- **Robot Parameters:** Modify the `numRobots`, `L`, `R`, or `max_velocity` variables.
- **Path Planning:** Adjust PRM parameters such as `NumNodes` and `ConnectionDistance`.
- **Formation Parameters:** Change `d` or `theta_formation` for different robot formations.

## Contact
For questions or issues, please contact habibputra151@gmail.com.

