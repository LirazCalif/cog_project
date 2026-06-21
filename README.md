# COG_PROJECT – Fetch Robot Simulation

This project implements a robotic planning and control system for the **Fetch mobile manipulator** using **Webots** and **ROS 2**.  
It includes robot description files, controllers, kinematics data, and simulation assets for interacting with objects (such as a medicine bottle) in a simulated environment.

---

## Project Overview

The goal of this project is to simulate and control the Fetch robot in Webots, enabling motion planning and interaction with objects using ROS 2 tools.  
The project integrates:
- Webots simulation
- ROS 2 robot description and launch files
- Fetch robot URDF and MoveIt configuration
- Custom controllers and kinematics data

---

## Project Structure
```bash
COG_PROJECT/
│
├── controllers/
│ └── fetch_controller/
│ ├── fetch_kinematics_data.json
│ ├── fetch_webots.urdf
│ └── run_fetch_controller.bat
│
├── src/
│ └── fetch_description/
│ ├── urdf/
│ ├── config/
│ ├── launch/
│ └── meshes/
│
├── fetch_webots.proto
├── MedicineBottle.proto
└── .gitignore
```

---

## Requirements

- **ROS 2** (Humble / Foxy or compatible)
- **Webots**
- **Python 3**
- **colcon**
- **MoveIt 2**
- Ubuntu or Windows (Windows supported via `.bat` scripts)

---

## Installation

1. Clone the repository:
   
   git clone https://github.com/LirazCalif/cog_project.git
   cd cog_project

2. Source the ROS 2 installation
   
   source /opt/ros/humble/setup.bash
  
3. Build the workspace:
   
   colcon build
  
4. Source the workspace:
   
   source install/setup.bash


Now, Running the Simulation:

1. Open Webots and load the Hospital world.

2. Choose the controller "webot_moveit"
