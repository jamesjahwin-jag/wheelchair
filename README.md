## Wheelchair Description & Simulation

This repository contains two folders: wheelchair-95.snapshot.8 and wheelchair_ws. The former (wheelchair-95) contains all the 3D CAD files for a wheelchair gotten from Grabcad and can be opened using Freecad, in case what you want to see what the parts look like or to experiment with it. The latter, wheelchair_ws, is the main area of concern, it contains all the files you need to launch the assembled wheelchair in Rviz and Gazebo. This project is part of an overarcing project at Aurora Robotics to apply robotics to medical assistive tools, in this case, a wheelchair.

## 🚀 Features

* **XACRO-based modeling:** Modular robot description for easy parameter adjustments (mass, inertia, wheel separation).
* **Gazebo Integration:** Includes physics properties (, ) and the `libgazebo_ros_diff_drive` plugin.
* **Visual Fidelity:** Utilizes high-quality STL meshes exported from Blender 4.5.6.
* **Partially Stabilized Physics:** Optimized mass and inertia values to prevent "jumping" and numerical instability in ODE.

---

## 🛠 Installation

### Prerequisites

* ROS 2 (Humble or newer recommended)
* Gazebo11 / Gazebo Sim
* `xacro` and `gazebo_ros_pkgs`

### Setup
1. Create a folder where you intend to work with this project
```
For example in Documents, create a folder wheelchair
```

2. Clone the repository into your new workspace:
```
cd ~/wheelchair
git clone https://github.com/jamesjahwin-jag/wheelchair.git
```

3. Build the package:
Before that you need to enter the folder, wheelchair_ws (assuming you're in the wheelchair folder from the terminal)
```
cd wheelchair_ws
```
And then run:
```
colcon build --symlink-install
source install/setup.bash
```

---

## 💻 Usage
To launch the wheelchair in Rviz:
```
ros2 launch urdf_tutorial display.launch.py model:=$PWD/wheelchair_description/urdf/wheelchairtest.xacro
```

To launch the wheelchair in an empty Gazebo world:
```
ros2 launch wheelchair_description wheelchair_gazebo.launch.py
```

---

## 📂 Project Structure

```
wheelchair_description/
├── launch/             # Python launch files for Gazebo
├── meshes/             # STL files for visual and collision geometry
├── urdf/               # URDF and XACRO robot description files
├── CMakeLists.txt      # Build configuration
└── package.xml         # Package metadata and dependencies
```

---

## 🔧 Note
I have not written the launch file perfectly. So, when you launch it in Gazebo, it looks like this:
<img width="1919" height="1079" alt="Screenshot 2026-02-18 204036" src="https://github.com/user-attachments/assets/5a7ea46d-6e99-42a2-8478-05fd4f13768c" />
It will be subsequently worked on.

---

## 🤝 Credits

* **Author:** Jahwin James James
* **Tooling:** URDF created with reference to Blender 4.5.6 using the [Linkforge Extension](https://github.com/arounamounchili/linkforge).

---

**Would you like me to add a section on how to troubleshoot common Gazebo errors or how to customize the wheelchair colors?**
