# Freedom Vehicle

Simulation of LOGNAV4.0 project vehicle. Here are all file necessary to run the simulation

## Build Status

| Service | Foxy  | Humble |
| :---: | :---: | :---: |
| ROS Status | ![Build Status](https://img.shields.io/badge/build-unknown-lightgray.svg) |  ![Build Status](https://img.shields.io/badge/build-stable-brightgreen.svg) |

## Cloning

<!-- To download the 3D model files, for example the .stl files, you will need to install git's open-source extension, called [Git Large File Storage](https://git-lfs.com/). (See [installation instructions for git-lfs](https://github.com/git-lfs/git-lfs/blob/main/INSTALLING.md)).  -->

<!-- ```
curl -s https://packagecloud.io/install/repositories/github/git-lfs/script.deb.sh | sudo bash
sudo apt-get install git-lfs
git lfs install
``` -->

<!-- Now, when you clone this repo, you should also download all the large files with it. -->

```
git clone https://github.com/lognav4-0/freedom_vehicle.git
```

## Installation

This packages has been released for the following ROS distributions

- `foxy` (See [installation instructions for ROS2 Foxy](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html))

Once the `ros-<distro>-desktop-full` package for the desired distribution is installed, it is needed to setup your .bashrc with (in our project: <workspace-directory> -> ros_ws):

```
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:$HOME/<workspace-directory>/install/lognav_vehicle/share/lognav_vehicle/models 
```

<!-- ## Velodyne Puck Installation

At the same src/ directory, clone into the following repo:
```
cd ~/<ros_workspace>/src/
git clone https://bitbucket.org/DataspeedInc/velodyne_simulator.git
```

And checkout to the foxy-devel branch:
```
cd velodyne_simulator/
git checkout foxy-devel
```

Build your simulation:
```
cd ~/<ros_workspace>/
colcon build
``` -->

## Simulation

To run the simulation model freedom: 
```
ros2 launch lognav_vehicle vehicle_display.launch.py
```

<!-- To run the simulation model simplified: 
```
ros2 launch freedom_vehicle display_simp.launch.py
``` -->
