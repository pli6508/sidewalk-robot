
# Sidewalk Robot Outdoor Testing Code

![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)

This repository contains the code and configurations for the Sidewalk Robot project, focused on developing and modifying navigation algorithms for a Husky UGV to operate autonomously on sidewalks. The project primarily focuses on outdoor testing scenarios, ensuring the robot can navigate effectively in real-world environments.

The repository includes custom ROS packages and configurations designed to enhance the Husky's navigation capabilities. These modifications include adjustments for GNSS data collection, AMCL localization, and mapping processes.

The robot used in this project is provided by [CLEARPATH Robotics](https://docs.clearpathrobotics.com/docs/robots/outdoor_robots/husky/user_manual_husky), and the model tested is the MCM07 HUSKY. This project is developed and tested on **Ubuntu 20.04** with **ROS Noetic**.

## Prerequisites

Ubuntu install of ROS Noetic guide is provided at the following link:
- [ROS Noetic Installation Guide](https://wiki.ros.org/noetic/Installation/Ubuntu)

Install the Husky ROS navigation stack. To quickly install the navigation stack, use the following command:

```bash
sudo apt-get install ros-noetic-husky-navigation
```

For full Husky UGV tutorials, please refer to the detailed guide provided by CLEARPATH Robotics at the following link:

- [Husky ROS Noetic Installation Guide](https://www.clearpathrobotics.com/assets/guides/noetic/husky/index.html)

## Installation and Setup

### Cloning the Repository

After installing the Husky ROS navigation stack, clone this repository into your ROS workspace:

```bash
cd ~/catkin_ws/src
git clone https://github.com/pli6508/sidewalk-robot.git
```

### Building the Workspace

Navigate back to your workspace and build the packages:

```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

```bash
# add the source code to the bashrc file
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
```

## Repository Structure

```plaintext
my_package/
├── launch/
│   ├── launch_mapping.launch
│   ├── launch_amcl.launch
│   └── duro_gnss_data_collection.launch
├── maps/
│   ├── MARC_IndoorL1.pgm
│   ├── MARC_IndoorL1.yaml
│   ├── MARC_OutdoorGoogle.pgm
│   ├── MARC_OutdoorGoogle.yaml
│   ├── MARC_OutdoorTurn.pgm
│   └── MARC_OutdoorTurn.yaml
├── rviz/
│   ├── robot_mapping.rviz
│   └── robot_navigation.rviz
├── images/
│   ├── MARC_L1.jpg
│   ├── MARC_L1_pgm.jpg
│   ├── MARC_GoogleMap.jpg
│   └── MARC_GoogleMap_pgm.jpg
└── src/
```

### `launch/`

This directory contains launch files for various operations:

- **launch_mapping.launch**: Launch file for creating maps of the environment.

Sample usage:

```bash
# Start the mapping process using the launch file from the my_package package
roslaunch my_package launch_mapping.launch
```

```bash
# Save the map after scanning, specifying the filename prefix
rosrun map_server map_saver -f ~/maps/my_map
```

- **launch_amcl.launch**: Launch file for running AMCL (Adaptive Monte Carlo Localization).

Sample usage:

```bash
# Start the navigation process with the specified map and custom scan topic
roslaunch my_package launch_amcl.launch map_name:=my_map.yaml
```

- **duro_gnss_data_collection.launch**: Launch file for collecting GNSS data (not required for mapping and navigation).

Sample usage:

```bash
# Launch the Piksi Multi node using default settings
roslaunch my_package duro_gnss_data_collection.launch
```

### `maps/`

This directory contains pre-generated maps used by the navigation stack:

- **MARC_IndoorL1.pgm / .yaml**: Indoor map files.

<p align="center">
  <img src="images/MARC_L1.jpg" alt="MARC L1" height="200px" />
  <img src="images/MARC_L1_pgm.jpg" alt="MARC L1 PGM" height="200px" />
</p>
<p align="center" style="font-size:14px;">
  <em>Left: Floor Plan | Right: Scanned Map for Highlighted Corner</em>
</p>

- **MARC_OutdoorGoogle.pgm / .yaml**: Outdoor map files from Google.

<p align="center">
  <img src="images/MARC_GoogleMap.jpg" alt="MARC GoogleMap" height="200px" />
  <img src="images/MARC_GoogleMap_pgm.jpg" alt="MARC GoogleMap PGM" height="200px" />
</p>
<p align="center" style="font-size:14px;">
  <em>Left: Plan View Referenced from Google Map | Right: Scanned Map for Highlighted Path</em>
</p>

- **MARC_OutdoorTurn.pgm / .yaml**: Turn-specific outdoor maps.

### `rviz/`

This directory contains RViz configuration files for visualizing the robot’s environment and navigation:

- **robot_mapping.rviz**: RViz configuration for mapping.
- **robot_navigation.rviz**: RViz configuration for navigation.

### `src/`

This directory is reserved for custom source code related to the project.

## Docker Container for GPU-Camera Module

This project develops a customized Docker container for running GPU-accelerated machine learning models on an OAK-D camera.

The Docker container is available on DockerHub at the repository [gpucam](https://hub.docker.com/r/ping6508/gpucam).

### Docker Container Setup:

**1. Before Starting:**

```bash
export DISPLAY=:1 # or 0 if 1 is not working  
xhost +
```

**2. Setup USB Permissions:**

```bash
echo 'SUBSYSTEM=="usb", ATTRS{idVendor}=="03e7", MODE="0666"' | sudo tee /etc/udev/rules.d/80-movidius.rules
sudo udevadm control --reload-rules
sudo udevadm trigger
```

**3. Run Docker:**

```bash
sudo docker run --rm -it \
  --runtime nvidia \
  --gpus all \
  --privileged \
  --mount type=bind,source=/dev/bus/usb,target=/dev/bus/usb \
  -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  ping6508/gpucam:latest
```

**4. Run YOLOP model:**

The YOLOP model is developed by Dong Wu, Manwen Liao, Weitian Zhang, Xinggang Wang, Xiang Bai, Wenqing Cheng, Wenyu Liu School of EIC, HUST. 

```bash
# Navigate to the YOLOP directory
cd /home/YOLOP
```

```bash
# Run YOLOP model with a webcam
python tools/demo.py --source 0 --device 0
```

```bash
# Run YOLOP model with OAK-D camera
python3 tools/oak_d.py --weights weights/End-to-end.pth --device 0
```

## Contributing

Contributions are welcome! Please fork this repository, make your changes, and submit a pull request.

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## Acknowledgments

- Original Husky navigation stack from [CLEARPATH Robotics](https://www.clearpathrobotics.com/).
- YOLOP model from [YOLOP](https://github.com/hustvl/YOLOP)
- The ROS community for providing necessary tools and libraries.
