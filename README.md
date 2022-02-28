# fbgs_sensor
ROS interface for [Illumisense (FBGS) software](https://fbgs.com/components/illumisense-software/)

This is a very simple example which creates an fbgs_sensor_node that currently only broadcast strain and wavelength measurements from FBGS

# Dependencies
* ROS 
* Illumisense (Windows Only Currently)

# Installation
Clone project into catkin workspace and build the project. 
```bash
git clone https://github.com/ScottyChung/fbgs_sensor
catkin build fbgs_sensor
```

# Getting Started
The Illumisense software has a setting to stream measurements over TCP. Ensure this streaming is enabled on the measurement workstation. Modify the host_computer variable in `src/fbgs_sensing_node.py` to match the measurment workstation. Running the following command should start the node. 

```bash
rosrun fbgs_sensing fbgs_sensing_node.py
```

This should create a node `/fbgs_sensor` that is broadcasting the topic `/fbgs_strain`

# TODO
~~* Explain more in readme~~
~~* Handle multiple channels and sensors (not sure how the strain is reported with multi-channels)~~
* Potentially add to ROS or cisstsaw repository
* Create topics for wavelength, power, other info
* Add CPP example (maybe)
* Add UI (maybe)
