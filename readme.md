# Description
This ROS package provide a simple demo for realize Digital Twin in my Master thesis on the topic: Environmental-aware control architecture for safety-critical cyber-physical systems -- A case study for quadrotor via Digital Twin. 

# Main documents
1. src/digital_twin.cpp: create a node of "twin_node", this script creates an object of class DigitalTwin which can include multiple digital models. in the project, it contains an object of class QuadDynamicModel defined in src/dynamic_model_quad.cpp. 
2. src/dynamic_model_quad.cpp: create an object of class QuadDynamicModel. It subscribes to specific rostopics as input, and calculates the wind drag force, resultant force, resultant moment, linear and angular acceleration based on drone dynamics.
3. src/env_node.cpp: it is designed to process the environmental information and publish them in the form of rostopic. This project realized the function of reading wind data from the weather station and publishing them immediately after aggregation.
4. src/physical_entity.cpp: it aggregates all the available information and publishes them under the topic of /physical_entity


# Prerequisite
Ubuntu 20.04 and ROS Noetic
Ubuntu 18.04 and ROS Melodic

# Usage
0. install pkg
    - create a catkin workspace with ```mkdir -p catkin_ws/src```
    - clone the package with ```git clone https://github.com/JunYou42/MA_dt_quadrotor.git``` and install dependencies
    - ompile the package with ```catkin build```
1. connect to drone

    - WINDOWS: open Vicon system -> select DIYDRONE


    - WEATHER STATION: connect and switch to weather log mode


    - UBUNTU:
        - terminal 1:  open GQC
    ``` ANT-X_tools/QGroundControl.AppImage ```
        - QGC -> turn on signal transfer to 14445

2. comanion computer 
    - vns login
    - quit current mavros node and open a new terminal. Ssh login, then relaunch the MAVROS node with command


3. run shell script for access to rostopic /actuator_outputs 
    <!-- !! everytime after starting PX4, redo the WHOLE SECTION -->
    - terminal 2
    ```
    ssh roboticlabquad-Precision-3640-Tower
    export ROS_MASTER_URI=http://192.168.1.172:11311
    cd jun_DigitalTwin_ws/dt_test/src/dt_test_pkg/scripts
    ./mavlink_shell_simulation.py 127.0.0.1:14445
    ```

4. run ROS script 
    - terminal 3
    ```
    ssh roboticlabquad-Precision-3640-Tower
    cd ~/dt_test 
    script src/dt_test_pkg/ref/logfile.txt
    export ROS_MASTER_URI=http://192.168.1.172:11311
    source devel/setup.bash
    roslaunch dt_test_pkg default.launch
    ```

5. companion computer 
    - run script to conduct flight task

