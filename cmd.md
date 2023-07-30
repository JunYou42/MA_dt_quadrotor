# UNI COMPUTER CMD SUMMARY


make px4_sitl_default gazebo-classic_iris__windy
export PX4_SIM_SPEED_FACTOR=8

## communication
### ground pc
sudo ufw disable
ssh roboticlabquad-Precision-3640-Tower

export ROS_MASTER_URI=http://192.168.1.172:11311
rosrun rospy_tutorials talker.py




## PX4
- INSTALL
'''
sudo apt-get install libgstreamer-plugins-base1.0-dev gstreamer1.0-plugins-bad gstreamer1.0-plugins-base gstreamer1.0-plugins-good gstreamer1.0-plugins-ugly -y
'''


roslaunch mavros px4.launch fcu_url:="udp://:14540@127.0.0.1:14557"
roslaunch mavros px4.launch fcu_url:="udp://:14540@192.168.0.101:14557"



---------------------
## RUN diy drone on school PC
1. connect to drone
WINDOWS:
    - open Vicon system -> select DIYDRONE

WEATHER STATION
    - connect and switch to weather log mode

UBUNTU:
    - terminal 1: QGC -> turn on signal transfer to 14445
    ANT-X_tools/QGroundControl.AppImage

2. comanion computer  !!
vns login
ssh
re-launch mavros 

3. run shell script for rostopic /actuator_outputs 
    <!-- !! everytime after starting PX4, redo the WHOLE SECTION -->
    - terminal 2
    ssh roboticlabquad-Precision-3640-Tower
    export ROS_MASTER_URI=http://192.168.1.172:11311
    cd jun_DigitalTwin_ws/dt_test/src/dt_test_pkg/scripts
    
    ./mavlink_shell_simulation.py 127.0.0.1:14445
    <!-- ./mavlink_shell.py 127.0.0.1:14445 -->


4. run ROS script 
    <!-- - terminal 2 & 3 & 4
    ssh roboticlabquad-Precision-3640-Tower
    export ROS_MASTER_URI=http://192.168.1.172:11311
    1. roslaunch vrpn_client_ros sample.launch server:=192.168.1.100  (vrpn server IP)
    2. 
    roslaunch mavros px4.launch gcs_url:=udp://@192.168.1.101:14550
    3. rosrun topic_tools relay /vrpn_client_node/Drone01/pose /mavros/vision_pose/pose -->

    - terminal 3
    **CHECK TOPIC NAME FIRST**
    ssh roboticlabquad-Precision-3640-Tower
    cd ~/jun_DigitalTwin_ws/dt_test 
    script src/dt_test_pkg/ref/0623_log_newactuator.txt
    <!-- BE CLEARED EVERY TIME AFTER INITIALIZE !!!! -->
    export ROS_MASTER_URI=http://192.168.1.172:11311
    source devel/setup.bash
    roslaunch dt_test_pkg default.launch

5. companion computer 
rosrun script


>  scp src/dt_test_pkg/src/flight_4real_spin.cpp ubuntu@192.168.1.172:~/catkin_ws/src/offboard/src/flight_4real_spin.cpp
---------------------
## RUN simulation on school PC

cd ~/jun_DigitalTwin_ws/firmware_ws/PX4-Autopilot
make px4_sitl_default gazebo





roslaunch mavros px4.launch fcu_url:="udp://:14540@127.0.0.1:14557"

ANT-X_tools/QGroundControl.AppImage

cd jun_DigitalTwin_ws/dt_test/src/dt_test_pkg/scripts
./mavlink_shell_simulation.py 127.0.0.1:14445


cd ~/jun_DigitalTwin_ws/dt_test
script src/dt_test_pkg/ref/0621_program_debug.txt

source de
script src/dt_test_pkg/ref/0621_program_debug.txt
roslaunch dt_test_pkg default.launch


---------------------
## RUN ANTX on school PC (QGC IN UBUNTU ONLY)
1. connect to drone
WINDOWS:
    - open Vicon system

UBUNTU
    - terminal 1
    matlab19b

    - terminal 2
    ANT-X_tools/QGroundControl.AppImage

WEATHER STATION
    - connect and switch to weather log mode

2. run shell script for rostopic /actuator_outputs
    - terminal 3
    cd jun_DigitalTwin_ws/dt_test/src/dt_test_pkg/scripts
    ./mavlink_shell.py 127.0.0.1:14445

3. run local script 
**CHECK TOPIC NAME FIRST**
cd ~/jun_DigitalTwin_ws/dt_test
source de ... 
roslaunch dt_test_pkg default.launch


<!-- roslaunch mavros px4.launch fcu_url:="udp://:14540@192.168.1.144:14557" -->


---------------------
## ROSIP
export ROS_IP=192.168.1.101  				(LOCAL
export ROS_MASTER_URI=http://192.168.1.172:11311	(DIY DRONE
export ROS_MASTER_URI=http://127.0.0.1:11311
export ROS_MASTER_URI=http://192.168.1.144:11311	(ANTX8

## QGC
127.0.0.1:14550 -> 127.0.0.1:14445
---------------------

ssh roboticlabquad-Precision-3640-Tower
export ROS_MASTER_URI=http://192.168.1.144:11311
cd jun_DigitalTwin_ws/dt_test/
source devel/setup.bash 