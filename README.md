# gdyn-legged-locomotion ![Travis CI status](https://travis-ci.org/modulabs/gdyn-legged-locomotion.svg?branch=master)
4-Legged Robot Project of 강남Dynamics Lab

## Videos
[강남Dynamics Youtube Channel](https://www.youtube.com/channel/UCZDq0-S-_fNhUzGUlN39hfg?view_as=subscriber)


## How to Install
### Install ROS Melodic
Follow the ros wiki or use below script if you want.
```bash
# Install ROS Melodic
$ wget https://raw.githubusercontent.com/modulabs/gdyn-legged-locomotion/master/install_ros_melodic.sh \
  && chmod 755 ./install_ros_melodic.sh \
  && bash ./install_ros_melodic.sh
```

### Install legged robot packages and their dependencies
```bash
# Download & Bulid legged robot packages
$ cd ~/catkin_ws/src
$ git clone https://github.com/modulabs/gdyn-legged-locomotion.git
$ sudo apt-get install ros-melodic-joy \
                       ros-melodic-joystick-drivers \
                       ros-melodic-teleop-twist-joy
$ sudo apt-get install python-pip
$ pip install cython wheel ds4drv
$ cd ~/catkin_ws && catkin_make
```

## How to Run
```bash
# Run Gazebo simulation (run_hyq.launch and run_hyq_fixed.launch are valid for now.)
$ roslaunch legged_robot_gazebo run_hyq.launch

# Run Rviz
$ roslaunch legged_robot_description hyq_view.launch

# Run GUI
$ rosrun legged_robot_gui legged_robot_gui

# Run Teleop Keyboard
$ rosrun legged_robot_teleop legged_robot_teleop_keyboard

# Run Teleop Joystick (PS4)
$ sudo ds4drv # Detailed Usage (https://github.com/RetroPie/RetroPie-Setup/wiki/PS4-Controller)
$ roslaunch teleop_twist_joy teleop.launch
$ rosrun legged_robot_teleop legged_robot_teleop_joystick
```

## References

### Robot Models
We brought several robot models from the other github sites, however only hyq works in gazebo.

1. [hyq](https://github.com/iit-DLSLab/hyq-description)
2. [laikago, mini cheetah, quadruped](https://github.com/bulletphysics/bullet3/tree/master/examples/pybullet/gym/pybullet_data) (not yet)
3. [opendog](https://github.com/IanTheEngineer/opendog_robot) (not yet)
4. [Anymal](https://github.com/leggedrobotics/raisimLib?fbclid=IwAR3bB5PpxuEAAPYyy0g2pWjJZnUjjM8JWR_39wXo_h1kHiNymYhPQieCY0U) (not yet)

### Papers & Online Materials
Reference papers.

1. [Construct, ROS Lecture](http://www.theconstructsim.com/ros-projects-create-hopper-robot-gazebo-step-step/)
2. [MIT Cheetah 3](https://ieeexplore.ieee.org/document/8593885)
3. [MPC, MIT Cheetah 3](https://www.researchgate.net/publication/330591547_Dynamic_Locomotion_in_the_MIT_Cheetah_3_Through_Convex_Model-Predictive_Control)
4. [Quadratic Programming Library, qpOASES](https://projects.coin-or.org/qpOASES)
