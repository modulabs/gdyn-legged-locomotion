# gdyn-legged-locomotion ![Travis CI status](https://travis-ci.org/rjshim/gdyn-legged-locomotion.svg?branch=master)
4-Legged Robot Project of 강남Dynamics Lab

## Videos
[강남Dynamics Youtube Channel](https://www.youtube.com/channel/UCZDq0-S-_fNhUzGUlN39hfg?view_as=subscriber)

## Install
  ```bash
  # Install ROS
  $ wget https://raw.githubusercontent.com/rjshim/gdyn-legged-locomotion/master/install_ros_kinetic.sh && chmod 755 ./install_ros_kinetic.sh && bash ./install_ros_kinetic.sh

  # Install & Bulid legged robot packages
  $ cd ~/catkin_ws/src
  $ git clone https://github.com/rjshim/gdyn-legged-locomotion.git
  $ sudo apt-get install ros-kinetic-joy \
                 ros-kinetic-joystick-drivers \
                 ros-kinetic-teleop-twist-joy
  $ sudo apt-get install python-pip
  $ pip install cython ds4drv
  $ cd ~/catkin_ws && catkin_make
  ```

## References

### Robot Models
We brought several robot models from the other github sites, however only hyq works in gazebo.

1. [hyq](https://github.com/iit-DLSLab/hyq-description)
2. [laikago, mini cheetah, quadruped](https://github.com/bulletphysics/bullet3/tree/master/examples/pybullet/gym/pybullet_data)
3. [opendog](https://github.com/IanTheEngineer/opendog_robot)
4. [Anymal](https://github.com/leggedrobotics/raisimLib?fbclid=IwAR3bB5PpxuEAAPYyy0g2pWjJZnUjjM8JWR_39wXo_h1kHiNymYhPQieCY0U)

### Papers & Online Materials
Reference papers.

1. [Construct, ROS Lecture](http://www.theconstructsim.com/ros-projects-create-hopper-robot-gazebo-step-step/)
2. [MIT Cheetah 3](https://ieeexplore.ieee.org/document/8593885)
3. [MPC, MIT Cheetah 3](https://www.researchgate.net/publication/330591547_Dynamic_Locomotion_in_the_MIT_Cheetah_3_Through_Convex_Model-Predictive_Control)
4. [Quadratic Programming Library, qpOASES](https://projects.coin-or.org/qpOASES)
