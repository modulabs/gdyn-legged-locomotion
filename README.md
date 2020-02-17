# gdyn-legged-locomotion
4-Legged Robot Project of 강남Dynamics Lab

## Videos
[강남Dynamics Youtube Channel](https://www.youtube.com/channel/UCZDq0-S-_fNhUzGUlN39hfg?view_as=subscriber)

## Dependencies
[spawn_robot_tools](https://bitbucket.org/theconstructcore/spawn_robot_tools/src/master/)
Just download this package in your catkin workspace, and build.

[ROS wrapper of qpOASES](https://github.com/SemRoCo/qpOASES)
Download this package in your catkin workspace, and build.
Install cython using pip, if you are requested.

```bash
$ pip install cython
```
## How to run
There are ros luanch files in legged_robots/legged_robots_sims/launch. 

run_hyq.launch and run_hyq_fixed.launch is valid for now.

```bash
$ roslaunch legged_robots_sims run_hyq.launch
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
