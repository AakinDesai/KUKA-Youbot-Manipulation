# KUKA’s Youbot Manipulation

## Summary

The plans a trajectory for the end-effector of the youBot mobile manipulator (a mobile base with four mecanum wheels and a 5R robot arm), performs odometry as the chassis moves, and performs feedback control to drive the youBot to pick up a block at a specified location, carry it to a desired location, and put it down. 

The process followed in the software to generate the configuration for the bot is :-

**TrajectoryGenerator:** In this step, the function provides the reference trajectory for the end-effector with the input of the cube’s initial and final trajectories and the initial end-effector state. It outputs a csv of end-effector along with the gripper state.

**NextState:** In this step, it provides the configuration of the robot (position, wheel angles and joint angles), provided the initial configuration, joint and wheel speeds. It outputs a csv of the next state (after a small-time step) configuration of the robot.

**FeedbackControl:** In this step, the function calculates the kinematic task-space feedforward plus feedback control law. Given the desired and actual end-effector configuration it calculates the error and proposes a twist to follow for achieving the desired configuration. It outputs a csv of the error file and the controls of wheels and joints.

Then, it runs a loop over the NextState and FeedbackControl functions to obtain the csv for animation.

The last part of the code is to plot the error vs time and the error plots.

#### Result

<img src="Media/Error Plot.png" width="600">