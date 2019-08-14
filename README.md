# ackermann2actuator

## Introduction

The ROS navigation stack abstracts robot hardware by sending Twist messages which tell the robot which translational and rotational velocities to apply.
A base controller takes these Twist messages and processes them into odometry data and actuator commands (servos and motors).
The [ros_control](http://wiki.ros.org/ros_control) collection provides multiple controller packages for different kinds of robots such as the [ackermann_steering_controller](http://wiki.ros.org/ackermann_steering_controller) for car-like (ackermann drive) robots. 
These controllers already come with build-in odometry calculations so we do not have to implement that on our own.
However, we do need to map the translational and rotational velocities to actuator commands which the robot actually executes.

This package focuses on finding such a mapping for ackermann drive robots which have two actuators: a motor and a servo.
To abstract away from specific driver implementations, we assume that the motor and servo actuator values are in the range -1 to 1.
Then our goal is to find the parameters of two linear mappings:

    velocity = velocity_gain * motor_actuator_value + velocity_offset
    steering_angle = steering_angle_gain * servo_actuator_value + steering_angle_offset

So the four parameters in question are:

* velocity_gain
* velocity_offset
* steering_angle_gain
* steering_angle_offset
  
which will be determined with linear regression based on a series of individual measuring experiments with your specific robot.
There are two kinds of experiments.
In the first experiment, the car measures the velocity given that it drives with a constant motor actuator value.
In the second experiment, the car measures its turn radius given that it drives along a circle with a constant servo actuator value.
The steering angle is then computed using the turn radius and the wheel base (distance between front and rear axles).

In the following, it is described how these experiments have to be set up.

### Measuring velocity

### Measuring steering angle
