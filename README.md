# Autonomous Car Control System - CMC12_2024

## Description

This project seeks to find a solution to the problem of automatic car driving with a line-following model adapted to the context of a road, using control loops for speed and steering angle. In the implementation, a PID controller with a pre-filter, implemented with Tustin transform was used for angle adjustment and a PF controller for speed adjustment.

The modeling was built for a simulation in which there are two lanes, each populated by cars traveling at a constant speed in the same direction as the controlled vehicle, with the controlled car able to accelerate to a maximum speed, maneuver to the right or left, and brake. To better approximate the real situation of a road, the simulation also considers the existence of wind force, which causes the simulated vehicle to decelerate proportionally to its speed, thereby preventing the vehicle from accelerating only at the beginning of the route, which would not work in a real physical scenario. The information collected to feed the loops are the speed value and the current position reading on the horizontal axis.

## Installation

In order to test and use the algorithms and simulation implemented in the project, it is recommended to clone te repository:

`git clone https://github.com/andresafira/Autonomous_Car_Control`

## Code Description
### Control

In this file, the controllers to be used in the project are stored and initialized, using the discrete implementation technique of the Tustin transform.

### Main Playable

In this file, the simulation of the vehicle controlled by the user occurs, using the directional keys on the keyboard (key arrows).

### Main Simulation

In this file, the simulation of the car's movement is performed, considering only the action of the speed and position controllers of the system in question.

### Main Gridsearch

File responsible for simulating the car's movement under some initial condition (mainly with initial speed 0 or maximum) for various values of xi and omega_n of the position loop movement, in order to search for the pair (xi, omega) that increases the vehicle's responsiveness to lane changes.

## Authors and acknowledgment
André Andrade Gonçalves and Guilherme Saraiva Brasiliense.
