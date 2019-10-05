# CarND-PID-Control-Project

Self-Driving Car Engineer Nanodegree Program

---
[image0]: ./imgs/Capture2.jpg 

This repository contains implementation of PID controller that derives steering angle for a car given the error from the center of the track as an input


## Background

One of the basic implementation of a control systems in motors, robotics and moving objects in a control system is the Proportional (P), Differential (D), Integral (I), controller(PID). PID controller is the most popular controller and is used in applications across domains. 


## Working of PID Controller

The basic principle of working of a PID controller is a minimization of the total error problem in a given system. This error is minimised through 3 main parameter the each of the P, I and D components contributes in dealing with a certain type of error and all of the 3 completes each other. These components are described as below:

  1. Proportional (P) component:
    Mathematically, the P component compensates for the current error value to get back on track. The value of P component is given by the formula:
    
    Proportional error = -Kp * error
    
    
  2. Differential (D) component:
    Mathematically, the D component compensates for the sudden changes of the error which minimizes the uncertainty in the system. The value of D component is given by the formula:
    
    Diffrential error = -Kd * d(error)/dt
    
    
  3. Integral (D) component:
    Mathematically, the I component compensates for the value of systemic bias error in the system. The value of I component is given by the formula:
    
    Integral error = -Ki * SUM( error)
    
  When all components are used, the mathematical equation is given by:
  
    Error = (-Kp * error) + (-Kd * d(error)/dt) + (Ki * SUM( error))
  
  where, Error is the control input to the system, often known as the **actuator** input.
  

## Project Goal

In this project, a PID controller was implemented to drive a car around circular track having sharp left and right turns with varrying the velocity. The PID aims to keep the car stay in the center of the lane and take smooth left and right turns without running over the edges of the lane.

The simulator used can be found here at [Udacity's self driving car simulator](https://github.com/udacity/self-driving-car-sim/releases). 

The simulator measured the cross track error (cte) between the lateral position of car and the center of the lane. This error was communicated to C++ code with the help of [uWebSockets library](https://github.com/uNetworking/uWebSockets). The cte was then used by the PID controller governing the steering angle of the car.

The final implementation consisted of following major steps:

  1. Calculating the steering angle according to the cte given by implementing the PID controller Algorithm.
  
  2. Setting initial starting values for the PID controller by firstly switching off the I and D components and only the P component was used to drive the car. This was done by setting the Ki and Kd parameters to 0. The car was allowed to drive along the track. The value of Kp was tuned manually to help the car stay on the track as much as possible without much of deviations then selecting intial values for the I and D components accordingly.
  
  3. Optimizing the PID controller parameters using Twiddle
  
  
  6. Gain parameters were then fine-tuned by Twiddle algorithm where the manual tuned values from step 2 were taken as a starting point. Each of the gain parameters Kp, Ki and Kd were tuned one at a time to optimize the algorithm, the car was driven only for 200 time steps after which the simulator was reset to bring back the car in original starting position. Few results obtained while fine tuning are given below:

  Final values of tuned parameters after using Twiddle for parameter optimization  
 **Fine tuning of parameters using twiddle**
  
| parameter | Starting point    | After twiddle   | 
|:---------:|:-----------------:|:---------------:| 
| Kp        | 6.0e-2               | 1.6e-1          | 
| Ki        | 3.0e-4               | 6.76e-4         |
| Kd        | 1.3               | 4.34            |
  -----------------------------------------------------------------------------------

  
## Results

PID controller that is used to derive the steering angles for a car moving on a circular track was implemented successfully. The car could stay close to the center of the lane and take smooth left and right turns along its path.

![][image0]

## Steps for building the project


## Dependencies

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1(mac, linux), 3.81(Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `./install-mac.sh` or `./install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets 
    cd uWebSockets
    git checkout e94b6e1
    ```
    Some function signatures have changed in v0.14.x. See [this PR](https://github.com/udacity/CarND-MPC-Project/pull/3) for more details.
* Simulator. You can download these from the [project intro page](https://github.com/udacity/self-driving-car-sim/releases) in the classroom.

Fellow students have put together a guide to Windows set-up for the project [here](https://s3-us-west-1.amazonaws.com/udacity-selfdrivingcar/files/Kidnapped_Vehicle_Windows_Setup.pdf) if the environment you have set up for the Sensor Fusion projects does not work for this project. There's also an experimental patch for windows in this [PR](https://github.com/udacity/CarND-PID-Control-Project/pull/3).

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./pid`. 

Tips for setting up your environment can be found [here](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/23d376c7-0195-4276-bdf0-e02f1f3c665d)

## Editor Settings

We've purposefully kept editor configuration files out of this repo in order to
keep it as simple and environment agnostic as possible. However, we recommend
using the following settings:

* indent using spaces
* set tab width to 2 spaces (keeps the matrices in source code aligned)

## Code Style

Please (do your best to) stick to [Google's C++ style guide](https://google.github.io/styleguide/cppguide.html).

## Project Instructions and Rubric

Note: regardless of the changes you make, your project must be buildable using
cmake and make!

More information is only accessible by people who are already enrolled in Term 2
of CarND. If you are enrolled, see [the project page](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/f1820894-8322-4bb3-81aa-b26b3c6dcbaf/lessons/e8235395-22dd-4b87-88e0-d108c5e5bbf4/concepts/6a4d8d42-6a04-4aa6-b284-1697c0fd6562)
for instructions and the project rubric.

## Hints!

* You don't have to follow this directory structure, but if you do, your work
  will span all of the .cpp files here. Keep an eye out for TODOs.

## Call for IDE Profiles Pull Requests

Help your fellow students!

We decided to create Makefiles with cmake to keep this project as platform
agnostic as possible. Similarly, we omitted IDE profiles in order to we ensure
that students don't feel pressured to use one IDE or another.

However! I'd love to help people get up and running with their IDEs of choice.
If you've created a profile for an IDE that you think other students would
appreciate, we'd love to have you add the requisite profile files and
instructions to ide_profiles/. For example if you wanted to add a VS Code
profile, you'd add:

* /ide_profiles/vscode/.vscode
* /ide_profiles/vscode/README.md

The README should explain what the profile does, how to take advantage of it,
and how to install it.

Frankly, I've never been involved in a project with multiple IDE profiles
before. I believe the best way to handle this would be to keep them out of the
repo root to avoid clutter. My expectation is that most profiles will include
instructions to copy files to a new location to get picked up by the IDE, but
that's just a guess.

One last note here: regardless of the IDE used, every submitted project must
still be compilable with cmake and make./

## How to write a README
A well written README file can enhance your project and portfolio.  Develop your abilities to create professional README files by completing [this free course](https://www.udacity.com/course/writing-readmes--ud777).

