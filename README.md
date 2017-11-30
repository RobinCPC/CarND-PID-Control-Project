# CarND-Controls-PID
Self-Driving Car Engineer Nanodegree Program
[![Udacity - Self-Driving Car NanoDegree](https://s3.amazonaws.com/udacity-sdc/github/shield-carnd.svg)](http://www.udacity.com/drive)

---
## Intro
A PID controller continuously calculates an error value (cross track error in this 
project) as the difference between a desired point (the center of the road) and a 
measured process variable (the center of the car) and applies a correction based 
on proportional, integral, and derivative terms (denoted P, I, and D respectively) 
which give the controller its name. In this project, the goal is to implement a PID 
Controller to drive a car, in the simulator, on the track safely and as fast as possible.

## The effect of P,I,D component in the car control
The control target of this project is to keep the car on the track (in the middle of the road).
Therefore the cross track error (cte) is the distance between the car and the middle of road. 
The effect of each P,I,D component is described in below:

P component is proportional to the cross track error. Larger cte (more faraway from the road) cause larger control output from P component (turn the steer harder).
However, large P gain will cause overshoot and oscillation (the car will drive unstably alone the middle of the road).

D component is the method to solve those issue. D is proportional to the changing (derivative) of cte, so it can reduce the changing rate of cte and prevent the car
from overshoot. 

I component is proportional to the cumulated (integral) cte. It could deal with the system bias (such as steering offset) which cause error keep increase (can't be fixed
by P and D component). 

The below is the implement in my PID.cpp:

``` cpp

void PID::UpdateError(double cte) {
    // update CTE (cross-track error) diff_CTE, int_CTE
    d_error = cte - p_error; // cte - prevouse cte
    p_error = cte;
    i_error += cte;
    return;
}

double PID::TotalError() {
    // calculate PID contorl ouptut for steering angle
    double total = -Kp * p_error -Kd * d_error -Ki * i_error;
    return total;
}

```

## How I tune the final hyperparameters?

I add Twiddle function to my PID class and modified `main.cpp` to accept arguments.
User could pass  4 arguments to set PID component (first three arguments) and, for last argument, set twiddle mode (1 is on, 0 for off).
For example:

`./pid 1.0 0.03 4.0 1` will start twiddle mode and use `P=1.0, I=0.03, D=4.0` as starting values

and

`./pid 1.0 0.03 4.0 0` will run in autonomous mode with `P=1.0, I=0.03, D=4.0`.


After couple hours of tuning in twiddle mode, my final hyperparameters are (Kp, Ki, Kd) = (1.30, 0.01, 6.02). To test, just run with argument:

`./pid`  will run with my final hyperparameters.


Below is original README from Udacity.

---

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

There's an experimental patch for windows in this [PR](https://github.com/udacity/CarND-PID-Control-Project/pull/3)

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

