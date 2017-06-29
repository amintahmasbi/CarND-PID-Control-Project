# CarND-Controls-PID
PID controller project

---
## Reflections
I ended up implementing the Twiddle algorithm for paramter tuning of the pid controller. For this specific project speed of the vehicle and the cross track error (CTE) which is the distance from center lane were provided through simulator, so the `cost function` is set in such a way that the optimizer (twiddle) should minimize the CTE and maximize speed (using a throttle value of 0.3, the speed error is calculated based on current speed and the highest acheivable speed of 100).
During the calibration of the simulator, the following behaviours were noticed:
1. The changes in the gain of P part of controller (`Kp`) would make the vehicle steer sharper and lower. If this was the only value to tweak, the vehicle would form wave-like maneauvers.
2. The changes in the gain of I part of controller (`Ki`) would make the vehicle to steer more than necessary. Since there was no drift noise in the vehicle, the final value of I part became really close to zero.

3. The changes in the gain of D part of controller (`Kd`) would change the rate of change in the steering. A high value for this gain would make the car to steer sharply back and forth quickly. It was noticed that if the speed error was not a factor in the cost function, a high value of `Kd` would keep the vehicle at center of the lane, but moving really slow because of the rapid steering change.

A sample of the first few steps of calibration among the above mentioned characteristics can be seen at: [click for sample video](https://youtu.be/sQVLuEvRnx0) and their correspoding gain and error values can be seen at [screenshot](./Readme_files/Readme_files/SampleCalibrationValues.png) 

The final parameters were chosen based on the `caliberation tolerance` of `0.1` (which is the sum of gain steps), and termination conditions of either `CTE > 2.3` or `simulation steps > 1000`. Although it is possible to fine-tune the parameters even more, the exisiting values pass the requirments of the project.

***note:*** *during my calibration, I notice that unlike was suggested in the course, the Initial steps for twiddle should not be in the same order of magnitude. It is better to start with a smaller values for I and D parts to avoid Local minimas*  

## Dependencies

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets) == 0.13, but the master branch will probably work just fine
  * Follow the instructions in the [uWebSockets README](https://github.com/uWebSockets/uWebSockets/blob/master/README.md) to get setup for your platform. You can download the zip of the appropriate version from the [releases page](https://github.com/uWebSockets/uWebSockets/releases). Here's a link to the [v0.13 zip](https://github.com/uWebSockets/uWebSockets/archive/v0.13.0.zip).
  * If you run OSX and have homebrew installed you can just run the ./install-mac.sh script to install this
* Simulator. You can download these from the [project intro page](https://github.com/udacity/CarND-PID-Control-Project/releases) in the classroom.

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./pid`. 

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
