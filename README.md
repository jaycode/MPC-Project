# MPC for Self-Driving Car
Self-Driving Car Engineer Nanodegree Program

---

In this project, we use Model Prediction Controller (MPC) to control the movement of a simulation car so it may follow a given path.

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
* [uWebSockets](https://github.com/uWebSockets/uWebSockets) == 0.14, but the master branch will probably work just fine
  * Follow the instructions in the [uWebSockets README](https://github.com/uWebSockets/uWebSockets/blob/master/README.md) to get setup for your platform. You can download the zip of the appropriate version from the [releases page](https://github.com/uWebSockets/uWebSockets/releases). Here's a link to the [v0.14 zip](https://github.com/uWebSockets/uWebSockets/archive/v0.14.0.zip).
  * If you have MacOS and have [Homebrew](https://brew.sh/) installed you can just run the ./install-mac.sh script to install this.
* Fortran Compiler
  * Mac: `brew install gcc` (might not be required)
  * Linux: `sudo apt-get install gfortran`. Additionall you have also have to install gcc and g++, `sudo apt-get install gcc g++`. Look in [this Dockerfile](https://github.com/udacity/CarND-MPC-Quizzes/blob/master/Dockerfile) for more info.
* [Ipopt](https://projects.coin-or.org/Ipopt)
  * Mac: `brew install ipopt`
  * Linux
    * You will need a version of Ipopt 3.12.1 or higher. The version available through `apt-get` is 3.11.x. If you can get that version to work great but if not there's a script `install_ipopt.sh` that will install Ipopt. You just need to download the source from the Ipopt [releases page](https://www.coin-or.org/download/source/Ipopt/) or the [Github releases](https://github.com/coin-or/Ipopt/releases) page.
    * Then call `install_ipopt.sh` with the source directory as the first argument, ex: `bash install_ipopt.sh Ipopt-3.12.1`. 
  * Windows: TODO. If you can use the Linux subsystem and follow the Linux instructions.
* [CppAD](https://www.coin-or.org/CppAD/)
  * Mac: `brew install cppad`
  * Linux `sudo apt-get install cppad` or equivalent.
  * Windows: TODO. If you can use the Linux subsystem and follow the Linux instructions.
* [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page). This is already part of the repo so you shouldn't have to worry about it.
* Simulator. You can download these from the [releases tab](https://github.com/udacity/CarND-MPC-Project/releases).
* Not a dependency but read the [DATA.md](./DATA.md) for a description of the data sent back from the simulator.


## Basic Build Instructions


1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./mpc`.

## Implementation

### The Model

The model uses the following states:

- x: Coordinate on the x-axis, assuming local coordinate where the car is positioned at 0,0. x-axis is where the car is facing.
- y: Coordinate on the y-axis.
- psi: Orientation.
- v: Speed of the car.
- cte: Cross Track Error.
- epsi: Orientation Error.

This motion model is based on CRTV (Constant Rate of Turn & Velocity). The state transition between steps is defined as:

```
x_[t+1] = x[t] + v[t] * cos(psi[t]) * dt
y_[t+1] = y[t] + v[t] * sin(psi[t]) * dt
psi_[t+1] = psi[t] + v[t] / Lf * delta[t] * dt
v_[t+1] = v[t] + a[t] * dt
cte[t+1] = f(x[t]) - y[t] + v[t] * sin(epsi[t]) * dt
epsi[t+1] = psi[t] - psides[t] + v[t] * delta[t] / Lf * dt

```

`delta`/steering angle and `a`/throttle are the actuators used by this model.

## Timestep Length and Elapsed Duration (N & dt)

The final model uses `N = 7` and `dt = 0.1`. The value of N was chosen by trying out several other values and measuring their effects, that is, at speed 40 and dt 0.15:
- Setting N to 20 causes the car to turn aggresively.
- Setting N to 5 causes the car to turn slowly. It scratches the sidewalk at times.
- Setting N to 7 causes the car to turn a bit more aggresively until it got stuck at a sidewalk. At max speed 80, and several adjustments however, it works perfectly for the car.

The setting of elapsed duration `dt` was chosen based on 100 miliseconds latency per step.

## Polynomial Fitting and MPC Preprocessing

- The waypoints were preprocessed by converting them into local positions based on the car's location.

## Model Predictive Control with Latency

As mentioned above, using `dt` of 0.1 ensured the system to perform optimally. To gain an optimal control of the car, we also implemented the following:

- I adjusted the input states for latency i.e. using the car's position in the next predicted 100 milliseconds instead of the current position. This helped improving the driving performance.
- Increase the importance of epsilon a.k.a. orientation difference between the waypoints and predicted states.
- Decrease the importance of Cross Track Error, which, accompanied by the delta factor adjustment, helped the car in dealing with sharper turns.
- Dynamic speed reference was implemented by looking at sharpness of turn. When the turn is sharp enough (which value found by experimenting), decrease the speed reference.

## Result

Video is available [here](https://youtu.be/AYXNlmw3f48).

## Self Drifting Car

I also tried running the car on 90-ish mph without braking [here](https://youtu.be/KJmkoU7BkaM).