# PID Controller

[![Udacity - Self-Driving Car NanoDegree](https://s3.amazonaws.com/udacity-sdc/github/shield-carnd.svg)](http://www.udacity.com/drive)

---
### Goals
The goal of the project is to build a Proportional-Integral-Derivative (PID) control for driving the autonomous car around a predefined track. PID is the most common control algorithm used in industry and has three basic coefficients; proportional, integral and derivative which are varied to get optimal response.

### Environment
The udacity's car simulator provides the cross track error (CTE) and the velocity (mph) in order to compute the appropriate steering angle. The goal is to use these values to build a PID controller in order to safely drive the car around the desgined track.

![image1](images/Image0.png "Udacity Simiulator")

#### CTE: The cross track error flowing into the PID controller from sensors.
#### Kp, Ki, and Kd are Proportional, Integral and Derivative parameters respectively.
#### p_error, i_error, and d_error are proportional, integral and derivative error components respectively.

### Desgin and Implementation
Initial values of PID are imperically found by running the car and observing the behavior. The approach for finding the values is:

1. First step is to set Ki and Kd values to zero, and only use Kp value until we reach a managable oscillation i.e. car oscillates but stays within the lane for majority of the track.

[![PID Controller with accepted Kp value](images/Image1.png)](https://youtu.be/BVALgxNa6v0)


2. Second step is to find the Kd(Derivative) value to minimize the oscillation so that cars stays within the lane.

3. Third step is to find a good Ki value to reduce the steady-state error.

The initial values selected for Kp,Ki,Kd are [0.08,0.0005,7.5] respectively.

### Optimization using Twiddle
In order to further optimize the initial PID values, an algorithm named Twiddle is used.  This was designed by Dr. Sebastian Thrun, Udacity. 

Please see the below video for details.

[![Twiddler by Sebastian Thrun, Udacity](images/Image3.png)](https://www.youtube.com/watch?v=2uQ2BSzDvXs)


#### Twiddle Algorithm
![image2](images/Image2.png)

---
#### Twiddle Implementation and Intgration:
I added a new member function to PID controller for Twiddler. The main.cpp is altered to integrate the Twiddle algorithm. The main.cpp runs the Twiddle for 4 times to find the optimized value. It also controls the timestep/frames before accepting the CTE. The main.cpp also implements a second Twiddle to control the car speed based on the difference between the set point speed and current speed. This allows the car to make a safe run around the track. The steering Twiddle turns itself off after running it for 4 times.

![image5](images/Image5.png)

### Final optimized values: [0.09, 0.0005, 7.22648]


---
Final Video with tuned PID controller:
[![PID Controller after Twiddle](images/Image4.png)](https://youtu.be/Gte-sV17Pvg)


## Basic Build Instructions
1. Clone this repo and from the repo root run ./m.sh 
#### or 
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./pid`. 


Tips for setting up your environment can be found [here](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/23d376c7-0195-4276-bdf0-e02f1f3c665d)

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
* Simulator. You can download these from the [project intro page](https://github.com/udacity/self-driving-car-sim/releases) in the classroom.

## Code Style
[Google's C++ style guide](https://google.github.io/styleguide/cppguide.html).
