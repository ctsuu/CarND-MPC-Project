# CarND-Controls-MPC
Self-Driving Car Engineer Nanodegree Program

---
## Objective
MPC(Model Predictive Control) is another important control method in the self-driving car tool box. MPC has the ability to anticipate future events and can take control actions accordingly. PID and LQR controllers do not have this predictive ability. In this project, we will drive on the same track as PID controller project, but I expect much faster speed and smoother driving, especially at hard corners. To simulate the real driving experence, we must to handle 100ms latency, it is like 10 Hz update rate. 

## The Model
The vehicle model is required for implement MPC. I am using simplified bicycle model, such as: 
```
      // The length from front wheel to CoG Lf = 2.67m;
      
      // x_[t+1] = x[t] + v[t] * cos(psi[t]) * dt
      // y_[t+1] = y[t] + v[t] * sin(psi[t]) * dt
      // psi_[t+1] = psi[t] + v[t] / Lf * delta[t] * dt
      // v_[t+1] = v[t] + a[t] * dt
      // cte[t+1] = f(x[t]) - y[t] + v[t] * sin(epsi[t]) * dt
      // epsi[t+1] = psi[t] - psides[t] + v[t] * delta[t] / Lf * dt
```
The vehicle state has 6 elements, x position, y position, heading psi, speed, cross track error, heading error. 
The state can be in globle coordinations system or local vehicle coordination. 

## Timestep and Duration

Model prediction is based on how many timestep projected into future. Depended on computing power and results, I tested 25 timesteps, each step took 0.02 sec seems match with other parameters in my setting. It yeilds 25*0.02 = 0.5 sec into the future. It is enough to overcome 0.1 sec latency and fairly smooth drive behavior. 

## Transforming
```
{"ptsx":[70.40827,61.24355,45.30827,36.03354,15.90826,5.64827],"ptsy":[157.101,155.4194,151.201,148.141,140.121,135.321],"psi_unity":4.534025,"psi":3.319957,"x":62.92501,"y":157.1006,"steering_angle":-0.01871635,"throttle":1,"speed":54.3903}
```
The waypoints and car states come from the simulator are presented in global map coordinate. I used two steps to transform them into vehicle coordinate for display and controls. 

First, I create a translation matrix from car location (px, py) to (0,0)
```
  Eigen::MatrixXd T(3, 3);
  T << 1, 0, -px,
       0, 1, -py,
       0, 0,  1;
```
Second, I create a rotation matrix from current (psi) to (0)
```
  Eigen::MatrixXd R(3, 3);
  R << cos(-psi), -sin(-psi), 0,
       sin(-psi), cos(-psi), 	0,
       0,   	  0, 		1;

```

Then, create Waypoint Matrix and vehicle coordinate Matrix 
```  
  Eigen::MatrixXd wps(3, ptsx.size());
  ... ...
  
  Eigen:: MatrixXd car(3, ptsx.size());
  car = R * T * wps;
```
The trick is take current car states, it makes the waypoints display stick to the track center line.   

## Polynomial Fitting
After the transform, we get 6-7 waypoints in vehicle coordinate. We can just plot them on simulator, or do a better job to fit into polynomial curve. This will help us project further, and make smoother curve. 

I am fitting 3 order polynomial, and verify it with 22 points calculated by the formular, with 2 points in the past, 20 points in future.   
```
          // Fit polynomial with order 3
                   
          auto coeffs = Eigen::VectorXd(4);
          
          coeffs = polyfit(next_x, next_y, 3);

          next_x_vals.clear();
          next_y_vals.clear();
           
          for (double x = -10; x <= 80; x += 4.0) {
            // use `polyeval` to evaluate the x values.
    	    auto ref = polyeval(coeffs, x);
            next_x_vals.push_back(x);
            next_y_vals.push_back(ref);
          }         
```
The coeffs is saved for next MPC solving as well. 

## MPC Preprocessing

Calculate cte in vehicle coordinates is simpler because car always at (0,0), and psi is 0. 
```
          // The cross track error is calculated by evaluating at polynomial at x, f(x)
          // and subtracting y.

	  double cte = polyeval(coeffs, 0) - 0;
          //std::cout << "CTE at x = 0 point" << std::endl;
          //std::cout << cte << std::endl;

 
	  // Due to the sign starting at 0, the orientation error is -f'(x).
	  // derivative of coeffs[0] + coeffs[1] * x -> coeffs[1]
	  double epsi = -atan(coeffs[1]);

          // Six elements car state
          Eigen::VectorXd car_state(6); 
          // px, py, psi, speed, cte, epsi
          car_state << 0, 0, 0, v, cte, epsi; 
          
          // Solve the path and actuation commands
          auto act = mpc.Solve(car_state, coeffs, limit);
```
Instead of driving one speed for the whole track, I also added speed limits and pass it into the mpc.Solve function. 
For long stratch, the car can go as fast as possible, like 85mph, and for sharp turns, go with 30mph.  
```
          fg[0] += CppAD::pow(vars[v_start + i] - limit, 2);
```

## MPC with Latency

Latency plays a huge rule in controller. If the system is fast enough, the actual latency is the time required for all calculations, it is about 0.02-0.03 sec on my setting. The driving is smooth and easy. When added the artifical latency 0.1 sec, it is about 5 times longer the computer to wait for the next state update, the transformed trajectory fly all over the place, the controller try to shot the moving target, and result a lot of overshooting. But we can't avoid the latency, as I increase the timesteps from 25 to 50, the computing time is about 0.1 sec. 

My approach is to look into the future steps, make the actuator move based on most likely future moves.  

## Post Speed limits
## Fine Tune Cost
## Reflection

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
* [Ipopt](https://projects.coin-or.org/Ipopt)
  * Mac: `brew install ipopt --with-openblas`
  * Linux
    * You will need a version of Ipopt 3.12.1 or higher. The version available through `apt-get` is 3.11.x. If you can get that version to work great but if not there's a script `install_ipopt.sh` that will install Ipopt. You just need to download the source from [here](https://www.coin-or.org/download/source/Ipopt/).
    * Then call `install_ipopt.sh` with the source directory as the first argument, ex: `bash install_ipopt.sh Ipopt-3.12.1`. 
  * Windows: TODO. If you can use the Linux subsystem and follow the Linux instructions.
* [CppAD](https://www.coin-or.org/CppAD/)
  * Mac: `brew install cppad`
  * Linux `sudo apt-get install cppad` or equivalent.
  * Windows: TODO. If you can use the Linux subsystem and follow the Linux instructions.
* [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page). This is already part of the repo so you shouldn't have to worry about it.
* Simulator. You can download these from the [releases tab](https://github.com/udacity/CarND-MPC-Project/releases).



## Basic Build Instructions


1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./mpc`.

## Tips

1. It's recommended to test the MPC on basic examples to see if your implementation behaves as desired. One possible example
is the vehicle starting offset of a straight line (reference). If the MPC implementation is correct, after some number of timesteps
(not too many) it should find and track the reference line.
2. The `lake_track_waypoints.csv` file has the waypoints of the lake track. You could use this to fit polynomials and points and see of how well your model tracks curve. NOTE: This file might be not completely in sync with the simulator so your solution should NOT depend on it.
3. For visualization this C++ [matplotlib wrapper](https://github.com/lava/matplotlib-cpp) could be helpful.

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
of CarND. If you are enrolled, see [the project page](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/f1820894-8322-4bb3-81aa-b26b3c6dcbaf/lessons/b1ff3be0-c904-438e-aad3-2b5379f0e0c3/concepts/1a2255a0-e23c-44cf-8d41-39b8a3c8264a)
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
