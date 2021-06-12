# CarND-Controls-PID
Self-Driving Car Engineer Nanodegree Program

## Objective
To complete a lap around the simulator track using a PID controller. No tire may leave the drivable portion of the track surface. The car may not pop up onto ledges or roll over any surfaces that would otherwise be considered unsafe (if humans were in the vehicle).

## P,I and D impacts
The first run that was done was a twiddle optimization with intial setting as Kp = 0, Ki = 0, Kd = 0 and dp = \[1,1,1\]
The observations from this were as follows:
* The algoruthm did not converge in 3 hours of running
* Changing P lead to a very noticable change to the track followed by car
* Changing I made car go off track always
* Changing D did not seem to have much effect

The next twiddle run was done with the initial setting of Kp = 0, Ki = 0, Kd = 0 and dp = \[0.1,0.1,0.1\]
The observations were as follows:
* The algorithm converged, but the final parameters led to the car being off-track a little later
* Changing P lead to a very noticable change to the track followed by car
* Changing I made car go off track always
* Changing D did not seem to have much effect

## Twiddle
The Twiddle algorithm was used and was implemented exactly as in the lecture. It was a little difficult as the information had to be kept across runs and the simulator had to be restarted every time. Each twiddle run lasted 1000 timesteps.

These variables were used as flags to ckeck if twiddle had converged and if twiddle should be applied
```
bool twiddle = false;
bool twiddle_complete = false;
```
If twiddle is applied, the follwing code is run
```
if(twiddle){
 if(!pid.twiddle_complete){
   if(n==0){
     n+=1;
   }else{
     if(n>=500){
       crosstrack_error += pow(cte, 2);
     }
   }
   n+=1;
   if(n>1000){
     pid.ContinueTwiddle(crosstrack_error/500);
     crosstrack_error = 0;
     n = 0;
     std::string reset_msg = "42[\"reset\",{}]";
     ws.send(reset_msg.data(), reset_msg.length(), uWS::OpCode::TEXT);
   }
 }else if(!twiddle_complete){
   twiddle_complete = true;
   std::cout << "Kp : " << pid.Kp << " Ki : " << pid.Ki << " Kd : " << pid.Kd << std::endl;
   std::string reset_msg = "42[\"reset\",{}]";
   ws.send(reset_msg.data(), reset_msg.length(), uWS::OpCode::TEXT);
 }
}
```

The error from 500-1000 steps is used to evaluate that particular run in the simulator. `pid.ContinueTwiddle` uses twiddle_var_num to keep track of which variable is currently being changed and twiddle_stage to see in which stage of the algorithm we are. The three stages defined are : 
* p - dp -> run simulator
* if error < min_error -> ( update dp, edit twiddle_var_num ) else -> ( p + 2\*dp -> run simulator )
* if error < min_error -> ( update dp, edit twiddle_var_num ) else -> (p - dp, edit twiddle_var_num)

```
void PID::ContinueTwiddle(double error){
  std::cout << "BEST CTE: " << minimum_crosstrack << " Twiddle Stage: " << twiddle_stage << " Twiddle VAR: " << twiddle_var_num << " dp: " << dp[0] << " " << dp[1] << " " << dp[2] <<  " " << Kp << " " << Ki << " " << Kd
  << std::endl;
  double sum = 0;
  for(double i : dp){
    sum += i;
  }
  if(sum <= tolerance){
    twiddle_complete = true;
    return;
  }
  std::vector<double> p = {Kp, Ki, Kd};

  if(twiddle_stage==0){
    if(twiddle_var_num == 0)
      Kp -= dp[twiddle_var_num];
    else if(twiddle_var_num == 1)
      Ki -= dp[twiddle_var_num];
    else
      Kd -= dp[twiddle_var_num];
    twiddle_stage += 1;
  }else if(twiddle_stage==1){
    if(error < minimum_crosstrack){
      minimum_crosstrack = error;
      dp[twiddle_var_num] *= 1.1;
      twiddle_var_num += 1;
      twiddle_var_num = twiddle_var_num%3;
      twiddle_stage = 0;
    }else{
      if(twiddle_var_num == 0)
        Kp += 2 * dp[twiddle_var_num];
      else if(twiddle_var_num == 1)
        Ki += 2 * dp[twiddle_var_num];
      else
        Kd += 2 * dp[twiddle_var_num];
      twiddle_stage = 2;
    }
  }else if(twiddle_stage==2){
    if(error < minimum_crosstrack){
      minimum_crosstrack = error;
      dp[twiddle_var_num] *= 1.1;
      twiddle_var_num += 1;
      twiddle_var_num = twiddle_var_num%3;
      twiddle_stage = 0;
    }else{
      if(twiddle_var_num == 0)
        Kp -= dp[twiddle_var_num];
      else if(twiddle_var_num == 1)
        Ki -= dp[twiddle_var_num];
      else
        Kd -= dp[twiddle_var_num];
      dp[twiddle_var_num] *= 0.9;
      twiddle_stage = 0;
      twiddle_var_num += 1;
      twiddle_var_num = twiddle_var_num%3;

    }
  }
```

## Throttle

The default throttle was updated to 0.4 and the throttle was adjusted based on the CrossTrack Error as shown below. This led to faster yet compliant track following
```
double throttle = 0.4;

if(fabs(cte) > 0.5){
  throttle = 0.2;
}
if(fabs(cte) > 1.0){
  throttle = 0.1;
}
if(fabs(cte) > 1.5){
  throttle = 0.05;
}
```

## How the parameters were reached

* A initial twiddle run was done with Kp = 0, Ki = 0, Kd = 0 and dp = \[1,1,1\]. This led to a very bad parameter search that did not converge and kept going off-track
* Another twiddle run was done with Kp = 0, Ki = 0, Kd = 0 and dp = \[0.1,0.1,0.1\]. This was done as I thought smaller changes would lead to better changes and tracks. This converged but the path-following was very wavy and it went off-track near the bridge.
* To stop these oscillations I increased the Kd parameter manually while keeping the other same as what were retrieved from twiddle. When I tried Kd = 3, the car completed the lap. This led me to believe that the last twiddle run was stuck at a local minima.
* So another twiddle run was done with Kp = 0, Ki = 0, Kd = 3 and dp = \[0.1,0.1,0.1\]. This converged to a great set of parameters that completed the lap and are listed in main.cpp.

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

