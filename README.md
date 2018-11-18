# CarND-Path-Planning-Project
Self-Driving Car Engineer Nanodegree Program
   
### Goals
In this project your goal is to safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit. You will be provided the car's localization and sensor fusion data, there is also a sparse map list of waypoints around the highway. The car should try to go as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible, note that other cars will try to change lanes too. The car should avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another. The car should be able to make one complete loop around the 6946m highway. Since the car is trying to go 50 MPH, it should take a little over 5 minutes to complete 1 loop. Also the car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 10 m/s^3.

### Results

The car is able to drive >7 miles without incident and maintain the speed ~45 miles per hour according to speed limit "50 mph". Along the drive, max Acceleration and Jerk are not Exceeded, and car does not have collisions.


![7miles](./data/7miles.png)

![4.5 miles](./data/4_5miles.png)



The car stays in its lane, but it will change lane if it can achieve higher speed in another lane.
here is an example of changing lane because of slow vehcile in front of it.


![changelane](./data/change_lane_at_lowspeed1.png))




### Implementations

#### 1. get another vehicles information from sensor fusion data, and add them into vehciles vector

                for(int i = 0; i < sensor_fusion.size();i++)
                {
                   ...
                   vehicles.push_back(new_vehicle);
                }


#### 2. check if there is any vechile closeby according to vehciles vector, and look far away for the closeby vechiles if it drive fast. 

                double distance_threshold = 30 * (car_speed / 20.0 );
                for (int i=0; i < vehicles.size();i++)
                {
                   ...

                   if ((check_car_s > car_s) && ((check_car_s - car_s) < distance_threshold))
                   {
                        too_close_[l] = true;
                    }
                }


#### 3. set a lanes status according to step 2 results into a 3 bit format. the left most bit represents left lane, and right most bit represents right lane. center bit represents current lane.

                for (int l=0; l < 3;l++)
                {
                   ...
                                if (too_close_[l] == true )
                                {
                                        if ( l == (lane - 1))// left lane occupied
                                        {
                                                lanes_status  |= 0x04; // 0100
                                        }
                                        else if (l == (lane + 1)) // right lane occupied
                                        {
                                                lanes_status  |= 0x01; // 0001
                                        }
                                        else if ( l == lane )
                                        {
                                                lanes_status  |= 0x02; // 0010
                                        }

                                        if  ( lane == 0)// left lane occupied
                                        {
                                                lanes_status  |= 0x04; // 0100
                                        }
                                        else if  ( lane == 2) // right lane occupied
                                        {
                                                lanes_status  |= 0x01; // 0001
                                        }
                                }
                }


#### 4. change lane if there is a vehcile close in front of us. check left lane first. check right lane if left lane is not available.  

                if ((prev_lanes_status_ != lanes_status ) ){
                        prev_lanes_status_ = lanes_status;

                        // my lane 0000 0010
                        if ( lanes_status & 2 )
                        {
                                // car in front of us
                                if (( lanes_status & 4 ) == 0 ) // check left lane
 
                                    lane = lane - 1;
                                }
                                else if (( lanes_status & 1 ) == 0 ) // check right lane
                                {
                                    lane = lane + 1;
                                }
                        }
                }



#### 5. however, changing lane in high speed is dangerous, and the vehcile exceeds speed limit when it changes lane

![original VGG](./data/change_lane_at_highspeed1.png))

![original VGG](./data/change_lane_at_highspeed2.png))


#### 6. slow down and then change lane to make our drive in safer way. we introduce a plan_to_change_lane flag, so we have one more state "plan to change lane" in state machine. the vehicle will slow down and then change lane.

                if ((prev_lanes_status_ != lanes_status ) || (plan_to_change_lane == true)){

                        prev_lanes_status_ = lanes_status;

                        // my lane 0000 0010
                        if ( lanes_status & 2 )
                        {
                                // car in front of us
                                if (( lanes_status & 4 ) == 0 ) // check left lane
                                {
                                        if (( car_speed < max_speed_for_changing_lane) || ( dangerous_close == true) )
                                        {       // change to left lane
                                                lane = lane - 1;
                                                plan_to_change_lane = false;
                                                cout << " CHANGE to LEFT lane" << " car speed : " << car_speed<<endl;
                                        }
                                        else
                                        {       // too fast to change lane. try slow down first to change lane
                                                cout << " Plan to change to Left lane"<< " car speed : " << car_speed <<endl;
                                                plan_to_change_lane = true;
                                        }

                                }
                                else if (( lanes_status & 1 ) == 0 ) // check right lane
                                {
                                        if (( car_speed < max_speed_for_changing_lane) || ( dangerous_close == true) )
                                        {       // change to right lane
                                                lane = lane + 1;
                                                plan_to_change_lane = false;
                                                cout << " CHANGE to RIGHT lane"<< " car speed : " << car_speed <<endl;
                                        }
                                        else
                                        {       // too fast to change lane. try slow down first to change lane
                                                cout << " Plan to change to RIGHT lane"<< " car speed : " << car_speed <<endl;
                                                plan_to_change_lane = true;
                                        }
                                }
                                //}
                                else
                                {       // slow down
                                        cout << " Keep lane, Slow down" <<endl;
                                        plan_to_change_lane = false;
                                }
                        }
                        else{
                                // no car in front of us, no plan to change lane
                                plan_to_change_lane = false;
                        }

                }




#### 7. as below diagram, the vechile changes lane safely as below diagram

![original VGG](./data/change_lane_at_lowspeed1.png))

![original VGG](./data/change_lane_at_lowspeed2.png))

![original VGG](./data/change_lane_at_lowspeed3.png))


#### 8. the vehcile also accelerates if its speed is < 49.5 mph or no plan to change lane. it will slow down if it is too close to the vehcile in the same lane.

                if(too_close)
                {
                        // Dump info

                        ref_vel-=0.224;
                        //cout << "  Slow down ref_vel:" << ref_vel <<endl;

                }
                else if(( ref_vel < 49.5 ) && ( plan_to_change_lane == false ) )
                {
                        ref_vel+=0.224;
                        //cout << "  Speedup ref_vel:" << ref_vel <<endl;
                }


### Simulator.
You can download the Term3 Simulator which contains the Path Planning Project from the [releases tab (https://github.com/udacity/self-driving-car-sim/releases/tag/T3_v1.2).


#### The map of the highway is in data/highway_map.txt
Each waypoint in the list contains  [x,y,s,dx,dy] values. x and y are the waypoint's map coordinate position, the s value is the distance along the road to get to that waypoint in meters, the dx and dy values define the unit normal vector pointing outward of the highway loop.

The highway's waypoints loop around so the frenet s value, distance along the road, goes from 0 to 6945.554.

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./path_planning`.

Here is the data provided from the Simulator to the C++ Program

#### Main car's localization Data (No Noise)

["x"] The car's x position in map coordinates

["y"] The car's y position in map coordinates

["s"] The car's s position in frenet coordinates

["d"] The car's d position in frenet coordinates

["yaw"] The car's yaw angle in the map

["speed"] The car's speed in MPH

#### Previous path data given to the Planner

//Note: Return the previous list but with processed points removed, can be a nice tool to show how far along
the path has processed since last time. 

["previous_path_x"] The previous list of x points previously given to the simulator

["previous_path_y"] The previous list of y points previously given to the simulator

#### Previous path's end s and d values 

["end_path_s"] The previous list's last point's frenet s value

["end_path_d"] The previous list's last point's frenet d value

#### Sensor Fusion Data, a list of all other car's attributes on the same side of the road. (No Noise)

["sensor_fusion"] A 2d vector of cars and then that car's [car's unique ID, car's x position in map coordinates, car's y position in map coordinates, car's x velocity in m/s, car's y velocity in m/s, car's s position in frenet coordinates, car's d position in frenet coordinates. 

## Details

1. The car uses a perfect controller and will visit every (x,y) point it recieves in the list every .02 seconds. The units for the (x,y) points are in meters and the spacing of the points determines the speed of the car. The vector going from a point to the next point in the list dictates the angle of the car. Acceleration both in the tangential and normal directions is measured along with the jerk, the rate of change of total Acceleration. The (x,y) point paths that the planner recieves should not have a total acceleration that goes over 10 m/s^2, also the jerk should not go over 50 m/s^3. (NOTE: As this is BETA, these requirements might change. Also currently jerk is over a .02 second interval, it would probably be better to average total acceleration over 1 second and measure jerk from that.

2. There will be some latency between the simulator running and the path planner returning a path, with optimized code usually its not very long maybe just 1-3 time steps. During this delay the simulator will continue using points that it was last given, because of this its a good idea to store the last points you have used so you can have a smooth transition. previous_path_x, and previous_path_y can be helpful for this transition since they show the last points given to the simulator controller with the processed points already removed. You would either return a path that extends this previous path or make sure to create a new path that has a smooth transition with this last path.

## Tips

A really helpful resource for doing this project and creating smooth trajectories was using http://kluge.in-chemnitz.de/opensource/spline/, the spline function is in a single hearder file is really easy to use.

---

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
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `install-mac.sh` or `install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets 
    cd uWebSockets
    git checkout e94b6e1
    ```

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


## Call for IDE Profiles Pull Requests

Help your fellow students!

We decided to create Makefiles with cmake to keep this project as platform
agnostic as possible. Similarly, we omitted IDE profiles in order to ensure
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

