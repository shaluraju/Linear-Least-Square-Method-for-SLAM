# Linear-Least-Square-Method-for-SLAM
In discrete time, Linear Least Square(LLS) Method is applied to solve the Localization part of the SLAM problem. A simpe case, Linear 1 dimension is considered, since LLS is a computationally heavy algorithm.
The Robot(car) moves in a Hallway which detects three landmarks with its Laser Range Finder along its path and it is also equipped with a pair of wheel encoders which helps to calculate the odometry of the vehicle.  LLS Method is applied on the data collected from the sensors to Localize the Robot. 

## Environment Setup

![Screenshot from 2021-05-27 10-14-31](https://user-images.githubusercontent.com/67613439/119812019-506d1580-bf05-11eb-8493-dff2aece0469.png)

## Assumptions

* The mobile Robot is considered to start its traversal of the hallway at position 0 (the robot knows this with high certainty), and drives at constant velocity until it reaches the end of the hallway 
* All discrete-time sensing and state estimation operates with a fixed time-step of 0.1 seconds.
* The hallway is 10m long, and the robot’s sensing range is 0.5m.
* The robot moves with a constant velocity of 0.1 m/s.
* The landmarks are located at 2m, 5m and 8m away from start position.
* The robot’s odometry measurements are corrupted with zero-mean, additive Gaussian white noise with standard deviation 0.1 m/s.
* The robot’s range measurements to landmarks (when the landmarks are within range) are corrupted with zero-mean, additive Gaussian white noise with standard deviation 0.01 m.
* There is no environmental process noise influencing the robot’s motion – it moves in a completely deterministic manner, exactly as intended.

## Results

![image](https://user-images.githubusercontent.com/67613439/125937114-80ec19a5-b7dc-49ac-b8d3-630d2eff7ebb.png)
![image](https://user-images.githubusercontent.com/67613439/125937140-f8ad734d-21df-446b-b2e8-145ff6c2c586.png)

