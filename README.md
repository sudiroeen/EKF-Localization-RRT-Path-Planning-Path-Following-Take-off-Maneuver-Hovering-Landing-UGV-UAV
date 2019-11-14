
# EKF-Localization-RRT-Path-Planning-Path-Following-Take-off-Maneuver-Hovering-Landing-UGV-UAV

# Video:
   + Part 1: https://www.youtube.com/watch?v=fzOL-B-07bc or https://www.youtube.com/watch?v=EAcPxWwW37g&feature=youtu.be
   + Part 2: https://www.youtube.com/watch?v=4pRkIPMCuYw or https://youtu.be/BWt6j2bahIg
# All Feature: 
   + Copyright by:
       - Sudiro
       - Another author explained below*
   + e-mail: SudiroEEN@gmail.com
   + My Site: ulur.in/sudiroeen


# Feature:

                        [Differential Driver/ UGV Robot]
1. Motion:
   - Open Loop Motion for rotational motion
   - Braitenberg Algorithm [My C++ style of V-REP example]*
   - code is inside "pioneer_package" package

2. EKF Localization [not yet successfull]
   - Laser Data Process:
      a. Adaptive Break-Point Detector
      b. Iterative End Point Fitting
      c. Rectangular Form of Least Square Fitting
   
   - code is inside "pioneer_package" package

3. RRT Path Planning:
   - Started when keyboard 'P' pressed
   - using true value position [from V-REP, cause non-successfull of my EKF Localization]
   - code is inside "rrt" package 

4. Bezier Spline smoothing algorithm [not yet successfull]
   - code is inside "rrt" package 

5. Path Following:
   - When Path Planning done and afther press any character on keyboard
   - P controller for rotational and translational motion
   - code is inside "pioneer_package" package

6. Visualized on OpenCV Window
   - code is inside "pioneer_package" package
   

                        [Quadrotor / UAV Robot]
+ All of these feature in Lua Form
+ "quad_package" is take-off process on Ros-C++ style [But not yet successfull]

1. Take-Off:
   - using P controller

2. Maneuver:
   - using P-scheduling controller
   - Following path specified before [manually]

3. Hovering:
   - When keyboard 'P' pressed:
   - start Path Planning
   - using P controller [pure from V-REP example with condition addition]*

4. Landing:
   - When UGV robot has been reached the finish path
   - using P controller


+ "common_include" package is V-REP API for C++ that allowing me to control robot using V-REP Built-In Function() using C++ API, copyright by V-REP Community*

# REQUIREMENTS:
   - VREP & VREP API
   - ROS Kinetic
   - OpenCV



# Story:

1. When you run V-REP, quadrotor will start to take-off then maneuver continuousely

2. To start UGV motion and Localization, run "roslaunch pioneer_package pioneer_package.launch" on terminal

3. To finish Localization, push 'P' key on your keyboard, then Quadrotor will go to do hovering,

4. In other side, RRT Path Planning will be starting, with UGV position as starting point, and Quadrotor position as finish point

5. After RRT gets path, you can start Path Following by push any key on your keyboard, and UGV start to follow that path.

5. After UGV reach x-y quadrotor position (with tolerant error), UGV will stop, and Quadrotor start to do landing

6. After Quadrotor z-position reach UGV, quadrotor will stop, and process finish

7. You can close all process by [Ctrl + C] on your terminal



# How To Use

1. At First time, cut EXCEPT "rrt" and "common_include" package from "src/" directory, because on hierarchy, they will be ran lastly, but other package includes them
2. run on your terminal:
   catkin_make && source devel/setup.bash
3. if success, enter other cut package to "src/" again
4. run on your terminal:
   catkin_make && source devel/setup.bash

5. after success, DON'T directly run "roslaunch", because node will work after communicate with V-REP, so you must run V-REP firstly

6. run your V-REP scene
7. run on your terminal:
   roslaunch pioneer_package pioneer_package.launch

6. do as descripted on "Story" section



# displayed on:

- img_laser -> line feature & landmark (red circled)
- img_bck -> all
- tampil_mc -> fitted line using least square to get angle

- path_hasil -> path resulted from Ptah Planning process
- bezier_spline -> smoothed path
- RRT -> RRT Path Planning Process
      -> red dot is random value generated from RRT Path Planning Process

# How To Contribute:
   Just Send Pull Request
