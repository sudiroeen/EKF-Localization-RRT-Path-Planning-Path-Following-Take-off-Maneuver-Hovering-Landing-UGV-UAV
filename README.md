# EKF-Localization-RRT-Path-Planning-Path-Following-Take-off-Maneuver-Hovering-Landing-UGV-UAV

# Video:
   + Part 1: https://www.youtube.com/watch?v=fzOL-B-07bc or https://www.youtube.com/watch?v=EAcPxWwW37g&feature=youtu.be
   + Part 2: https://www.youtube.com/watch?v=4pRkIPMCuYw or 
# All Feature: 
   + Copyright by:
       <> Sudiro 
       <> Kecuali yang saya jelaskan lebih di bawah
   + e-mail: SudiroEEN@gmail.com
   + My Site: github.com/sudiroeen


# Feature:

                        [Differential Driver/ UGV Robot]
1. Motion:
   - Open Loop Motion for rotational motion
   - Braitenberg Algorithm [My C++ style of V-REP example]
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
      # start Path Planning
   - using P controller [pure from V-REP example with condition addition]

4. Landing:
   - When UGV robot has been reached the finish path
   - using P controller


+ "common_include" package is V-REP API for C++ that allowing me to controller robot using V-REP Built-In Function() using C++ API, copyright by V-REP Community

# REQUIREMENTS:
   - VREP & VREP API
   - ROS Kinetic
   - OpenCV



# Alur cerita:

1. saat V-REP di run, quadrotor akan take-off kemudian maneuver terus menerus

2. untuk menjalankan UGV dan lokalisasi, run perintah "roslaunch pioneer_package pioneer_package.launch" pada terminal

3. saat tombol 'P' di keyboard ditekan quadrotor akan hovering, di sisi yang lain path planning dengan RRT dimulai, dengan posisi UGV sebagai starting point, dan posisi quadrotor sebagai finish point

4. setelah proses path planning sudah mendapatkan path nya, kemudian tekan tombol sembarang untuk memulai path following oleh UGV

5. setelah UGV sampai di posisi quadrotor, UGV akan otomatis berhenti, sedangkan quadrotor akan mulai landing.



# How To Use

1. Saat pertama kali, SELAIN package "rrt" dan "common_include" keluarkan dari folder "src/", hal ini karena hierarchy running program mengakhirkan kedua package tersebut.
2. run di terminal:
   catkin_make && source devel/setup.bash
3. setelah sukses, package selain "rrt" dan "common_include" tersebut kembalikan ke dalam folder "src/"
4. run kembali di terminal:
   catkin_make && source devel/setup.bash

5. setelah sukses JANGAN di roslaunch dulu, karena proses menunggu komunikasi dengan V-REP

6. oleh karena itu, run V-REP dulu

7. run di terminal:
   roslaunch pioneer_package pioneer_package.launch

6. lakukan sesuai deskripsi di file "alur_cerita_robotika.txt"  



# displayed on:

- img_laser -> line feature & landmark (red circled)
- img_bck -> all
- tampil_mc -> fitted line using least square to get angle

- path_hasil -> hasil path dari proses path planning
- bezier_spline -> smoothed path
- RRT -> Proses RRT Path Planning 
      -> bercak merah adalah random value generated

# How To Contribute:
   Just Send Pull Request
