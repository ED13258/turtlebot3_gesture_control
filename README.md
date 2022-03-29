# turtlebot3 gesture control

### Dependencies Overview
* ROS Noetic
* Ubuntu 20.04.4 LTS
* Turtlebot3 waffle_pi
* Raspberry Pi Camera (for Turtlebot3)
* Camera connected to your PC
* VScode (run in /usr/bin/python3.6)

### How to Run
1. Copy this package into your workspace and run `catkin_make`.  ( package name is 'gesture_teleop' )
2. Bringup TurtleBot3
* [PC]  `roscore`
* [TurtleBot3]  `ssh pi@{IP_ADDRESS_OF_RASPBERRY_PI}`
* [TurtleBot3]  `roslaunch turtlebot3_bringup turtlebot3_robot.launch`
3. Raspberry Pi Camera
* [TurtleBot3]  `roslaunch turtlebot3_bringup turtlebot3_rpicamera.launch`
* [PC]  `rqt_image_view`
(You can see TurtleBot3 point of view)
4. run in VScode
* [PC]  `/usr/bin/python3.6 /home/edward/turtlebot3_ws/src/gesture_teleop/scripts/tesla_detect.py`
* [PC]  `/usr/bin/python3.6 /home/edward/turtlebot3_ws/src/gesture_teleop/scripts/tesla_teleop.py`
5. gesture control
* Put your left hand first, and then put your right hand: Drive forward
* Right hand is higher than left hand: Turn left
* Right hand is lower than left hand: Turn right
* put one hand down: Stop

### Demo video
https://www.youtube.com/watch?v=KIFebzCm9UU

### Reference
* https://www.youtube.com/watch?v=lvt_i8tVSfs  ( Vedio imitation )
* https://emanual.robotis.com/docs/en/platform/turtlebot3/overview/  ( Turtlebot e-Manual )
* https://www.ncnynl.com/archives/202111/4820.html  ( Raspberry Pi Camera Setup )
* https://youtu.be/vQZ4IvB07ec  ( hand joint detection )
* https://github.com/HaofanYang/gesture-recognition-ros-pkg#dependencies-overview  ( Hand Gesture Recognition  )


