xm_people
===============

`xm_people` is a ROS Wrapper for the OpenNI2 and NiTE2 Skeleton Tracker, which you can use ros node of depth_image when using OpenNI. 
With this wrapper,we can ** run skelon detection ,hand tracking ,visiual slam ,object recognition and face recognition at the same time**
### Pre-work
- Install [ROS](http://www.ros.org/) in your Ubuntu
- Install [OpenNI2 and NITE2](https://ariandy1.wordpress.com/2013/07/10/ros-openni2-nite2/)
### Using
 1. make sure xm_people in your ROS catkin_ws directory
 2. 
	 ```
		catkin_make
		roslaunch xm_people tracker.launch 
	 ```
 3. then you can see the tips on terminal/
 and your skelon and hands position are publishing on 'rostopic/people' /
 you can check it 
	 ```
		rostopic echo /people
	 ```
 
 4. meanwhile you can check the topic /camera/depth_registered/points and /camera/rgb/image_raw
