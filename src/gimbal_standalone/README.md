# Arm Robot simulation with ROS2 and Gazebo.
  
this package requires ros2_control, ros2_controllers and gazebo_ros2_control package to be used. 
  
to see the model on rviz use:
```
ros2 launch gimbal_standalone rviz.launch.py
```
to run the simulation on gazebo use:
```
ros2 launch gimbal_standalone launch_sim.launch.py
```
you can move the angle(in degrees) of each axis using this command:
```
ros2 launch gimbal_standalone control.launch.py ax1:=35 ax2:=35 
```
![alt text](https://github.com/MickySukmana/gimbal_standalone/blob/main/img/lr.png?raw=true)
