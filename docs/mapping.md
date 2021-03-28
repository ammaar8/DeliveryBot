# Mapping Tutorial

1. First launch the simulation world in gazebo. 
```
roslaunch deliverybot_simulations building_world.launch
```

2. Next we will need rviz to get visualization of the map and robot
```
rosrun rviz rviz
```

3. Start rqt robot steering to drive the robot
```
rosrun rqt_robot_steering
```

4. Launch elevator controls gui. We will need this to control the elevator and move the robot to a different level for mapping.
```
roslaunch elevator_gui elevator_hw_gui.launch
```

5. Finally launch mapping gui.
```
rosrun deliverybot_mapping mapping_gui.py
```