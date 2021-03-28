# Running Deliveries

1. 
```
roslaunch deliverybot_simulations building_world.launch

```

2. 
```
rosparam set /map_folder_path "/path/to/maps/folder"
```

3. 
```
rosrun deliverybot_delivery delivery_client.py
```

4. 
```
rosrun deliverybot_delivery delivery_server.py
```

5. 
```
rosrun rviz rviz
```

6. 
```
roslaunch deliverybot_navigation move_base.launch
```

7. 
```
roslaunch deliverybot_navigation deliverybot_navigation.launch

```