# Building

Navigate into devcontainer workspace root and run:

bash
```
colcon build
```

# Launching

Run following command to launch the station launch system in simulation without Gazbo GUI:

bash
```
ros2 launch station station.launch.py simulated:=true gazebo_gui:=false use_sim_time:=true model:=ur10e
```

# View camera image

To view camera image (raw or simulated) run:

bash
```
ros2 run rqt_image_view rqt_image_view /cameras/basler_camera/image_raw
```