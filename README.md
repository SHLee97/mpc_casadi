# Setup
This algorithm is tested in Ubuntu 20.04 ROS noetic with python 3.x

## Requirement for polyhedron
### decompROS (https://github.com/sikang/DecompROS)

## For dynamic object environment

```
$ cd ~/simulation_setup
$ mkdir build && cd build
$ cmake ..
$ make

$ cd ~/simulation_setup/dynamic_obstacles
$ echo "export GAZEBO_MODEL_PATH=$(pwd):$GAZEBO_MODEL_PATH" >> ~/.bashrc
$ source ~/.bashrc

$ cd ~/simulation_setup/build
$ echo "export GAZEBO_PLUGIN_PATH=$(pwd):$GAZEBO_PLUGIN_PATH" >> ~/.bashrc
$ source ~/.bashrc
```

### How to add dynamic objects

1. Add dynamic model (model.config & sdf)
2. Make animation plugin (animated_box.cc)
3. Modify the trajectory and class name in xxx.cc file
4. Add following tag to model.sdf
```
    ...
    <static>1</static>
    <allow_auto_disable>1</allow_auto_disable>
    <plugin name='push_animate' filename='libanimated_box.so'/>
    </model>
    </sdf>
```
5. Add new plugin to CMakeLists.txt in simulation_setup folder
6. Call new model in .world file as follows:

```
    <include>
        <uri>model://drc_practice_blue_cylinder</uri>      
        <name>blue_cylinder</name>
    </include>
```

# How to start
```
    roslaunch mpc_casadi world_setup.launch
    roslaunch tracking_nmpc_dk.launch
    rviz -d rviz.rviz
``` 