## [draft] How to use. This is manual works only for lab's server 

To run docker, use following alias

```
irun
```

To open a new bash session inside docker use

```
iexec
```

It's realy simple to use docker :)


### Run real robot with gripper

1. Turn on the UR5e.
    
    - press "on" button and wait
    - press to the programm _ros-control_ and press _open_
    - press to an icon on the right-bottom corner and press on->start

2. On the server

```
irun
```

```
roslaunch the_mainest toborn.launch
```

It runs
 - robot driver
 - motion-planning-interface
 - rviz with preset simple config
 - gripper driver

3. Press _play_ button on the pendant of the robot

#### To use gripper See its repository (just need to call the service /gripper_state with params [0,1,2,3,4,5,6])

#### To use _motion-planning_ add to rviz _motion_planning_


## Usage

### Check next:

1. Network-manager setup

```
ip 192.168.88.190
mask 255.255.255.0
getway 192.168.88.1
dns 192.168.88.1
```

2. `env` variables on the host (find it in `~/.bashrc`)

```
export ROS_HOSTNAME=192.168.88.190
export ROS_IP=192.168.88.190
export ROS_MASTER_URI=http://$ROS_IP:11311
```


### Inside the docker-container for real robot stuff.

1. Open terminal-tab on the host

```
irun
roslaunch the_mainest born_robot.launch
```

2. [ROBOT PENDANT] 

- load `ros_control` program
- switch to `manual` mode (see right top panel)
- turn on robot
- press `play`

3. Open new-tab on the host

```
iexec
rosrun msdp pick_and_place
```

4. Open new-tab on the host

```
iexec
rosrun msdp add_object_node
```

5. Open new-tab on the host. Run the camera (reconnect cable please)

```
iexec
roslaunch the_mainest rs_aligned_depth.launch
```

6. Open new-tab on the host. Run SYSTEM

```
iexec
rosrun the_mainest the_mainest_node.py 
```

and call service to start `/start_me`.

### Inside the docker-container by IVAN











