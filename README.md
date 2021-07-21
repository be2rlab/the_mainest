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

