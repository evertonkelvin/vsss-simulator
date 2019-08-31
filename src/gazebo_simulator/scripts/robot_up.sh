#!/bin/bash

### Set the number of robots
declare -i home_num=$(rosparam get /home/num)
declare -i guest_num=$(rosparam get /guest/num)

ball_name=$(rosparam get /ball/name)
home_prefix=$(rosparam get /home/prefix)
guest_prefix=$(rosparam get /guest/prefix)

# POSIÇÕES: BOLA GOLEIRO VOLANTE ATACANTE ROBO4 ROBO5  
home_x=(    0   -0.6   -0.4    -0.2   0   0) 
home_y=(    0   0      0       0      0   0) 
guest_x=(   0   0.6    0.4     0.2    0   0) 
guest_y=(   0   0      0       0      0   0) 


### spawn home robots
for ((i=1; i<=home_num; ++i))
do
    rosrun gazebo_ros spawn_model -file $(rospack find gazebo_simulator)/models/home${i}/model.sdf -sdf \
                                  -model ${home_prefix}${i} \
                                  -x ${home_x[$i]} -y ${home_y[$i]} -z 0.0 &
    sleep 0.7
done 

### spawn guest robots
for ((i=1; i<=guest_num; ++i))
do
    rosrun gazebo_ros spawn_model -file $(rospack find gazebo_simulator)/models/guest${i}/model.sdf -sdf \
                                  -model ${guest_prefix}${i} \
                                  -x ${guest_x[$i]} -y ${guest_y[$i]} -z 0.0 &
    sleep 0.7
done 

