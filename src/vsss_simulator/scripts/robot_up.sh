#!/bin/bash

### Set the number of robots
declare -i home_num=$(rosparam get /home/num)
declare -i guest_num=$(rosparam get /guest/num)

ball_name=$(rosparam get /ball/name)
home_prefix=$(rosparam get /home/prefix)
guest_prefix=$(rosparam get /guest/prefix)
		                               
home_x=(    0   -0.6   -0.4    -0.2   0   0)  # the first one is for goal-keeper
home_y=(    0   0      0       0      0   0)       # the first one is for goal-keeper 
guest_x=(   0   0.6    0.4     0.2    0   0)      # the first one is not useful now
guest_y=(   0   0      0       0      0   0)     # the first one is not useful now-keeper

### spawn the ball
# rosrun gazebo_ros spawn_model -file $(rospack find nubot_description)/models/ball/model.sdf -sdf \
#                               -model ${ball_name} \
#                               -x 0.0 -y 0.0 -z 0.0 \
#                                /
# sleep 1

### spawn home robots
for ((i=1; i<=home_num; ++i))
do
    rosrun gazebo_ros spawn_model -file $(rospack find vsss_simulator)/models/home${i}/model.sdf -sdf \
                                  -model ${home_prefix}${i} \
                                  -x ${home_x[$i]} -y ${home_y[$i]} -z 0.0 &
    sleep 0.9
done 

### spawn guest robots
for ((i=1; i<=guest_num; ++i))
do
    rosrun gazebo_ros spawn_model -file $(rospack find vsss_simulator)/models/guest${i}/model.sdf -sdf \
                                  -model ${guest_prefix}${i} \
                                  -x ${guest_x[$i]} -y ${guest_y[$i]} -z 0.0 &
    sleep 0.9
done 

