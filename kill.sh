#!/bin/bash

list='gzclient
      gzserver
      rviz
      roslaunch 
      rosmaster
      rosout
      rostopic
      message_to_tf
      hector_quadrotor_actions
      hector_quadrotor_teleop
      controller_manager
      robot_state_publisher
      topic_tools
      gazebo_ros
      hector_quadrotor_actions
      '

for i in $list
    do
        ps -ef | grep $i | awk '{print $2}' | xargs -i kill -9 {}
    done
