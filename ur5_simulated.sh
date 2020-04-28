#!/bin/bash
echo "Simulation is starting"
source devel/setup.bash
# roscore &
# roscore_pid=$!
sleep 8
roslaunch ur_gazebo ur5.launch limited:=true &
gazebo_pid=$!
sleep 10
roslaunch ur5_moveit_config ur5_moveit_planning_execution.launch sim:=true limited:=true &
moveit_pid=$!
sleep 3
roslaunch ur5_moveit_config moveit_rviz.launch config:=true &
rviz_pid=$!
sleep 20
echo "Simulation is running"
running="run"
while [[ ! $running == "exit" ]]; do
    echo "To exit the programm type exit"
    read running
done
echo "Simulation is closing"
kill $rviz_pid &
kill $moveit_pid &
kill --signal 15 $gazebo_pid &
echo "waiting"
wait
# kill $roscore_pid
