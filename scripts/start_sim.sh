#!/bin/bash
# Запуск симуляції: Gazebo Harmonic + ArduPilot SITL + ROS 2 (світ maze)

source /opt/ros/humble/setup.bash
source "$HOME/ardu_ws/install/setup.bash"

export GZ_VERSION=harmonic
export SDF_PATH="$(ls -d $HOME/ardu_ws/install/*/share | paste -sd:)${SDF_PATH:+:$SDF_PATH}"
export GZ_SIM_RESOURCE_PATH="$(ls -d $HOME/ardu_ws/install/*/share | paste -sd:)${GZ_SIM_RESOURCE_PATH:+:$GZ_SIM_RESOURCE_PATH}"

cleanup() {
    echo "Зупинка процесів симуляції..."
    pkill -9 -f "gz sim" 2>/dev/null
    pkill -9 arducopter 2>/dev/null
    pkill -9 micro_ros_agent 2>/dev/null
    pkill -9 -f mavproxy 2>/dev/null
    pkill -9 rviz2 2>/dev/null
}

# Завершити старі процеси перед запуском (звільнити порти)
cleanup
sleep 2

# Ctrl+C зупиняє симуляцію і чистить процеси
trap cleanup INT TERM

ros2 launch ardupilot_gz_bringup iris_maze.launch.py
cleanup
