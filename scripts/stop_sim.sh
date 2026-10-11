#!/bin/bash
# Примусова зупинка всіх процесів симуляції
pkill -9 -f "gz sim" 2>/dev/null
pkill -9 -f "ros2 launch" 2>/dev/null
pkill -9 arducopter 2>/dev/null
pkill -9 micro_ros_agent 2>/dev/null
pkill -9 -f mavproxy 2>/dev/null
pkill -9 rviz2 2>/dev/null
echo "Процеси симуляції зупинено"
