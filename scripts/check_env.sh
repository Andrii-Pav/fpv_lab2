#!/bin/bash
source /opt/ros/humble/setup.bash 2>/dev/null
source "$HOME/ardu_ws/install/setup.bash" 2>/dev/null

ok()  { echo "[ OK ] $1"; }
bad() { echo "[FAIL] $1"; }
check() { desc="$1"; shift; if "$@" >/dev/null 2>&1; then ok "$desc"; else bad "$desc"; fi; }

echo "=== Система ==="
lsb_release -d
echo "Архітектура: $(uname -m)"
python3 --version
free -h | grep -E "Mem|Swap"
echo "Графіка: $(glxinfo 2>/dev/null | grep 'OpenGL renderer')"

echo; echo "=== Версії ПЗ ==="
echo "ROS 2:       $(dpkg-query -W -f='${Package} ${Version}' ros-humble-desktop 2>/dev/null)"
echo "Gazebo:      $(dpkg-query -W -f='${Package} ${Version}' gz-harmonic 2>/dev/null)"
echo "colcon-core: $(dpkg-query -W -f='${Version}' python3-colcon-core 2>/dev/null)"
echo "ArduPilot:   $(git -C $HOME/ardu_ws/src/ardupilot describe --tags --always 2>/dev/null)"
echo "GZ_VERSION=$GZ_VERSION"

echo; echo "=== Інструменти ==="
check "ROS_DISTRO = humble" test "$ROS_DISTRO" = "humble"
check "ros2 CLI" ros2 --help
check "colcon" which colcon
check "rosdep" which rosdep
check "vcs" which vcs
check "microxrceddsgen" which microxrceddsgen
check "mavproxy.py" which mavproxy.py
check "gz (Gazebo Sim)" which gz
check "бінарник arducopter" bash -c 'find $HOME/ardu_ws/install -name arducopter -type f | grep -q .'

echo; echo "=== ROS-пакети у ~/ardu_ws ==="
for p in ardupilot_msgs ardupilot_sitl ardupilot_gazebo ardupilot_gz_bringup \
         ardupilot_gz_description ardupilot_gz_gazebo ros_gz_bridge ros_gz_sim micro_ros_agent; do
  check "пакет $p" ros2 pkg prefix "$p"
done

echo; echo "=== Налаштування ==="
check "GZ_VERSION=harmonic" test "$GZ_VERSION" = "harmonic"
check "SDF_PATH містить ardupilot_gazebo" bash -c 'echo "$SDF_PATH" | grep -q ardupilot_gazebo'
check "launch: render-engine-gui ogre (встановлено)" grep -q "render-engine-gui ogre" \
  "$HOME/ardu_ws/install/ardupilot_gz_bringup/share/ardupilot_gz_bringup/launch/iris_maze.launch.py"
check "start_sim.sh" test -x "$HOME/fpv_labs/scripts/start_sim.sh"
check "stop_sim.sh" test -x "$HOME/fpv_labs/scripts/stop_sim.sh"
swapon --show
