## --------------------------------------------------------------
## |                       waitFor* macros                      |
## --------------------------------------------------------------

# #{ waitForRos()

waitForRos() {
  echo "waiting for ROS"
  until timeout 6s rosparam get /run_id > /dev/null 2>&1; do
    echo "waiting for /run_id"
    sleep 1;
  done
  sleep 1;
}

# #}

# #{ waitForSimulation()

waitForSimulation() {
  until timeout 6s rostopic echo /gazebo/model_states -n 1 --noarr > /dev/null 2>&1; do
    echo "waiting for simulation"
    sleep 1;
  done
  sleep 1;
}

# #}

# #{ waitForSpawn()

waitForSpawn() {
  until timeout 6s rostopic echo /mrs_drone_spawner/diagnostics -n 1 | grep -z 'spawn_called: True.*processes: 0' > /dev/null 2>&1; do
    echo "waiting for spawn"
    sleep 1;
  done
  sleep 1;
}

# #}

# #{ waitForTopic()

waitForTopic() {
  until timeout 6s rostopic echo "$1" -n 1 --noarr > /dev/null 2>&1; do
    echo "waiting for $1"
    sleep 1;
  done
}

# #}

# #{ waitForOdometry()

waitForOdometry() {
  if [[ "$OLD_PX4_FW" == "true" ]]; then
    until timeout 6s rostopic echo /$UAV_NAME/mavros/local_position/odom -n 1 --noarr > /dev/null 2>&1; do
      echo "waiting for odometry - /$UAV_NAME/mavros/local_position/odom"
      sleep 1;
    done
  else
    until timeout 6s rostopic echo /$UAV_NAME/mavros/odometry/in -n 1 --noarr > /dev/null 2>&1; do
      echo "waiting for odometry - /$UAV_NAME/mavros/odometry/in"
      sleep 1;
    done
  fi
}

# #}

# #{ waitForControlManager()

waitForControlManager() {
  until timeout 6s rostopic echo /$UAV_NAME/control_manager/diagnostics -n 1 --noarr > /dev/null 2>&1; do
    echo "waiting for control manager"
    sleep 1;
  done
}

# #}

# #{ waitForControl()

waitForControl() {
  until timeout 6s rostopic echo /$UAV_NAME/control_manager/diagnostics -n 1 --noarr > /dev/null 2>&1; do
    echo "waiting for control"
    sleep 1;
  done
  until timeout 6s rostopic echo /$UAV_NAME/odometry/odom_main -n 1 --noarr > /dev/null 2>&1; do
    echo "waiting for odom_main"
    sleep 1;
  done
}

# #}

# #{ waitForMpc()

waitForMpc() {
  until timeout 6s rostopic echo /$UAV_NAME/control_manager/diagnostics -n 1 --noarr > /dev/null 2>&1; do
    echo "waiting for control"
    sleep 1;
  done
  until timeout 6s rostopic echo /$UAV_NAME/odometry/odom_main -n 1 --noarr > /dev/null 2>&1; do
    echo "waiting for odom_main"
    sleep 1;
  done
}

# #}

# #{ waitForOffboard()

waitForOffboard() {
  until timeout 6s rostopic echo /$UAV_NAME/control_manager/offboard_on -n 1 --noarr > /dev/null 2>&1; do
    echo "waiting for offboard mode"
    sleep 1;
  done
}

# #}

# #{ waitForCompile()

waitForCompile() {
  while timeout 6s  ps aux | grep "catkin build" | grep -v grep > /dev/null 2>&1; do
    echo "waiting for compilation to complete"
    sleep 1;
  done
}

# #}