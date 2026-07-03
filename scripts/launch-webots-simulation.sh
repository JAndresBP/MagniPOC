#!/bin/bash

# Simple helper to launch the Webots simulation with optional features.
# Usage: ./launch-webots-simulation.sh [--rviz] [--mapping] [--no-arm] [--arm-family=rebotarm] [--robot-type=fr3] [--no-hand]

source /opt/ros/jazzy/setup.bash
source /home/ws/install/setup.bash

RVIZ=false
MAPPING=false
ARM_INSTALLED=true
ARM_FAMILY=rebotarm
ROBOT_TYPE=fr3
ARM_HAND=true

print_usage() {
  echo "Usage: $0 [--rviz] [--mapping] [--no-arm] [--arm-family=<family>] [--robot-type=<type>] [--no-hand]"
  echo
  echo "  --rviz                 Launch RViz (default: false)"
  echo "  --mapping              Launch mapping subsystem (default: false)"
  echo "  --no-arm               Spawn Magni without an arm (default: arm installed)"
  echo "  --arm-family=<family>  Which arm to mount: rebotarm, franka (default: rebotarm)"
  echo "  --robot-type=<type>    Franka arm model, only used with --arm-family=franka: fr3, fer, fp3, ... (default: fr3)"
  echo "  --no-hand              Omit the Franka gripper, only used with --arm-family=franka (default: hand installed)"
}

# Parse args
while [[ $# -gt 0 ]]; do
  case "$1" in
    --rviz)
      RVIZ=true
      shift
      ;;
    --mapping)
      MAPPING=true
      shift
      ;;
    --no-arm)
      ARM_INSTALLED=false
      shift
      ;;
    --arm-family=*)
      ARM_FAMILY="${1#*=}"
      shift
      ;;
    --robot-type=*)
      ROBOT_TYPE="${1#*=}"
      shift
      ;;
    --no-hand)
      ARM_HAND=false
      shift
      ;;
    -h|--help)
      print_usage
      exit 0
      ;;
    *)
      echo "Unknown option: $1"
      print_usage
      exit 1
      ;;
  esac
done

case "$ARM_FAMILY" in
  rebotarm|franka) ;;
  *)
    echo "Invalid --arm-family: $ARM_FAMILY (expected rebotarm or franka)"
    exit 1
    ;;
esac

LAUNCH_ARGS=()
if [ "$RVIZ" = true ]; then
  LAUNCH_ARGS+=("rviz:=true")
fi
if [ "$MAPPING" = true ]; then
  LAUNCH_ARGS+=("mapping:=true")
fi

# magni_spawn.launch.py builds the spawned robot_description before any launch
# context exists, so the arm options can't be passed as ros2 launch arguments --
# it reads them from the environment instead.
if [ "$ARM_INSTALLED" = true ]; then
  export MAGNI_ARM_INSTALLED=true
else
  export MAGNI_ARM_INSTALLED=false
fi
export MAGNI_ARM_FAMILY="$ARM_FAMILY"
export MAGNI_ARM_ROBOT_TYPE="$ROBOT_TYPE"
if [ "$ARM_HAND" = true ]; then
  export MAGNI_ARM_HAND=true
else
  export MAGNI_ARM_HAND=false
fi

# Launch the simulation (pass through chosen launch args)
ros2 launch magni_webots magni_sim_bringup.launch.py "${LAUNCH_ARGS[@]}"
