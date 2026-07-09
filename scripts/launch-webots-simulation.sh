#!/bin/bash

# Simple helper to launch the Webots simulation with optional features.
# Usage: ./launch-webots-simulation.sh [--rviz] [--mapping] [--nav2] [--no-arm] [--arm-family=rebotarm] [--robot-type=fr3] [--no-hand]
#                                      [--no-torso] [--torso-left=tray|none] [--torso-head=screen|sensor_head|none]

source /opt/ros/jazzy/setup.bash
source /home/ws/install/setup.bash

RVIZ=false
MAPPING=false
NAV2=false
ARM_INSTALLED=true
ARM_FAMILY=rebotarm
ROBOT_TYPE=fr3
ARM_HAND=true
TORSO_INSTALLED=true
TORSO_LEFT=tray
TORSO_HEAD=screen

print_usage() {
  echo "Usage: $0 [--rviz] [--mapping] [--no-arm] [--arm-family=<family>] [--robot-type=<type>] [--no-hand]"
  echo "          [--no-torso] [--torso-left=<acc>] [--torso-head=<acc>]"
  echo
  echo "  --rviz                 Launch RViz (default: false)"
  echo "  --mapping              Launch mapping subsystem (default: false)"
  echo "  --nav2                 Launch Nav2 navigation (implies --mapping, default: false)"
  echo "  --no-arm               Spawn Magni without an arm (default: arm installed)"
  echo "  --arm-family=<family>  Which arm to mount: rebotarm, franka (default: rebotarm)"
  echo "  --robot-type=<type>    Franka arm model, only used with --arm-family=franka: fr3, fer, fp3, ... (default: fr3)"
  echo "  --no-hand              Omit the Franka gripper, only used with --arm-family=franka (default: hand installed)"
  echo "  --no-torso             Spawn without the humanoid torso; arm reverts to the tower-shelf mount (default: torso installed)"
  echo "  --torso-left=<acc>     Left shoulder accessory: tray, none (default: tray)"
  echo "  --torso-head=<acc>     Head accessory: screen, sensor_head, none (default: screen)"
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
    --nav2)
      NAV2=true
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
    --no-torso)
      TORSO_INSTALLED=false
      shift
      ;;
    --torso-left=*)
      TORSO_LEFT="${1#*=}"
      shift
      ;;
    --torso-head=*)
      TORSO_HEAD="${1#*=}"
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

case "$TORSO_LEFT" in
  tray|none) ;;
  *)
    echo "Invalid --torso-left: $TORSO_LEFT (expected tray or none)"
    exit 1
    ;;
esac

case "$TORSO_HEAD" in
  screen|sensor_head|none) ;;
  *)
    echo "Invalid --torso-head: $TORSO_HEAD (expected screen, sensor_head or none)"
    exit 1
    ;;
esac

LAUNCH_ARGS=()
if [ "$RVIZ" = true ]; then
  LAUNCH_ARGS+=("rviz:=true")
fi
# Nav2 (SLAM mode) needs Cartographer's /map, so --nav2 implies mapping too.
if [ "$NAV2" = true ]; then
  MAPPING=true
fi
if [ "$MAPPING" = true ]; then
  LAUNCH_ARGS+=("mapping:=true")
fi
if [ "$NAV2" = true ]; then
  LAUNCH_ARGS+=("navigation:=true")
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
if [ "$TORSO_INSTALLED" = true ]; then
  export MAGNI_TORSO_INSTALLED=true
else
  export MAGNI_TORSO_INSTALLED=false
fi
export MAGNI_TORSO_LEFT="$TORSO_LEFT"
export MAGNI_TORSO_HEAD="$TORSO_HEAD"

# Launch the simulation (pass through chosen launch args)
ros2 launch magni_webots magni_sim_bringup.launch.py "${LAUNCH_ARGS[@]}"
