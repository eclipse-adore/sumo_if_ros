#!/usr/bin/env bash

set -euo pipefail

echoerr (){ printf "%s" "$@" >&2;}
exiterr (){ printf "%s\n" "$@" >&2; exit 1;}

SCRIPT_DIRECTORY="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
SUMO_CONFIG_DIRECTORY="${SCRIPT_DIRECTORY}/sumo_configs"
SUMO_CONFIG_FILE="demo_sumo_bridge.sumocfg"

ros2 run sumo_bridge sumo_bridge --ros-args --param "sumo_config_file:=${SUMO_CONFIG_DIRECTORY}/${SUMO_CONFIG_FILE}"
