#!/bin/bash
set -e

export MUJOCO_PLUGIN_DIR=/home/x/mujoco/mujoco-3.6.0/build/lib
cd "$(dirname "$0")"
./build/bin/quadrotor "$@"
