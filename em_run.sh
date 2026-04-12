#!/bin/bash
set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
PLUGIN_DIR="${SCRIPT_DIR}/build/bin/mujoco_plugin"
BUNDLED_PLUGIN_DIR="${SCRIPT_DIR}/third_party/mujoco-3.6.0/bin/mujoco_plugin"
GENERATOR="${SCRIPT_DIR}/third_party/dynamic_obs_generator/generate_scene_obstacles.py"
TARGET_CONFIG="${SCRIPT_DIR}/.em_run_target"

if [ -d "${BUNDLED_PLUGIN_DIR}" ] && [ -d "${PLUGIN_DIR}" ]; then
  export MUJOCO_PLUGIN_DIR="${BUNDLED_PLUGIN_DIR}:${PLUGIN_DIR}"
elif [ -d "${BUNDLED_PLUGIN_DIR}" ]; then
  export MUJOCO_PLUGIN_DIR="${BUNDLED_PLUGIN_DIR}"
elif [ -d "${PLUGIN_DIR}" ]; then
  export MUJOCO_PLUGIN_DIR="${PLUGIN_DIR}"
fi

MERGED_CONFIG=""
SIM_CONFIG=""
ROBOT_CONFIG=""
SHOW_HELP=0
FORCE_SELECT=0
TARGET=""
EXECUTABLE=""
PASSTHROUGH_ARGS=()

is_valid_target() {
  case "$1" in
    quadrotor|scout)
      return 0
      ;;
    *)
      return 1
      ;;
  esac
}

choose_target() {
  local choice=""
  local selected=""

  echo "请选择要启动的仿真目标："
  echo "  1) quadrotor"
  echo "  2) scout"

  while true; do
    read -r -p "输入编号或名称: " choice
    case "${choice}" in
      ""|1|quadrotor)
        selected="quadrotor"
        break
        ;;
      2|scout|ground_vehicle)
        selected="scout"
        break
        ;;
      *)
        echo "无效选择：${choice}。请输入 1/2、quadrotor 或 scout。"
        ;;
    esac
  done

  printf "%s\n" "${selected}" > "${TARGET_CONFIG}"
  echo "已保存默认仿真目标：${selected}（之后用 ./em_run.sh -S 可重新选择）"
  TARGET="${selected}"
}

load_target() {
  if [ "${FORCE_SELECT}" -eq 1 ]; then
    choose_target
    return
  fi

  if [ -f "${TARGET_CONFIG}" ]; then
    TARGET="$(tr -d '[:space:]' < "${TARGET_CONFIG}")"
    if is_valid_target "${TARGET}"; then
      return
    fi
    echo "已保存的仿真目标无效：${TARGET}，将重新选择。"
  fi

  if [ -t 0 ]; then
    choose_target
  else
    TARGET="quadrotor"
    printf "%s\n" "${TARGET}" > "${TARGET_CONFIG}"
    echo "未检测到交互式终端，默认选择 quadrotor。之后用 ./em_run.sh -S 可重新选择。"
  fi
}

resolve_executable() {
  case "${TARGET}" in
    quadrotor)
      EXECUTABLE="./build/bin/quadrotor"
      ;;
    scout)
      EXECUTABLE="./build/bin/scout"
      if [ -z "${MERGED_CONFIG}" ] && [ -z "${SIM_CONFIG}" ]; then
        PASSTHROUGH_ARGS+=(--sim-config "${SCRIPT_DIR}/ground_vehicle/cfg/sim_config.yaml")
        SIM_CONFIG="${SCRIPT_DIR}/ground_vehicle/cfg/sim_config.yaml"
      fi
      ;;
    *)
      echo "未知仿真目标：${TARGET}" >&2
      exit 1
      ;;
  esac

  if [ ! -x "${EXECUTABLE}" ]; then
    echo "目标可执行文件不存在或不可执行：${EXECUTABLE}" >&2
    echo "请先构建对应目标，例如：cmake --build build --target ${TARGET}" >&2
    exit 1
  fi
}

ARGS=("$@")
for ((i=0; i<${#ARGS[@]}; ++i)); do
  arg="${ARGS[$i]}"
  case "${arg}" in
    -S)
      FORCE_SELECT=1
      ;;
    --help|-h)
      SHOW_HELP=1
      PASSTHROUGH_ARGS+=("${arg}")
      ;;
    --config)
      PASSTHROUGH_ARGS+=("${arg}")
      if [ $((i + 1)) -lt ${#ARGS[@]} ]; then
        MERGED_CONFIG="${ARGS[$((i + 1))]}"
        PASSTHROUGH_ARGS+=("${MERGED_CONFIG}")
        ((++i))
      fi
      ;;
    --sim-config)
      PASSTHROUGH_ARGS+=("${arg}")
      if [ $((i + 1)) -lt ${#ARGS[@]} ]; then
        SIM_CONFIG="${ARGS[$((i + 1))]}"
        PASSTHROUGH_ARGS+=("${SIM_CONFIG}")
        ((++i))
      fi
      ;;
    --robot-config)
      PASSTHROUGH_ARGS+=("${arg}")
      if [ $((i + 1)) -lt ${#ARGS[@]} ]; then
        ROBOT_CONFIG="${ARGS[$((i + 1))]}"
        PASSTHROUGH_ARGS+=("${ROBOT_CONFIG}")
        ((++i))
      fi
      ;;
    --viewer|--headless)
      PASSTHROUGH_ARGS+=("${arg}")
      ;;
    -*)
      PASSTHROUGH_ARGS+=("${arg}")
      ;;
    *)
      if [ -z "${MERGED_CONFIG}" ] && [ -z "${SIM_CONFIG}" ]; then
        MERGED_CONFIG="${arg}"
      fi
      PASSTHROUGH_ARGS+=("${arg}")
      ;;
  esac
done

cd "${SCRIPT_DIR}"

load_target
resolve_executable

if [ "${SHOW_HELP}" -eq 0 ] && [ "${TARGET}" = "quadrotor" ] && [ -f "${GENERATOR}" ]; then
  GENERATOR_ARGS=(--print-output-path)
  if [ -n "${MERGED_CONFIG}" ]; then
    GENERATOR_ARGS+=(--config "${MERGED_CONFIG}")
  else
    if [ -n "${SIM_CONFIG}" ]; then
      GENERATOR_ARGS+=(--sim-config "${SIM_CONFIG}")
    fi
    if [ -n "${ROBOT_CONFIG}" ]; then
      GENERATOR_ARGS+=(--robot-config "${ROBOT_CONFIG}")
    fi
  fi

  GENERATED_SCENE_XML="$(python3 "${GENERATOR}" "${GENERATOR_ARGS[@]}")"
  export AUSIM_SCENE_XML_OVERRIDE="${GENERATED_SCENE_XML}"
fi

exec "${EXECUTABLE}" "${PASSTHROUGH_ARGS[@]}"
