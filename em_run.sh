#!/bin/bash
set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
PLUGIN_DIR="${SCRIPT_DIR}/build/bin/mujoco_plugin"
BUNDLED_PLUGIN_DIR="${SCRIPT_DIR}/third_party/mujoco-3.6.0/bin/mujoco_plugin"
GENERATOR="${SCRIPT_DIR}/third_party/dynamic_obs_generator/generate_scene_obstacles.py"
REGISTRY_RESOLVER="${SCRIPT_DIR}/script/resolve_model_registry.py"
MODEL_TARGET_CONFIG="${SCRIPT_DIR}/.em_run_target"
REGISTRY_FILES=(
  "${SCRIPT_DIR}/quadrotor/cfg/quadrotor_registry.yaml"
  "${SCRIPT_DIR}/ground_vehicle/cfg/ground_vehicle_registry.yaml"
)

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
MODEL_TARGET=""
EXECUTABLE=""
DEFAULT_MODEL_TARGET=""
PASSTHROUGH_ARGS=()
declare -a MODEL_ORDER=()
declare -A MODEL_DISPLAY_NAME=()
declare -A MODEL_FAMILY=()
declare -A MODEL_EXECUTABLE=()
declare -A MODEL_SIM_CONFIG=()
declare -A MODEL_ROBOT_CONFIG=()
declare -A MODEL_DYNAMIC_OBSTACLE_ENABLED=()
declare -A MODEL_DYNAMIC_OBSTACLE_CONFIG=()

load_model_registry() {
  local registry_dump=""

  if [ ! -f "${REGISTRY_RESOLVER}" ]; then
    echo "模型注册解析脚本不存在：${REGISTRY_RESOLVER}" >&2
    exit 1
  fi

  if ! registry_dump="$(python3 "${REGISTRY_RESOLVER}" "${SCRIPT_DIR}" "${REGISTRY_FILES[@]}")"; then
    echo "加载模型注册表失败。" >&2
    exit 1
  fi

  eval "${registry_dump}"
}

normalize_model_target() {
  local candidate="$1"
  local target=""

  for target in "${MODEL_ORDER[@]}"; do
    if [ "${candidate}" = "${target}" ] || [ "${candidate}" = "${MODEL_DISPLAY_NAME[$target]}" ]; then
      printf "%s\n" "${target}"
      return 0
    fi
  done

  return 1
}

choose_model_target() {
  local choice=""
  local selected=""
  local index=0

  echo "请选择要启动的仿真模型："
  for target in "${MODEL_ORDER[@]}"; do
    index=$((index + 1))
    echo "  ${index}) ${MODEL_DISPLAY_NAME[$target]} (${MODEL_FAMILY[$target]})"
  done

  while true; do
    read -r -p "输入编号或名称: " choice
    if [[ "${choice}" =~ ^[0-9]+$ ]] && [ "${choice}" -ge 1 ] && [ "${choice}" -le "${#MODEL_ORDER[@]}" ]; then
      selected="${MODEL_ORDER[$((choice - 1))]}"
      break
    fi

    if selected="$(normalize_model_target "${choice}")"; then
      break
    fi

    echo "无效选择：${choice}。可选项：编号或 ${MODEL_ORDER[*]}。"
  done

  printf "%s\n" "${selected}" > "${MODEL_TARGET_CONFIG}"
  echo "已保存默认仿真模型：${selected}（之后用 ./em_run.sh -S 可重新选择）"
  MODEL_TARGET="${selected}"
}

load_model_target() {
  local stored_target=""

  if [ "${FORCE_SELECT}" -eq 1 ]; then
    choose_model_target
    return
  fi

  if [ -f "${MODEL_TARGET_CONFIG}" ]; then
    stored_target="$(tr -d '[:space:]' < "${MODEL_TARGET_CONFIG}")"
    if MODEL_TARGET="$(normalize_model_target "${stored_target}")"; then
      echo "使用默认仿真模型：${MODEL_TARGET}（之后用 ./em_run.sh -S 可重新选择）"
      return
    fi
    echo "已保存的仿真模型无效：${stored_target}。请运行 ./em_run.sh -S 重新选择。" >&2
    exit 1
  fi

  if [ -t 0 ]; then
    choose_model_target
  else
    MODEL_TARGET="${DEFAULT_MODEL_TARGET}"
    printf "%s\n" "${MODEL_TARGET}" > "${MODEL_TARGET_CONFIG}"
    echo "未检测到交互式终端，默认选择 ${MODEL_TARGET}。之后用 ./em_run.sh -S 可重新选择。"
  fi
}

resolve_runtime_profile() {
  if [ -z "${MODEL_TARGET}" ] || [ -z "${MODEL_FAMILY[$MODEL_TARGET]+x}" ]; then
    echo "未知仿真模型：${MODEL_TARGET}" >&2
    exit 1
  fi

  EXECUTABLE="${MODEL_EXECUTABLE[$MODEL_TARGET]}"

  if [ -z "${MERGED_CONFIG}" ] && [ -z "${SIM_CONFIG}" ]; then
    PASSTHROUGH_ARGS+=(--sim-config "${MODEL_SIM_CONFIG[$MODEL_TARGET]}")
    SIM_CONFIG="${MODEL_SIM_CONFIG[$MODEL_TARGET]}"
  fi

  if [ -z "${MERGED_CONFIG}" ] && [ -z "${ROBOT_CONFIG}" ]; then
    PASSTHROUGH_ARGS+=(--robot-config "${MODEL_ROBOT_CONFIG[$MODEL_TARGET]}")
    ROBOT_CONFIG="${MODEL_ROBOT_CONFIG[$MODEL_TARGET]}"
  fi

  if [ ! -x "${EXECUTABLE}" ]; then
    echo "目标可执行文件不存在或不可执行：${EXECUTABLE}" >&2
    echo "请先构建对应目标，例如：cmake --build build --target ${EXECUTABLE##*/}" >&2
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

load_model_registry
load_model_target
resolve_runtime_profile

export AUSIM_DYNAMIC_OBSTACLE_ENABLED_OVERRIDE="${MODEL_DYNAMIC_OBSTACLE_ENABLED[$MODEL_TARGET]}"
if [ -n "${MODEL_DYNAMIC_OBSTACLE_CONFIG[$MODEL_TARGET]}" ]; then
  export AUSIM_DYNAMIC_OBSTACLE_CONFIG_OVERRIDE="${MODEL_DYNAMIC_OBSTACLE_CONFIG[$MODEL_TARGET]}"
else
  unset AUSIM_DYNAMIC_OBSTACLE_CONFIG_OVERRIDE
fi

if [ "${SHOW_HELP}" -eq 0 ] && [ "${MODEL_DYNAMIC_OBSTACLE_ENABLED[$MODEL_TARGET]}" = "1" ]; then
  if [ ! -f "${GENERATOR}" ]; then
    echo "动态障碍生成器不存在：${GENERATOR}" >&2
    exit 1
  fi

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
  GENERATOR_ARGS+=(--enable-dynamic-obstacles)
  if [ -n "${MODEL_DYNAMIC_OBSTACLE_CONFIG[$MODEL_TARGET]}" ]; then
    GENERATOR_ARGS+=(--obstacle-config "${MODEL_DYNAMIC_OBSTACLE_CONFIG[$MODEL_TARGET]}")
  fi

  GENERATED_SCENE_XML="$(python3 "${GENERATOR}" "${GENERATOR_ARGS[@]}")"
  export AUSIM_SCENE_XML_OVERRIDE="${GENERATED_SCENE_XML}"
fi

exec "${EXECUTABLE}" "${PASSTHROUGH_ARGS[@]}"
