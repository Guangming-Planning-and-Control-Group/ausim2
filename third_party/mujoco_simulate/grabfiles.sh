#!/usr/bin/env bash

set -euo pipefail

usage() {
	cat <<'EOF'
Usage:
	./grabfiles.sh /path/to/mujoco-3.3.x

Description:
	Copy all files used by third_party/mujoco_simulate from a MuJoCo source tree.
	The script will copy known simulate sources plus lodepng dependencies.
	从 MuJoCo 源码中复制third_party/mujoco_simulate 使用的所有文件。
	该脚本将复制已知的模拟源以及 lodepng 依赖项。
EOF
}

if [[ "${1:-}" == "-h" || "${1:-}" == "--help" ]]; then
	usage
	exit 0
fi

if [[ $# -ne 1 ]]; then
	usage
	exit 1
fi

MUJOCO_SRC="$1"
if [[ ! -d "$MUJOCO_SRC" ]]; then
	echo "[ERROR] MuJoCo path does not exist: $MUJOCO_SRC" >&2
	exit 1
fi

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

copy_rel() {
	local rel="$1"
	local src="$MUJOCO_SRC/$rel"
	local dst="$SCRIPT_DIR/$(basename "$rel")"
	if [[ ! -f "$src" ]]; then
		echo "[ERROR] Missing required file: $src" >&2
		exit 1
	fi
	cp "$src" "$dst"
	echo "[OK] $rel -> $(basename "$rel")"
}

copy_first_found() {
	local dst_name="$1"
	shift

	local rel
	for rel in "$@"; do
		if [[ -f "$MUJOCO_SRC/$rel" ]]; then
			cp "$MUJOCO_SRC/$rel" "$SCRIPT_DIR/$dst_name"
			echo "[OK] $rel -> $dst_name"
			return 0
		fi
	done

	local hit
	hit="$(find "$MUJOCO_SRC" -type f -name "$dst_name" 2>/dev/null | head -n 1 || true)"
	if [[ -n "$hit" ]]; then
		cp "$hit" "$SCRIPT_DIR/$dst_name"
		echo "[OK] ${hit#"$MUJOCO_SRC/"} -> $dst_name"
		return 0
	fi

	echo "[ERROR] Could not locate required file: $dst_name" >&2
	return 1
}

echo "[INFO] MuJoCo source: $MUJOCO_SRC"
echo "[INFO] Destination : $SCRIPT_DIR"

# Files originating from MuJoCo simulate/.
copy_rel "simulate/array_safety.h"
copy_rel "simulate/glfw_adapter.cc"
copy_rel "simulate/glfw_adapter.h"
copy_rel "simulate/glfw_corevideo.h"
copy_rel "simulate/glfw_dispatch.cc"
copy_rel "simulate/glfw_dispatch.h"
copy_rel "simulate/platform_ui_adapter.cc"
copy_rel "simulate/platform_ui_adapter.h"
copy_rel "simulate/simulate.cc"
copy_rel "simulate/simulate.h"

# lodepng dependency can live under build outputs after configure/build.
copy_first_found "lodepng.h" \
	"build/_deps/lodepng-src/lodepng.h" \
	"build/_deps/lodepng-src/src/lodepng.h"

# Optional prebuilt static library used by current third_party layout.
if ! copy_first_found "liblodepng.a" "build/lib/liblodepng.a"; then
	echo "[WARN] liblodepng.a was not found."
	echo "[WARN] If you need this binary, configure/build MuJoCo first, then rerun this script."
fi

echo "[DONE] MuJoCo simulate dependencies refreshed."
