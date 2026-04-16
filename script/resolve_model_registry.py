#!/usr/bin/env python3

from __future__ import annotations

import argparse
import shlex
import sys
from pathlib import Path
from typing import NoReturn

import yaml


def fail(message: str) -> NoReturn:
    raise SystemExit(message)


def require_mapping(value: object, context: str) -> dict:
    if not isinstance(value, dict):
        fail(f"{context} must be a mapping")
    return value


def require_string(model_id: str, model: dict, field: str) -> str:
    value = model.get(field)
    if not isinstance(value, str) or not value.strip():
        fail(f"model '{model_id}' is missing required string field '{field}'")
    return value.strip()


def require_bool(model_id: str, node: dict, field: str) -> bool:
    value = node.get(field)
    if isinstance(value, bool):
        return value
    fail(f"model '{model_id}' field '{field}' must be a boolean")


def resolve_repo_path(repo_root: Path, value: str) -> Path:
    candidate = Path(value)
    if candidate.is_absolute():
        return candidate.resolve()
    return (repo_root / candidate).resolve()


def shell_quote(value: str) -> str:
    return shlex.quote(value)


def infer_family(registry_path: Path) -> str:
    try:
        return registry_path.parent.parent.name
    except IndexError as exc:
        fail(f"unable to infer family from registry path: {registry_path}")
        raise exc


def infer_executable(repo_root: Path, family: str) -> Path:
    executable_by_family = {
        "quadrotor": repo_root / "build/bin/quadrotor",
        "ground_vehicle": repo_root / "build/bin/scout",
    }
    executable_path = executable_by_family.get(family)
    if executable_path is None:
        fail(f"unsupported family for executable inference: {family}")
    return executable_path.resolve()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Resolve model registries into shell declarations.")
    parser.add_argument("repo_root", help="Repository root path")
    parser.add_argument("registry_paths", nargs="+", help="Registry YAML files")
    return parser.parse_args()


def load_registry(repo_root: Path, registry_paths: list[Path]) -> tuple[list[str], str, dict[str, dict[str, str]]]:
    order: list[str] = []
    mappings: dict[str, dict[str, str]] = {
        "MODEL_DISPLAY_NAME": {},
        "MODEL_FAMILY": {},
        "MODEL_EXECUTABLE": {},
        "MODEL_SIM_CONFIG": {},
        "MODEL_ROBOT_CONFIG": {},
        "MODEL_DYNAMIC_OBSTACLE_ENABLED": {},
        "MODEL_DYNAMIC_OBSTACLE_CONFIG": {},
    }
    default_target: str | None = None

    for registry_path in registry_paths:
        if not registry_path.is_file():
            fail(f"registry file not found: {registry_path}")

        root = yaml.safe_load(registry_path.read_text(encoding="utf-8")) or {}
        root = require_mapping(root, str(registry_path))

        local_default = root.get("default_target")
        if local_default is not None:
            if not isinstance(local_default, str) or not local_default.strip():
                fail(f"{registry_path}: default_target must be a non-empty string")
            if default_target is not None:
                fail(f"multiple registry files declare default_target: {default_target}, {local_default}")
            default_target = local_default.strip()

        models = root.get("models")
        if not isinstance(models, list) or not models:
            fail(f"{registry_path}: models must be a non-empty list")

        for entry in models:
            model = require_mapping(entry, f"{registry_path} model entry")
            model_id = require_string("<unknown>", model, "id")
            if model_id in mappings["MODEL_FAMILY"]:
                fail(f"duplicate model id in registries: {model_id}")

            order.append(model_id)
            family = infer_family(registry_path)
            mappings["MODEL_FAMILY"][model_id] = family
            mappings["MODEL_DISPLAY_NAME"][model_id] = model_id

            executable_path = infer_executable(repo_root, family)
            sim_config_path = resolve_repo_path(repo_root, require_string(model_id, model, "sim_config"))
            robot_config_path = resolve_repo_path(repo_root, require_string(model_id, model, "robot_config"))
            scene_xml_path = resolve_repo_path(repo_root, require_string(model_id, model, "scene_xml"))

            if not sim_config_path.is_file():
                fail(f"model '{model_id}' sim_config not found: {sim_config_path}")
            if not robot_config_path.is_file():
                fail(f"model '{model_id}' robot_config not found: {robot_config_path}")
            if not scene_xml_path.is_file():
                fail(f"model '{model_id}' scene_xml not found: {scene_xml_path}")

            mappings["MODEL_EXECUTABLE"][model_id] = str(executable_path)
            mappings["MODEL_SIM_CONFIG"][model_id] = str(sim_config_path)
            mappings["MODEL_ROBOT_CONFIG"][model_id] = str(robot_config_path)

            dynamic_node = model.get("dynamic_obstacle")
            if not isinstance(dynamic_node, dict):
                fail(f"model '{model_id}' field 'dynamic_obstacle' must be a mapping")

            dynamic_enabled = require_bool(model_id, dynamic_node, "enabled")
            mappings["MODEL_DYNAMIC_OBSTACLE_ENABLED"][model_id] = "1" if dynamic_enabled else "0"

            config_value = dynamic_node.get("config_path")
            if not isinstance(config_value, str) or not config_value.strip():
                fail(f"model '{model_id}' dynamic_obstacle.config_path must be a non-empty string")
            resolved_dynamic_config = resolve_repo_path(repo_root, config_value.strip())
            if not resolved_dynamic_config.is_file():
                fail(f"model '{model_id}' dynamic obstacle config not found: {resolved_dynamic_config}")
            mappings["MODEL_DYNAMIC_OBSTACLE_CONFIG"][model_id] = str(resolved_dynamic_config)

    if not order:
        fail("no models available in registry")
    if default_target is None:
        default_target = order[0]
    if default_target not in mappings["MODEL_FAMILY"]:
        fail(f"default_target '{default_target}' is not declared in any registry")

    return order, default_target, mappings


def emit_shell(order: list[str], default_target: str, mappings: dict[str, dict[str, str]]) -> str:
    lines = [
        "declare -ga MODEL_ORDER=(" + " ".join(shell_quote(model_id) for model_id in order) + ")",
        "declare -g DEFAULT_MODEL_TARGET=" + shell_quote(default_target),
    ]

    for name in (
        "MODEL_DISPLAY_NAME",
        "MODEL_FAMILY",
        "MODEL_EXECUTABLE",
        "MODEL_SIM_CONFIG",
        "MODEL_ROBOT_CONFIG",
        "MODEL_DYNAMIC_OBSTACLE_ENABLED",
        "MODEL_DYNAMIC_OBSTACLE_CONFIG",
    ):
        lines.append(f"declare -gA {name}=(")
        for model_id in order:
            lines.append(f"  [{shell_quote(model_id)}]={shell_quote(mappings[name][model_id])}")
        lines.append(")")

    return "\n".join(lines)


def main() -> int:
    args = parse_args()
    repo_root = Path(args.repo_root).resolve()
    registry_paths = [Path(path).resolve() for path in args.registry_paths]
    order, default_target, mappings = load_registry(repo_root, registry_paths)
    print(emit_shell(order, default_target, mappings))
    return 0


if __name__ == "__main__":
    sys.exit(main())
