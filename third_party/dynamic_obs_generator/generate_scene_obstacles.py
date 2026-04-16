#!/usr/bin/env python3
"""
Generate a MuJoCo scene XML with dynamic obstacles injected from obstacle.yaml.

This script can be used directly with explicit XML/config paths, or it can
resolve the active scene and obstacle config from the same CLI flags that
`quadrotor` accepts.
"""

from __future__ import annotations

import argparse
import os
import random
import shutil
import sys
import xml.etree.ElementTree as xml_et
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import yaml


REPO_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_SCENE_XML = REPO_ROOT / "assets" / "crazyfile" / "scene.xml"
IDENTITY_QUAT = "1 0 0 0"
OBSTACLE_NAME_PREFIX = "dynamic_obs_"
EPSILON = 1e-9


@dataclass(frozen=True)
class ObstacleConfig:
    random_seed: int
    dynamic: bool
    debug: bool
    mode: str
    radius: float
    box_size: float
    range_x_min: float
    range_x_max: float
    range_y_min: float
    range_y_max: float
    range_z_min: float
    range_z_max: float
    obstacle_count: int
    min_speed: float
    max_speed: float
    collision_enabled: bool


@dataclass(frozen=True)
class SceneResolution:
    input_xml: Path
    obstacle_config_path: Path | None
    dynamic_obstacle_enabled: bool


@dataclass(frozen=True)
class GeneratedObstacle:
    name: str
    geom_type: str
    size: list[float]
    position: list[float]
    half_extents: tuple[float, float, float]


def log(message: str) -> None:
    print(message, file=sys.stderr)


def load_yaml_file(path: Path) -> dict[str, Any]:
    with path.open("r", encoding="utf-8") as stream:
        data = yaml.safe_load(stream) or {}
    if not isinstance(data, dict):
        raise ValueError(f"YAML root must be a map: {path}")
    return data


def resolve_path(base_config_path: Path, maybe_relative_path: str | Path) -> Path:
    candidate = Path(maybe_relative_path)
    if candidate.is_absolute():
        return candidate.resolve()
    return (base_config_path.parent / candidate).resolve()


def resolve_default_sim_config() -> Path:
    candidates = [
        REPO_ROOT / "quadrotor" / "cfg" / "sim_config.yaml",
        REPO_ROOT / "cfg" / "sim_config.yaml",
        REPO_ROOT.parent / "quadrotor" / "cfg" / "sim_config.yaml",
    ]
    for candidate in candidates:
        if candidate.exists():
            return candidate.resolve()
    raise FileNotFoundError(
        "Unable to locate default simulation config. "
        "Expected quadrotor/cfg/sim_config.yaml."
    )


def resolve_scene_inputs(
    merged_config_path: str | None,
    sim_config_path: str | None,
    robot_config_path: str | None,
) -> SceneResolution:
    if merged_config_path:
        merged_path = Path(merged_config_path).resolve()
        merged_root = load_yaml_file(merged_path)
        model_node = merged_root.get("model", {})
        dynamic_node = merged_root.get("dynamic_obstacle", {})
        input_xml = resolve_path(merged_path, model_node["scene_xml"]) if "scene_xml" in model_node else DEFAULT_SCENE_XML.resolve()
        obstacle_config_path = (
            resolve_path(merged_path, dynamic_node["config_path"])
            if "config_path" in dynamic_node
            else None
        )
        return SceneResolution(
            input_xml=input_xml,
            obstacle_config_path=obstacle_config_path,
            dynamic_obstacle_enabled=bool(dynamic_node.get("enabled", False)),
        )

    sim_path = Path(sim_config_path).resolve() if sim_config_path else resolve_default_sim_config()
    sim_root = load_yaml_file(sim_path)

    input_xml = DEFAULT_SCENE_XML.resolve()
    sim_model = sim_root.get("model", {})
    if "scene_xml" in sim_model:
        input_xml = resolve_path(sim_path, sim_model["scene_xml"])

    robot_path: Path | None = None
    if robot_config_path:
        robot_path = Path(robot_config_path).resolve()
    elif isinstance(sim_root.get("robot_config"), str):
        robot_path = resolve_path(sim_path, sim_root["robot_config"])

    if robot_path is not None:
        robot_root = load_yaml_file(robot_path)
        robot_model = robot_root.get("model", {})
        if "scene_xml" in robot_model:
            input_xml = resolve_path(robot_path, robot_model["scene_xml"])

    dynamic_node = sim_root.get("dynamic_obstacle", {})
    obstacle_config_path = (
        resolve_path(sim_path, dynamic_node["config_path"])
        if "config_path" in dynamic_node
        else None
    )

    return SceneResolution(
        input_xml=input_xml,
        obstacle_config_path=obstacle_config_path,
        dynamic_obstacle_enabled=bool(dynamic_node.get("enabled", False)),
    )


def load_obstacle_config(path: Path) -> ObstacleConfig:
    root = load_yaml_file(path)
    mode = str(root.get("mode", "2d")).lower()
    if mode not in {"2d", "3d"}:
        raise ValueError(f"Unsupported obstacle mode '{mode}'. Use '2d' or '3d'.")

    box_size = resolve_box_size(root, path)
    range_node = root.get("range", {})
    config = ObstacleConfig(
        random_seed=int(root.get("random_seed", 42)),
        dynamic=bool(root.get("dynamic", True)),
        debug=bool(root.get("debug", False)),
        mode=mode,
        radius=float(root.get("radius", 0.3)),
        box_size=box_size,
        range_x_min=float(range_node.get("x_min", -5.0)),
        range_x_max=float(range_node.get("x_max", 5.0)),
        range_y_min=float(range_node.get("y_min", -5.0)),
        range_y_max=float(range_node.get("y_max", 5.0)),
        range_z_min=float(range_node.get("z_min", 0.0)),
        range_z_max=float(range_node.get("z_max", 2.0)),
        obstacle_count=int(root.get("obstacle_count", 0)),
        min_speed=float(root.get("min_speed", 0.0)),
        max_speed=float(root.get("max_speed", 0.0)),
        collision_enabled=bool(root.get("collision_enabled", False)),
    )
    validate_obstacle_config(config, path)
    return config


def resolve_box_size(root: dict[str, Any], path: Path) -> float:
    if "box_size" in root:
        return float(root["box_size"])

    legacy_values = [
        float(root[key])
        for key in ("box_length", "box_width", "cube_size")
        if key in root
    ]
    if not legacy_values:
        return 0.5

    first_value = legacy_values[0]
    if any(abs(value - first_value) > EPSILON for value in legacy_values[1:]):
        raise ValueError(
            f"{path}: legacy box_length/box_width/cube_size must match; "
            "please migrate to box_size"
        )
    return first_value


def validate_obstacle_config(config: ObstacleConfig, path: Path) -> None:
    if config.obstacle_count < 0:
        raise ValueError(f"{path}: obstacle_count must be non-negative")
    if config.range_x_min >= config.range_x_max:
        raise ValueError(f"{path}: range.x_min must be less than range.x_max")
    if config.range_y_min >= config.range_y_max:
        raise ValueError(f"{path}: range.y_min must be less than range.y_max")
    if config.range_z_min >= config.range_z_max:
        raise ValueError(f"{path}: range.z_min must be less than range.z_max")
    if config.radius <= 0.0:
        raise ValueError(f"{path}: radius must be positive")
    if config.box_size <= 0.0:
        raise ValueError(f"{path}: box_size must be positive")
    if config.min_speed < 0.0 or config.max_speed < 0.0:
        raise ValueError(f"{path}: min_speed and max_speed must be non-negative")
    if config.min_speed > config.max_speed:
        raise ValueError(f"{path}: min_speed must not exceed max_speed")


def list_to_str(values: list[float]) -> str:
    return " ".join(f"{value:.6g}" for value in values)


def obstacle_palette(index: int) -> str:
    palette = [
        "0.82 0.26 0.26 1",
        "0.2 0.62 0.4 1",
        "0.22 0.42 0.8 1",
        "0.84 0.68 0.2 1",
        "0.58 0.32 0.78 1",
        "0.14 0.64 0.7 1",
    ]
    return palette[index % len(palette)]


def rebase_file_attributes(root: xml_et.Element, input_xml: Path, output_xml: Path) -> None:
    for element in root.iter():
        file_attr = element.attrib.get("file")
        if not file_attr:
            continue

        source_path = Path(file_attr)
        if source_path.is_absolute():
            resolved_path = source_path
        else:
            resolved_path = (input_xml.parent / source_path).resolve()

        try:
            rebased_path = resolved_path.relative_to(output_xml.parent)
        except ValueError:
            rebased_path = Path(os.path.relpath(resolved_path, output_xml.parent))

        element.attrib["file"] = rebased_path.as_posix()


def remove_dynamic_obstacles(root: xml_et.Element) -> int:
    removed = 0
    for parent in root.iter():
        for child in list(parent):
            if child.tag != "geom":
                continue
            name = child.attrib.get("name", "")
            if name.startswith(OBSTACLE_NAME_PREFIX):
                parent.remove(child)
                removed += 1
    return removed


def choose_shape(index: int, mode: str, rng: random.Random) -> str:
    if mode == "2d":
        return "cylinder" if index % 2 == 0 else "box"
    return "sphere" if rng.random() < 0.5 else "cube"


def half_extents_for_shape(shape: str, config: ObstacleConfig) -> tuple[float, float, float]:
    if shape == "cylinder":
        half_height = max((config.range_z_max - config.range_z_min) * 0.5, EPSILON)
        return config.radius, config.radius, half_height
    if shape == "box":
        half_height = max((config.range_z_max - config.range_z_min) * 0.5, EPSILON)
        half_size = config.box_size * 0.5
        return half_size, half_size, half_height
    if shape == "sphere":
        return config.radius, config.radius, config.radius
    if shape == "cube":
        half_size = config.box_size * 0.5
        return half_size, half_size, half_size
    raise ValueError(f"Unsupported obstacle shape: {shape}")


def geom_size_for_shape(shape: str, config: ObstacleConfig) -> tuple[str, list[float]]:
    if shape == "cylinder":
        half_height = max((config.range_z_max - config.range_z_min) * 0.5, EPSILON)
        return "cylinder", [config.radius, half_height]
    if shape == "box":
        half_height = max((config.range_z_max - config.range_z_min) * 0.5, EPSILON)
        half_size = config.box_size * 0.5
        return "box", [half_size, half_size, half_height]
    if shape == "sphere":
        return "sphere", [config.radius]
    if shape == "cube":
        half_size = config.box_size * 0.5
        return "box", [half_size, half_size, half_size]
    raise ValueError(f"Unsupported obstacle shape: {shape}")


def sample_position(
    shape: str,
    config: ObstacleConfig,
    rng: random.Random,
) -> tuple[list[float], tuple[float, float, float]]:
    half_x, half_y, half_z = half_extents_for_shape(shape, config)

    x_min = config.range_x_min + half_x
    x_max = config.range_x_max - half_x
    y_min = config.range_y_min + half_y
    y_max = config.range_y_max - half_y
    if x_min > x_max or y_min > y_max:
        raise ValueError("Obstacle footprint does not fit inside configured XY range")

    x = rng.uniform(x_min, x_max)
    y = rng.uniform(y_min, y_max)

    if config.mode == "2d":
        z = config.range_z_min + half_z
    else:
        z_min = config.range_z_min + half_z
        z_max = config.range_z_max - half_z
        if z_min > z_max:
            raise ValueError("Obstacle size does not fit inside configured Z range")
        z = rng.uniform(z_min, z_max)

    return [x, y, z], (half_x, half_y, half_z)


def aabb_overlaps(
    center_a: list[float],
    half_a: tuple[float, float, float],
    center_b: list[float],
    half_b: tuple[float, float, float],
    margin: float = 0.05,
) -> bool:
    return (
        abs(center_a[0] - center_b[0]) < (half_a[0] + half_b[0] + margin)
        and abs(center_a[1] - center_b[1]) < (half_a[1] + half_b[1] + margin)
        and abs(center_a[2] - center_b[2]) < (half_a[2] + half_b[2] + margin)
    )


def generate_obstacles(config: ObstacleConfig) -> list[GeneratedObstacle]:
    rng = random.Random(None if config.random_seed == 0 else config.random_seed)
    obstacles: list[GeneratedObstacle] = []
    max_attempts = 256

    for index in range(config.obstacle_count):
        shape = choose_shape(index, config.mode, rng)
        geom_type, size = geom_size_for_shape(shape, config)
        best_position: list[float] | None = None
        best_half_extents: tuple[float, float, float] | None = None

        for _ in range(max_attempts):
            position, half_extents = sample_position(shape, config, rng)
            if any(
                aabb_overlaps(position, half_extents, existing.position, existing.half_extents)
                for existing in obstacles
            ):
                continue
            best_position = position
            best_half_extents = half_extents
            break

        if best_position is None or best_half_extents is None:
            position, half_extents = sample_position(shape, config, rng)
            best_position = position
            best_half_extents = half_extents

        obstacles.append(
            GeneratedObstacle(
                name=f"{OBSTACLE_NAME_PREFIX}{index}",
                geom_type=geom_type,
                size=size,
                position=best_position,
                half_extents=best_half_extents,
            )
        )

    return obstacles


def ensure_worldbody(root: xml_et.Element) -> xml_et.Element:
    worldbody = root.find("worldbody")
    if worldbody is None:
        raise ValueError("Input XML must contain a <worldbody> element")
    return worldbody


def inject_obstacles(worldbody: xml_et.Element, obstacles: list[GeneratedObstacle], collision_enabled: bool) -> None:
    contype = "1" if collision_enabled else "0"
    conaffinity = "1" if collision_enabled else "0"

    for index, obstacle in enumerate(obstacles):
        geom = xml_et.SubElement(worldbody, "geom")
        geom.attrib["name"] = obstacle.name
        geom.attrib["type"] = obstacle.geom_type
        geom.attrib["size"] = list_to_str(obstacle.size)
        geom.attrib["pos"] = list_to_str(obstacle.position)
        geom.attrib["quat"] = IDENTITY_QUAT
        geom.attrib["contype"] = contype
        geom.attrib["conaffinity"] = conaffinity
        geom.attrib["rgba"] = obstacle_palette(index)


def write_scene_xml(
    input_xml: Path,
    output_xml: Path,
    obstacle_config: ObstacleConfig | None,
    dynamic_obstacle_enabled: bool,
) -> None:
    if not input_xml.exists():
        raise FileNotFoundError(f"Input XML does not exist: {input_xml}")

    tree = xml_et.parse(input_xml)
    root = tree.getroot()
    worldbody = ensure_worldbody(root)
    removed_count = remove_dynamic_obstacles(root)

    generated_obstacles: list[GeneratedObstacle] = []
    if dynamic_obstacle_enabled and obstacle_config is not None and obstacle_config.obstacle_count > 0:
        generated_obstacles = generate_obstacles(obstacle_config)
        inject_obstacles(worldbody, generated_obstacles, obstacle_config.collision_enabled)

    output_xml.parent.mkdir(parents=True, exist_ok=True)
    rebase_file_attributes(root, input_xml, output_xml)
    xml_et.indent(tree, space="  ")
    tree.write(output_xml, encoding="utf-8", xml_declaration=True)

    log(
        "[dynamic_obs_generator] "
        f"source={input_xml} output={output_xml} "
        f"removed_old={removed_count} generated={len(generated_obstacles)} "
        f"mode={(obstacle_config.mode if obstacle_config else 'n/a')} "
        f"motion={'enabled' if obstacle_config and obstacle_config.dynamic else 'static'}"
    )


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Generate a MuJoCo scene with dynamic obstacles from obstacle.yaml",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )

    parser.add_argument("--config", help="Merged quadrotor config path")
    parser.add_argument("--sim-config", help="Simulation config path")
    parser.add_argument("--robot-config", help="Robot config override path")
    parser.add_argument("-i", "--input", help="Input MuJoCo XML path (overrides config resolution)")
    parser.add_argument(
        "--obstacle-config",
        help="Obstacle YAML path (overrides dynamic_obstacle.config_path resolution)",
    )
    parser.add_argument(
        "-o",
        "--output",
        help="Output MuJoCo XML path. Defaults to <input>.dynamic_obstacles.xml",
    )
    parser.add_argument(
        "--print-output-path",
        action="store_true",
        help="Print the resolved output scene path to stdout after generation",
    )
    parser.add_argument(
        "--enable-dynamic-obstacles",
        action="store_true",
        help="Force-enable dynamic obstacle generation regardless of sim_config content",
    )

    return parser.parse_args()


def main() -> int:
    args = parse_args()

    resolution = resolve_scene_inputs(args.config, args.sim_config, args.robot_config)
    input_xml = Path(args.input).resolve() if args.input else resolution.input_xml
    dynamic_obstacle_enabled = bool(args.enable_dynamic_obstacles or resolution.dynamic_obstacle_enabled)
    obstacle_config_path = (
        Path(args.obstacle_config).resolve()
        if args.obstacle_config
        else resolution.obstacle_config_path
    )

    obstacle_config: ObstacleConfig | None = None
    if obstacle_config_path is not None and obstacle_config_path.exists():
        obstacle_config = load_obstacle_config(obstacle_config_path)
    elif dynamic_obstacle_enabled:
        raise FileNotFoundError(
            "Dynamic obstacles are enabled, but obstacle config could not be resolved."
        )

    output_xml = (
        Path(args.output).resolve()
        if args.output
        else input_xml.with_name(f"{input_xml.stem}.dynamic_obstacles.xml")
    )

    if not dynamic_obstacle_enabled and obstacle_config is None:
        output_xml.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(input_xml, output_xml)
        log(
            "[dynamic_obs_generator] "
            f"dynamic obstacles disabled, copied source scene to {output_xml}"
        )
        if args.print_output_path:
            print(output_xml)
        return 0

    write_scene_xml(
        input_xml=input_xml,
        output_xml=output_xml,
        obstacle_config=obstacle_config,
        dynamic_obstacle_enabled=dynamic_obstacle_enabled,
    )
    if args.print_output_path:
        print(output_xml)
    return 0


if __name__ == "__main__":
    sys.exit(main())
