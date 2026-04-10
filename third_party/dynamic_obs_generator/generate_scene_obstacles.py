#!/usr/bin/env python3
"""
Dynamic Obstacle Scene Generator for MuJoCo

This script generates a MuJoCo XML scene with pre-defined obstacle geoms
that can be controlled by the DynamicObstacleManager at runtime.

Usage:
    python3 generate_scene_obstacles.py --input scene.xml --output scene_with_obstacles.xml
    python3 generate_scene_obstacles.py --input scene.xml --output scene_with_obstacles.xml --count 10 --mode 2d
"""

import argparse
import xml.etree.ElementTree as xml_et
import numpy as np
import random

ROBOT = "go2"

def euler_to_quat(roll, pitch, yaw):
    """Convert euler angles (zyx) to quaternion."""
    cx = np.cos(roll / 2)
    sx = np.sin(roll / 2)
    cy = np.cos(pitch / 2)
    sy = np.sin(pitch / 2)
    cz = np.cos(yaw / 2)
    sz = np.sin(yaw / 2)

    return np.array([
        cx * cy * cz + sx * sy * sz,
        sx * cy * cz - cx * sy * sz,
        cx * sy * cz + sx * cy * sz,
        cx * cy * sz - sx * sy * cz,
    ], dtype=np.float64)


def list_to_str(vec):
    """Convert list/array to space-separated string."""
    return " ".join(str(s) for s in vec)


def add_obstacle_to_worldbody(worldbody, name, geom_type, size, position, euler=[0, 0, 0], contype=0, conaffinity=0):
    """Add a single obstacle geom to the worldbody.

    Note: contype=0 and conaffinity=0 means the obstacle does not participate in
    physical collision detection. It will still be rendered and can be used for
    sensing (e.g., ray casting for depth sensors). This prevents the obstacles
    from interfering with the drone's physics.

    If you want physical collision, set contype=1 and conaffinity=1.
    """
    geom = xml_et.SubElement(worldbody, "geom")
    geom.attrib["name"] = name
    geom.attrib["type"] = geom_type
    geom.attrib["size"] = list_to_str(size)
    geom.attrib["pos"] = list_to_str(position)
    quat = euler_to_quat(euler[0], euler[1], euler[2])
    geom.attrib["quat"] = list_to_str(quat)
    geom.attrib["contype"] = str(contype)
    geom.attrib["conaffinity"] = str(conaffinity)
    # Make obstacles visually distinct - dark red color
    geom.attrib["rgba"] = "0.7 0.2 0.2 1"
    return geom


def generate_obstacles_xml(args):
    """Generate obstacles based on arguments."""
    tree = xml_et.parse(args.input)
    root = tree.getroot()

    # Find or create worldbody
    worldbody = root.find("worldbody")
    if worldbody is None:
        raise ValueError("Input XML must have a <worldbody> element")

    # Find existing floor if any
    floor = worldbody.find("geom[@type='plane']")

    # Set random seed
    if args.random_seed >= 0:
        random.seed(args.random_seed)
        np.random.seed(args.random_seed)

    # Generate obstacles
    obstacles = []
    half_count = args.count // 2  # Split between two types

    if args.mode == "2d":
        # 2D mode: use cylinders and boxes
        shapes = ["cylinder"] * half_count + ["box"] * (args.count - half_count)
    else:
        # 3D mode: use spheres and cubes
        shapes = ["sphere"] * half_count + ["box"] * (args.count - half_count)
        # boxes become cubes in 3D

    for i in range(args.count):
        shape = shapes[i]

        # Generate random position
        x = random.uniform(args.x_min, args.x_max)
        y = random.uniform(args.y_min, args.y_max)
        z_min = args.z_min

        if args.mode == "2d":
            # 2D: obstacles sit on ground, z is height
            z = args.z_max / 2  # center of obstacle
        else:
            # 3D: random height
            z = random.uniform(args.z_min, args.z_max)

        position = [x, y, z]

        # Generate size based on shape
        if shape == "cylinder":
            size = [args.radius, args.radius]  # MuJoCo cylinder: [radius, half-height]
        elif shape == "sphere":
            size = [args.radius]  # MuJoCo sphere: [radius]
        elif shape == "box":
            if args.mode == "2d":
                # Box for 2D: length x width x height (height from ground)
                size = [args.box_length / 2, args.box_width / 2, args.z_max / 2]
            else:
                # Cube for 3D
                size = [args.cube_size / 2, args.cube_size / 2, args.cube_size / 2]
        else:
            size = [args.radius]

        # Collision settings
        contype = 1 if args.collision else 0
        conaffinity = 1 if args.collision else 0

        # Add to worldbody
        name = f"dynamic_obs_{i}"
        geom_type = "cylinder" if shape == "cylinder" else ("sphere" if shape == "sphere" else "box")
        add_obstacle_to_worldbody(worldbody, name, geom_type, size, position, contype=contype, conaffinity=conaffinity)

        obstacles.append({
            "name": name,
            "shape": shape,
            "position": position,
            "size": size
        })

        print(f"Added obstacle: {name} ({shape}) at ({x:.2f}, {y:.2f}, {z:.2f})")

    # Write output
    tree.write(args.output, encoding="utf-8", xml_declaration=True)
    print(f"\nGenerated scene with {args.count} obstacles written to: {args.output}")
    print(f"Collision mode: {'enabled' if args.collision else 'disabled (sensing only)'}")
    print(f"Update your obstacle.yaml to match: count={args.count}, mode={args.mode}")

    return obstacles


def main():
    parser = argparse.ArgumentParser(
        description="Generate MuJoCo XML scene with dynamic obstacles",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Generate 10 obstacles in 2D mode (default: no physical collision)
  python3 generate_scene_obstacles.py -i scene.xml -o scene_obs.xml -c 10 --mode 2d

  # Generate 15 obstacles in 3D mode
  python3 generate_scene_obstacles.py -i scene.xml -o scene_obs.xml -c 15 --mode 3d

  # Generate obstacles with physical collision enabled
  python3 generate_scene_obstacles.py -i scene.xml -o scene_obs.xml -c 10 --collision

  # Custom range and size
  python3 generate_scene_obstacles.py -i scene.xml -o scene_obs.xml -c 8 \\
      --x-min -2 --x-max 2 --y-min -2 --y-max 2 --z-min 0 --z-max 1.5 \\
      --radius 0.2 --mode 2d
        """
    )

    parser.add_argument("-i", "--input", required=True, help="Input XML file")
    parser.add_argument("-o", "--output", required=True, help="Output XML file")
    parser.add_argument("-c", "--count", type=int, default=10, help="Number of obstacles (default: 10)")
    parser.add_argument("--mode", choices=["2d", "3d"], default="2d", help="Generation mode (default: 2d)")
    parser.add_argument("--random-seed", type=int, default=42, help="Random seed (default: 42)")
    parser.add_argument("--collision", action="store_true", help="Enable physical collision for obstacles (default: disabled for sensing-only)")

    # Range parameters
    parser.add_argument("--x-min", type=float, default=-3.0, help="Minimum x coordinate")
    parser.add_argument("--x-max", type=float, default=3.0, help="Maximum x coordinate")
    parser.add_argument("--y-min", type=float, default=-3.0, help="Minimum y coordinate")
    parser.add_argument("--y-max", type=float, default=3.0, help="Maximum y coordinate")
    parser.add_argument("--z-min", type=float, default=0.0, help="Minimum z coordinate")
    parser.add_argument("--z-max", type=float, default=2.0, help="Maximum z coordinate (height)")

    # Size parameters
    parser.add_argument("--radius", type=float, default=0.3, help="Radius for cylinder/sphere")
    parser.add_argument("--box-length", type=float, default=0.5, help="Box length")
    parser.add_argument("--box-width", type=float, default=0.5, help="Box width")
    parser.add_argument("--cube-size", type=float, default=0.5, help="Cube size")

    args = parser.parse_args()

    # Validate
    if args.count < 0:
        raise ValueError("count must be non-negative")
    if args.x_min >= args.x_max:
        raise ValueError("x_min must be less than x_max")
    if args.y_min >= args.y_max:
        raise ValueError("y_min must be less than y_max")
    if args.z_min >= args.z_max:
        raise ValueError("z_min must be less than z_max")
    if args.radius <= 0:
        raise ValueError("radius must be positive")

    print(f"Generating {args.count} obstacles in {args.mode} mode")
    print(f"Range: x=[{args.x_min}, {args.x_max}], y=[{args.y_min}, {args.y_max}], z=[{args.z_min}, {args.z_max}]")

    generate_obstacles_xml(args)


if __name__ == "__main__":
    main()
