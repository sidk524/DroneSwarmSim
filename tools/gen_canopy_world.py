#!/usr/bin/env python3
"""Generate a rainforest-canopy sandbox world for PX4 SITL / Gazebo.

Emits a stratified forest built from static primitives (cylinder trunks and
branches, sphere-cluster crowns) so the depth camera and the planner's grid
map see exactly the geometry that is physically there:

  - emergent trees   : tall thick trunks, big crowns poking above the canopy
  - canopy trees     : semi-continuous crown layer at ~7-11 m with light gaps
  - understory trees : small trees at flight altitude
  - saplings         : thin vertical poles, below-voxel-resolution stress test
  - branches + twigs : angled cylinders on trunks between ~2-7 m

A spawn clearing at the origin, extra ground clearings, and vertical canopy
light gaps are kept free of vegetation so the drone can take off, land, and
transit between the under- and above-canopy regimes.

Usage:
  python3 tools/gen_canopy_world.py --seed 42 \
      --out px4_ws/PX4-Autopilot/Tools/simulation/gz/worlds/canopy.sdf

The <world name> is derived from the output filename stem, which is what
PX4_GZ_WORLD must be set to.
"""

import argparse
import math
import random
from pathlib import Path

# ---------------------------------------------------------------------------
# SDF snippets
# ---------------------------------------------------------------------------

HEADER = """<?xml version="1.0" encoding="UTF-8"?>
<sdf version="1.9">
  <world name="{world_name}">
    <physics type="ode">
      <max_step_size>0.004</max_step_size>
      <real_time_factor>1.0</real_time_factor>
      <real_time_update_rate>250</real_time_update_rate>
    </physics>
    <gravity>0 0 -9.8</gravity>
    <magnetic_field>6e-06 2.3e-05 -4.2e-05</magnetic_field>
    <atmosphere type="adiabatic"/>
    <scene>
      <grid>false</grid>
      <ambient>0.4 0.4 0.4 1</ambient>
      <background>0.55 0.7 0.85 1</background>
      <shadows>true</shadows>
    </scene>
    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 30 0 0 0</pose>
      <diffuse>0.9 0.9 0.85 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <direction>-0.4 0.2 -0.9</direction>
    </light>
    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>1 1</size>
            </plane>
          </geometry>
          <surface>
            <friction>
              <ode/>
            </friction>
            <bounce/>
            <contact/>
          </surface>
        </collision>
        <visual name="visual">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>500 500</size>
            </plane>
          </geometry>
          <material>
            <ambient>0.22 0.3 0.16 1</ambient>
            <diffuse>0.25 0.34 0.18 1</diffuse>
            <specular>0.05 0.05 0.05 1</specular>
          </material>
        </visual>
      </link>
    </model>
"""

FOOTER = """    <spherical_coordinates>
      <surface_model>EARTH_WGS84</surface_model>
      <world_frame_orientation>ENU</world_frame_orientation>
      <latitude_deg>47.397971057728974</latitude_deg>
      <longitude_deg>8.546163739800146</longitude_deg>
      <elevation>0</elevation>
    </spherical_coordinates>
  </world>
</sdf>
"""

CYLINDER_PART = """        <collision name="{name}_col">
          <pose>{pose}</pose>
          <geometry>
            <cylinder>
              <radius>{radius:.3f}</radius>
              <length>{length:.3f}</length>
            </cylinder>
          </geometry>
        </collision>
        <visual name="{name}_vis">
          <pose>{pose}</pose>
          <geometry>
            <cylinder>
              <radius>{radius:.3f}</radius>
              <length>{length:.3f}</length>
            </cylinder>
          </geometry>
          <material>
            <ambient>{color} 1</ambient>
            <diffuse>{color} 1</diffuse>
            <specular>0.05 0.05 0.05 1</specular>
          </material>
        </visual>
"""

SPHERE_PART = """        <collision name="{name}_col">
          <pose>{pose}</pose>
          <geometry>
            <sphere>
              <radius>{radius:.3f}</radius>
            </sphere>
          </geometry>
        </collision>
        <visual name="{name}_vis">
          <pose>{pose}</pose>
          <geometry>
            <sphere>
              <radius>{radius:.3f}</radius>
            </sphere>
          </geometry>
          <material>
            <ambient>{color} 1</ambient>
            <diffuse>{color} 1</diffuse>
            <specular>0.05 0.05 0.05 1</specular>
          </material>
        </visual>
"""

MODEL_TEMPLATE = """    <model name="{name}">
      <static>true</static>
      <pose>{x:.2f} {y:.2f} 0 0 0 0</pose>
      <link name="link">
{parts}      </link>
    </model>
"""


def fmt_color(rgb):
    return "{:.2f} {:.2f} {:.2f}".format(*rgb)


def cylinder(name, pose, radius, length, rgb):
    return CYLINDER_PART.format(
        name=name, pose=pose, radius=radius, length=length, color=fmt_color(rgb))


def sphere(name, pose, radius, rgb):
    return SPHERE_PART.format(
        name=name, pose=pose, radius=radius, color=fmt_color(rgb))


# ---------------------------------------------------------------------------
# Tree builders — parts are positioned relative to the model origin (trunk base)
# ---------------------------------------------------------------------------

def trunk_color(rng):
    b = rng.uniform(0.0, 0.08)
    return (0.32 + b, 0.22 + b, 0.12 + b)


def crown_color(rng):
    # Broad green variation, from deep bottle green to yellowish light green,
    # so the closed canopy reads as many individual crowns like real forest.
    g = rng.uniform(-0.10, 0.18)
    y = rng.uniform(0.0, 0.12)  # yellow tint
    return (0.10 + g * 0.5 + y, 0.38 + g, 0.10 + g * 0.3)


TWIG_COLOR = (0.65, 0.55, 0.40)


def branch_parts(rng, trunk_h, count, twig_chance):
    """Angled cylinders radiating from the trunk between z 2 m and trunk_h-1."""
    parts = []
    z_lo, z_hi = 2.0, max(2.5, trunk_h - 1.0)
    for i in range(count):
        is_twig = rng.random() < twig_chance
        radius = rng.uniform(0.02, 0.03) if is_twig else rng.uniform(0.06, 0.15)
        length = rng.uniform(1.2, 2.2) if is_twig else rng.uniform(1.5, 3.0)
        z_attach = rng.uniform(z_lo, z_hi)
        yaw = rng.uniform(0, 2 * math.pi)
        pitch = rng.uniform(math.radians(50), math.radians(80))  # from vertical
        # cylinder axis after Ry(pitch) then Rz(yaw)
        dx = math.sin(pitch) * math.cos(yaw)
        dy = math.sin(pitch) * math.sin(yaw)
        dz = math.cos(pitch)
        cx = dx * length / 2
        cy = dy * length / 2
        cz = z_attach + dz * length / 2
        pose = "{:.3f} {:.3f} {:.3f} 0 {:.4f} {:.4f}".format(cx, cy, cz, pitch, yaw)
        rgb = TWIG_COLOR if is_twig else trunk_color(rng)
        parts.append(cylinder(f"branch_{i}", pose, radius, length, rgb))
    return parts


def build_emergent(rng):
    trunk_r = rng.uniform(0.5, 0.7)
    trunk_h = rng.uniform(14.0, 16.0)
    parts = [cylinder("trunk", f"0 0 {trunk_h / 2:.2f} 0 0 0",
                      trunk_r, trunk_h, trunk_color(rng))]
    rgb = crown_color(rng)
    n_blobs = rng.randint(5, 7)
    for i in range(n_blobs):
        r = rng.uniform(2.5, 4.0)
        ox = rng.uniform(-2.5, 2.5)
        oy = rng.uniform(-2.5, 2.5)
        oz = trunk_h + rng.uniform(-1.0, 1.5)
        parts.append(sphere(f"crown_{i}", f"{ox:.2f} {oy:.2f} {oz:.2f} 0 0 0", r, rgb))
    parts += branch_parts(rng, trunk_h * 0.6, rng.randint(2, 4), 0.25)
    crown_r = 6.5  # widest horizontal extent, used for gap rejection
    return parts, trunk_h, crown_r


def build_canopy_tree(rng):
    trunk_r = rng.uniform(0.20, 0.4)
    trunk_h = rng.uniform(7.0, 12.0)  # wide spread -> lumpy canopy top
    parts = [cylinder("trunk", f"0 0 {trunk_h / 2:.2f} 0 0 0",
                      trunk_r, trunk_h, trunk_color(rng))]
    rgb = crown_color(rng)
    n_blobs = rng.randint(3, 5)
    for i in range(n_blobs):
        r = rng.uniform(1.8, 3.0)
        ox = rng.uniform(-1.8, 1.8)
        oy = rng.uniform(-1.8, 1.8)
        oz = trunk_h + rng.uniform(-1.5, 1.0)
        parts.append(sphere(f"crown_{i}", f"{ox:.2f} {oy:.2f} {oz:.2f} 0 0 0", r, rgb))
    parts += branch_parts(rng, trunk_h - 2.0, rng.randint(2, 5), 0.25)
    crown_r = 4.5
    return parts, trunk_h, crown_r


def build_understory_tree(rng):
    trunk_r = rng.uniform(0.10, 0.15)
    trunk_h = rng.uniform(3.0, 6.0)
    parts = [cylinder("trunk", f"0 0 {trunk_h / 2:.2f} 0 0 0",
                      trunk_r, trunk_h, trunk_color(rng))]
    rgb = crown_color(rng)
    r = rng.uniform(0.8, 1.5)
    parts.append(sphere("crown_0", f"0 0 {trunk_h:.2f} 0 0 0", r, rgb))
    if trunk_h > 3.5:
        parts += branch_parts(rng, trunk_h - 0.5, rng.randint(1, 3), 0.4)
    return parts, trunk_h, r


def build_sapling(rng):
    trunk_r = rng.uniform(0.03, 0.05)
    trunk_h = rng.uniform(2.0, 4.0)
    rgb = (0.28, 0.30, 0.14)
    parts = [cylinder("trunk", f"0 0 {trunk_h / 2:.2f} 0 0 0", trunk_r, trunk_h, rgb)]
    return parts, trunk_h, trunk_r


# ---------------------------------------------------------------------------
# Placement
# ---------------------------------------------------------------------------

def place(rng, existing, x_range, y_range, spacing, clearings,
          gap_centers=None, gap_block_r=0.0, attempts=600):
    """Rejection-sample a position keeping min spacing to existing trunks,
    staying out of ground clearings and (optionally) canopy light gaps."""
    for _ in range(attempts):
        x = rng.uniform(*x_range)
        y = rng.uniform(*y_range)
        if any(math.hypot(x - cx, y - cy) < cr for cx, cy, cr in clearings):
            continue
        if gap_centers and any(
                math.hypot(x - gx, y - gy) < gap_block_r for gx, gy in gap_centers):
            continue
        if any(math.hypot(x - ex, y - ey) < max(spacing, es)
               for ex, ey, es in existing):
            continue
        return x, y
    return None


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--seed", type=int, default=42)
    ap.add_argument("--out", type=Path, required=True,
                    help="output .sdf path; world name = filename stem")
    ap.add_argument("--area-x", type=float, default=60.0)
    ap.add_argument("--area-y", type=float, default=40.0)
    ap.add_argument("--center-x", type=float, default=5.0,
                    help="forest block center (spawn stays at the origin)")
    ap.add_argument("--center-y", type=float, default=0.0)
    ap.add_argument("--emergents", type=int, default=6)
    ap.add_argument("--canopy", type=int, default=90)
    ap.add_argument("--understory", type=int, default=45)
    ap.add_argument("--saplings", type=int, default=35)
    args = ap.parse_args()

    rng = random.Random(args.seed)
    world_name = args.out.stem

    x_range = (args.center_x - args.area_x / 2, args.center_x + args.area_x / 2)
    y_range = (args.center_y - args.area_y / 2, args.center_y + args.area_y / 2)

    # Ground clearings (x, y, radius): spawn at origin + two candidates for
    # future landing-adjacent tests.
    clearings = [(0.0, 0.0, 4.0), (25.0, -12.0, 3.0), (-15.0, 12.0, 3.0)]

    # Canopy light gaps: crown-free columns for vertical transit.
    gap_centers = [(12.0, 8.0), (20.0, -5.0), (-8.0, -10.0)]
    gap_r = 3.0

    models = []
    trunks = []  # (x, y, own_spacing) of everything placed

    def add(kind, builder, count, spacing, blocked_by_gaps, crown_r_hint):
        placed = 0
        for i in range(count):
            pos = place(rng, trunks, x_range, y_range, spacing, clearings,
                        gap_centers if blocked_by_gaps else None,
                        gap_r + crown_r_hint)
            if pos is None:
                continue
            x, y = pos
            parts, _h, _r = builder(rng)
            models.append(MODEL_TEMPLATE.format(
                name=f"{kind}_{i}", x=x, y=y, parts="".join(parts)))
            trunks.append((x, y, spacing))
            placed += 1
        return placed

    # Emergent and canopy trees carry the crown layer -> keep them out of the
    # light gaps; understory and saplings may grow beneath the gaps.
    n_e = add("emergent", build_emergent, args.emergents, 5.0, True, 6.5)
    n_c = add("canopy_tree", build_canopy_tree, args.canopy, 1.8, True, 4.5)
    n_u = add("understory", build_understory_tree, args.understory, 1.0, False, 0.0)
    n_s = add("sapling", build_sapling, args.saplings, 0.6, False, 0.0)

    sdf = HEADER.format(world_name=world_name) + "".join(models) + FOOTER
    args.out.write_text(sdf)

    print(f"wrote {args.out} (world '{world_name}')")
    print(f"  emergent {n_e}/{args.emergents}, canopy {n_c}/{args.canopy}, "
          f"understory {n_u}/{args.understory}, saplings {n_s}/{args.saplings}")
    print(f"  forest x {x_range[0]:.0f}..{x_range[1]:.0f}, "
          f"y {y_range[0]:.0f}..{y_range[1]:.0f}")
    print(f"  spawn clearing r4 @ origin; ground clearings {clearings[1:]};")
    print(f"  canopy light gaps r{gap_r} @ {gap_centers}")


if __name__ == "__main__":
    main()
