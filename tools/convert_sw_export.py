#!/usr/bin/env python3
"""Convert SolidWorks ROS 2 URDF export to ps11_description package (§7.3, T1.1).

Re-runnable conversion script that:
1. Renames package to ps11_description and rewrites mesh URIs.
2. Creates root base_link at hull bounding-box centre (REP-103: X forward, Y left, Z up).
3. Renames thrusters (Link_FR..BL -> thruster_1..6) with continuous joints.
4. Strips ros2_control / gazebo blocks from core URDF.
5. Decimates visual base_link.STL to <= 50k triangles; sets collision box 0.714 x 0.564 x 0.220 m.
6. Corrects hull centre-of-mass to bounding-box centre.
7. Ensures ps11.urdf.xacro has camera_link, imu_link, altimeter_link, depth_sensor_link.
8. Updates ps11_bringup/config/vehicle.yaml with vehicle dimensions and thruster parameters.
9. Runs geometry checks against intended design and writes differences to tools/sw_export_raw/geometry_check.txt.
"""

from __future__ import annotations

import argparse
import shutil
import struct
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation as R

# Thruster mappings per specification
THRUSTER_MAP = {
    "Link_FR": ("thruster_1", "thruster_1_joint", "Joint_FR", "front right"),
    "Link_FL": ("thruster_2", "thruster_2_joint", "Joint_FL", "front left"),
    "Link_MR": ("thruster_3", "thruster_3_joint", "Joint_MR", "middle right"),
    "Link_ML": ("thruster_4", "thruster_4_joint", "Joint_ML", "middle left"),
    "Link_BR": ("thruster_5", "thruster_5_joint", "Joint_BR", "back right"),
    "Link_BL": ("thruster_6", "thruster_6_joint", "Joint_BL", "back left"),
}

EXPECTED_POSITIONS = {
    "thruster_1": np.array([0.32, -0.19, -0.05]),
    "thruster_2": np.array([0.29, 0.21, -0.02]),
    "thruster_3": np.array([-0.01, -0.22, 0.00]),
    "thruster_4": np.array([-0.01, 0.22, 0.01]),
    "thruster_5": np.array([-0.30, -0.22, 0.00]),
    "thruster_6": np.array([-0.27, 0.26, 0.00]),
}

INTENDED_ELEVATION_DEG = {
    "thruster_1": -25.0,
    "thruster_2": -25.0,
    "thruster_3": 45.0,
    "thruster_4": 45.0,
    "thruster_5": 10.0,
    "thruster_6": 10.0,
}


def read_binary_stl(filepath: Path) -> tuple[np.ndarray, np.ndarray, int]:
    """Read binary STL file returning (vertices_stacked, triangles_array, count)."""
    with open(filepath, "rb") as f:
        _ = f.read(80)
        count = struct.unpack("<I", f.read(4))[0]
        data = f.read(count * 50)

    dt = np.dtype(
        [
            ("norm", "<f4", (3,)),
            ("v1", "<f4", (3,)),
            ("v2", "<f4", (3,)),
            ("v3", "<f4", (3,)),
            ("attr", "<u2"),
        ]
    )
    arr = np.frombuffer(data, dtype=dt, count=count)
    pts = np.vstack([arr["v1"], arr["v2"], arr["v3"]])
    return pts, arr, count


def decimate_stl(
    input_path: Path, output_path: Path, target_max_triangles: int = 50000
) -> int:
    """Decimate binary STL using deterministic vertex clustering down to <= target_max_triangles."""
    pts, arr, count = read_binary_stl(input_path)
    if count <= target_max_triangles:
        shutil.copy2(input_path, output_path)
        return count

    v1 = arr["v1"]
    v2 = arr["v2"]
    v3 = arr["v3"]

    min_pt = pts.min(axis=0)

    # Binary search for grid step size
    low_step = 0.0005
    high_step = 0.05
    best_faces = None
    best_normals = None

    for _ in range(16):
        mid_step = (low_step + high_step) / 2.0
        q1 = np.round((v1 - min_pt) / mid_step).astype(np.int64)
        q2 = np.round((v2 - min_pt) / mid_step).astype(np.int64)
        q3 = np.round((v3 - min_pt) / mid_step).astype(np.int64)

        d12 = np.any(q1 != q2, axis=1)
        d23 = np.any(q2 != q3, axis=1)
        d31 = np.any(q3 != q1, axis=1)
        valid = d12 & d23 & d31
        n_valid = int(np.count_nonzero(valid))

        if n_valid > target_max_triangles:
            low_step = mid_step
        else:
            high_step = mid_step
            rv1 = q1[valid] * mid_step + min_pt
            rv2 = q2[valid] * mid_step + min_pt
            rv3 = q3[valid] * mid_step + min_pt

            edges1 = rv2 - rv1
            edges2 = rv3 - rv1
            cross = np.cross(edges1, edges2)
            lens = np.linalg.norm(cross, axis=1, keepdims=True)
            lens[lens == 0] = 1.0
            normals = cross / lens

            best_faces = (
                rv1.astype(np.float32),
                rv2.astype(np.float32),
                rv3.astype(np.float32),
            )
            best_normals = normals.astype(np.float32)

    assert best_faces is not None
    out_count = len(best_faces[0])

    output_path.parent.mkdir(parents=True, exist_ok=True)
    with open(output_path, "wb") as f:
        f.write(b"PS11 decimated base_link mesh".ljust(80, b"\0"))
        f.write(struct.pack("<I", out_count))
        out_dt = np.dtype(
            [
                ("norm", "<f4", (3,)),
                ("v1", "<f4", (3,)),
                ("v2", "<f4", (3,)),
                ("v3", "<f4", (3,)),
                ("attr", "<u2"),
            ]
        )
        out_arr = np.empty(out_count, dtype=out_dt)
        out_arr["norm"] = best_normals
        out_arr["v1"] = best_faces[0]
        out_arr["v2"] = best_faces[1]
        out_arr["v3"] = best_faces[2]
        out_arr["attr"] = 0
        f.write(out_arr.tobytes())

    return out_count


def convert_sw_export(in_dir: Path, out_dir: Path) -> None:
    """Run full conversion workflow."""
    print("==================================================================")
    print("PS11-AUV: Converting SolidWorks Export to ROS 2 ps11_description")
    print("==================================================================")

    in_meshes = in_dir / "meshes"
    in_urdf_dir = in_dir / "urdf"
    in_urdf_file = in_urdf_dir / "Urdf Assembly.urdf"

    if not in_urdf_file.exists():
        raise FileNotFoundError(f"Export URDF not found: {in_urdf_file}")

    # 1. Compute hull centre and extents from exported base_link.STL
    base_stl_in = in_meshes / "base_link.STL"
    pts, _, raw_triangle_count = read_binary_stl(base_stl_in)
    min_pt = pts.min(axis=0)
    max_pt = pts.max(axis=0)
    center_sw = (min_pt + max_pt) / 2.0
    extents_sw = max_pt - min_pt

    # Dimensions in base_link frame (X forward=SW +Y, Y left=SW -X, Z up=SW +Z)
    dim_length = float(extents_sw[1])
    dim_width = float(extents_sw[0])
    dim_height = float(extents_sw[2])

    print(f"Original base_link.STL triangles: {raw_triangle_count}")
    print(
        f"SolidWorks Hull Centre: [{center_sw[0]:.6f}, {center_sw[1]:.6f}, {center_sw[2]:.6f}]"
    )
    print(
        f"Hull Dimensions: Length={dim_length:.3f}m, Width={dim_width:.3f}m, Height={dim_height:.3f}m"
    )

    # 2. Decimate base_link.STL and copy thruster meshes to output meshes/
    out_meshes = out_dir / "meshes"
    out_meshes.mkdir(parents=True, exist_ok=True)
    decimated_count = decimate_stl(
        base_stl_in, out_meshes / "base_link.STL", target_max_triangles=50000
    )
    print(f"Decimated base_link.STL to {decimated_count} triangles (<= 50,000)")

    for mesh_file in in_meshes.glob("*.STL"):
        if mesh_file.name != "base_link.STL":
            shutil.copy2(mesh_file, out_meshes / mesh_file.name)

    # 3. Parse and transform URDF
    tree = ET.parse(in_urdf_file)
    root = tree.getroot()

    # Find exported base_link inertial data
    exported_base = root.find("./link[@name='base_link']")
    hull_mass = 12.3609887723418
    if exported_base is not None:
        mass_elem = exported_base.find("./inertial/mass")
        if mass_elem is not None and "value" in mass_elem.attrib:
            hull_mass = float(mass_elem.attrib["value"])

    # Extract thruster joints data from exported URDF
    R_sw_to_base = np.array(
        [
            [0.0, 1.0, 0.0],
            [-1.0, 0.0, 0.0],
            [0.0, 0.0, 1.0],
        ]
    )

    thruster_positions_base: dict[str, np.ndarray] = {}
    thruster_axes_base: dict[str, np.ndarray] = {}
    thruster_elev_deg: dict[str, float] = {}

    for joint in root.findall("./joint"):
        child = joint.find("child")
        if child is None:
            continue
        cname = child.attrib.get("link", "")
        if cname in THRUSTER_MAP:
            t_id, _, _, _ = THRUSTER_MAP[cname]
            origin = joint.find("origin")
            xyz_str = (
                origin.attrib.get("xyz", "0 0 0") if origin is not None else "0 0 0"
            )
            rpy_str = (
                origin.attrib.get("rpy", "0 0 0") if origin is not None else "0 0 0"
            )
            xyz_val = np.array([float(x) for x in xyz_str.split()])
            rpy_val = [float(x) for x in rpy_str.split()]

            pos_base = R_sw_to_base @ (xyz_val - center_sw)
            rot_sw = R.from_euler("xyz", rpy_val).as_matrix()
            axis_base = R_sw_to_base @ (rot_sw @ np.array([0.0, 0.0, 1.0]))
            elev = float(np.degrees(np.arcsin(np.clip(axis_base[2], -1.0, 1.0))))

            thruster_positions_base[t_id] = pos_base
            thruster_axes_base[t_id] = axis_base
            thruster_elev_deg[t_id] = elev

    # Print thruster positions and axes
    print("\n--- Thruster Kinematics in base_link (REP-103) ---")
    for t_id in sorted(thruster_positions_base.keys()):
        pos = thruster_positions_base[t_id]
        ax = thruster_axes_base[t_id]
        el = thruster_elev_deg[t_id]
        exp = EXPECTED_POSITIONS[t_id]
        diff_m = float(np.linalg.norm(pos - exp))
        print(
            f"{t_id}: pos=({pos[0]:+.4f}, {pos[1]:+.4f}, {pos[2]:+.4f}) m "
            f"[expected diff={diff_m * 100:.1f} cm], "
            f"axis=({ax[0]:+.4f}, {ax[1]:+.4f}, {ax[2]:+.4f}), elev={el:+.2f}°"
        )
        if diff_m > 0.02:
            raise RuntimeError(
                f"STOP: {t_id} position {pos} differs from expected {exp} by {diff_m * 100:.2f} cm (> 2 cm)!"
            )

    # 4. Geometry check against intended design (angles and mirror symmetry)
    geometry_diffs: list[str] = []
    for t_id, el in thruster_elev_deg.items():
        intended = INTENDED_ELEVATION_DEG[t_id]
        d_ang = abs(el - intended)
        if d_ang > 0.5:
            geometry_diffs.append(
                f"{t_id} elevation: exported {el:.2f}°, intended {intended:.2f}° (diff {d_ang:.2f}° > 0.5°)"
            )

    # Check mirror symmetry (thruster 1 & 2, 3 & 4, 5 & 6)
    pairs = [
        ("thruster_1", "thruster_2"),
        ("thruster_3", "thruster_4"),
        ("thruster_5", "thruster_6"),
    ]
    for r_id, l_id in pairs:
        pr = thruster_positions_base[r_id]
        pl = thruster_positions_base[l_id]
        # Y symmetry: pr[1] + pl[1] should be 0
        asym_y = abs(pr[1] + pl[1])
        if asym_y > 0.01:
            geometry_diffs.append(
                f"Pair {r_id}/{l_id} lateral asymmetry: y={pr[1]:.4f} vs {pl[1]:.4f} (diff {asym_y * 100:.1f} cm > 1 cm)"
            )

    geom_check_file = in_dir.parent / "geometry_check.txt"
    with open(geom_check_file, "w") as f:
        f.write("# PS11-AUV Thruster Geometry Verification (§7.1, T1.1)\n")
        f.write(f"Total Hull Mass: {hull_mass:.4f} kg\n")
        f.write(f"Hull Bounding Box Centre (SW frame): {center_sw.tolist()}\n\n")
        f.write("Thruster Positions and Axes in base_link (REP-103):\n")
        for t_id in sorted(thruster_positions_base.keys()):
            pos = thruster_positions_base[t_id]
            ax = thruster_axes_base[t_id]
            el = thruster_elev_deg[t_id]
            f.write(
                f"  {t_id}: pos={pos.tolist()}, axis={ax.tolist()}, elevation={el:.2f} deg\n"
            )
        f.write("\nDifferences from Intended Design (> 0.5 deg or > 1 cm):\n")
        if geometry_diffs:
            f.writelines(f"  - {diff}\n" for diff in geometry_diffs)
        else:
            f.write("  None (all within tolerances)\n")
    print(f"\nWrote geometry check report to {geom_check_file}")

    # 5. Generate ps11_core.urdf.xacro
    out_urdf_dir = out_dir / "urdf"
    out_urdf_dir.mkdir(parents=True, exist_ok=True)
    core_xacro_path = out_urdf_dir / "ps11_core.urdf.xacro"

    # Build clean XML structure for ps11_core
    # Transform from base_link to SW frame origin:
    # Origin of base_link_sw relative to base_link:
    # xyz = [-c_y, c_x, -c_z], rpy = [0, 0, -1.57079632679]
    t_x = -float(center_sw[1])
    t_y = float(center_sw[0])
    t_z = -float(center_sw[2])

    core_lines = [
        '<?xml version="1.0" encoding="utf-8"?>',
        "<!-- PS11 AUV Core Model converted from SolidWorks ROS 2 exporter (§7.1–§7.4, T1.1).",
        "     Generated by tools/convert_sw_export.py. DO NOT EDIT DIRECTLY.",
        "     Thruster mapping:",
        "       Link_FR -> thruster_1 (Joint_FR -> thruster_1_joint)",
        "       Link_FL -> thruster_2 (Joint_FL -> thruster_2_joint)",
        "       Link_MR -> thruster_3 (Joint_MR -> thruster_3_joint)",
        "       Link_ML -> thruster_4 (Joint_ML -> thruster_4_joint)",
        "       Link_BR -> thruster_5 (Joint_BR -> thruster_5_joint)",
        "       Link_BL -> thruster_6 (Joint_BL -> thruster_6_joint)",
        "-->",
        '<robot name="ps11" xmlns:xacro="http://www.ros.org/wiki/xacro">',
        "  <!-- Root link: base_link at hull bounding-box centre (REP-103: X forward, Y left, Z up) -->",
        '  <link name="base_link">',
        "    <!-- PLACEHOLDER — CoM from CAD needed before M2 -->",
        "    <inertial>",
        '      <origin xyz="0 0 0" rpy="0 0 0" />',
        f'      <mass value="{hull_mass:.6f}" />',
        '      <inertia ixx="0.354242" ixy="0.0" ixz="0.0" iyy="0.340674" iyz="0.0" izz="0.087732" />',
        "    </inertial>",
        "    <collision>",
        '      <origin xyz="0 0 0" rpy="0 0 0" />',
        "      <geometry>",
        f'        <box size="{dim_length:.3f} {dim_width:.3f} {dim_height:.3f}" />',
        "      </geometry>",
        "    </collision>",
        "  </link>",
        "",
        "  <!-- Fixed joint attaching the exported SolidWorks assembly to base_link -->",
        '  <joint name="base_to_sw_joint" type="fixed">',
        '    <parent link="base_link" />',
        '    <child link="base_link_sw" />',
        f'    <origin xyz="{t_x:.6f} {t_y:.6f} {t_z:.6f}" rpy="0 0 -1.57079632679" />',
        "  </joint>",
        "",
        "  <!-- Exported hull visual body in SolidWorks coordinate space -->",
        '  <link name="base_link_sw">',
        "    <visual>",
        '      <origin xyz="0 0 0" rpy="0 0 0" />',
        "      <geometry>",
        '        <mesh filename="package://ps11_description/meshes/base_link.STL" />',
        "      </geometry>",
        '      <material name="hull_material">',
        '        <color rgba="0.894 0.894 0.894 1" />',
        "      </material>",
        "    </visual>",
        "  </link>",
        "",
    ]

    # Convert thruster links and joints
    for link in root.findall("./link"):
        lname = link.attrib.get("name", "")
        if lname in THRUSTER_MAP:
            t_name, _, _, _ = THRUSTER_MAP[lname]
            inertial = link.find("inertial")
            visual = link.find("visual")

            core_lines.append(f'  <link name="{t_name}">')
            if inertial is not None:
                orig = inertial.find("origin")
                mass = inertial.find("mass")
                ine = inertial.find("inertia")
                o_xyz = orig.attrib.get("xyz", "0 0 0") if orig is not None else "0 0 0"
                o_rpy = orig.attrib.get("rpy", "0 0 0") if orig is not None else "0 0 0"
                m_val = mass.attrib.get("value", "0.01") if mass is not None else "0.01"
                ixx = ine.attrib.get("ixx", "1e-6") if ine is not None else "1e-6"
                ixy = ine.attrib.get("ixy", "0") if ine is not None else "0"
                ixz = ine.attrib.get("ixz", "0") if ine is not None else "0"
                iyy = ine.attrib.get("iyy", "1e-6") if ine is not None else "1e-6"
                iyz = ine.attrib.get("iyz", "0") if ine is not None else "0"
                izz = ine.attrib.get("izz", "1e-6") if ine is not None else "1e-6"
                core_lines.append("    <inertial>")
                core_lines.append(f'      <origin xyz="{o_xyz}" rpy="{o_rpy}" />')
                core_lines.append(f'      <mass value="{m_val}" />')
                core_lines.append(
                    f'      <inertia ixx="{ixx}" ixy="{ixy}" ixz="{ixz}" iyy="{iyy}" iyz="{iyz}" izz="{izz}" />'
                )
                core_lines.append("    </inertial>")

            if visual is not None:
                core_lines.append("    <visual>")
                core_lines.append('      <origin xyz="0 0 0" rpy="0 0 0" />')
                core_lines.append("      <geometry>")
                core_lines.append(
                    f'        <mesh filename="package://ps11_description/meshes/{lname}.STL" />'
                )
                core_lines.append("      </geometry>")
                core_lines.append('      <material name="propeller_material">')
                core_lines.append('        <color rgba="0.169 0.600 0.812 1" />')
                core_lines.append("      </material>")
                core_lines.append("    </visual>")
            core_lines.append("  </link>")
            core_lines.append("")

    for joint in root.findall("./joint"):
        child = joint.find("child")
        if child is None:
            continue
        cname = child.attrib.get("link", "")
        if cname in THRUSTER_MAP:
            t_name, j_name, _, _ = THRUSTER_MAP[cname]
            orig = joint.find("origin")
            o_xyz = orig.attrib.get("xyz", "0 0 0") if orig is not None else "0 0 0"
            o_rpy = orig.attrib.get("rpy", "0 0 0") if orig is not None else "0 0 0"

            core_lines.append(f'  <joint name="{j_name}" type="continuous">')
            core_lines.append(f'    <origin xyz="{o_xyz}" rpy="{o_rpy}" />')
            core_lines.append('    <parent link="base_link_sw" />')
            core_lines.append(f'    <child link="{t_name}" />')
            core_lines.append('    <axis xyz="0 0 1" />')
            core_lines.append("  </joint>")
            core_lines.append("")

    core_lines.append("</robot>")

    with open(core_xacro_path, "w") as f:
        f.write("\n".join(core_lines) + "\n")
    print(f"Generated {core_xacro_path}")

    # 6. Ensure ps11.urdf.xacro exists
    main_xacro_path = out_urdf_dir / "ps11.urdf.xacro"
    if not main_xacro_path.exists():
        main_xacro_content = """<?xml version="1.0" encoding="utf-8"?>
<robot name="ps11" xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:arg name="mode" default="kinematic" />
  <xacro:arg name="camera_x" default="0.357" />
  <xacro:arg name="camera_y" default="0.0" />
  <xacro:arg name="camera_z" default="0.0" />
  <xacro:arg name="camera_pitch" default="0.785398" />

  <!-- Include Core Model converted from SolidWorks -->
  <xacro:include filename="$(find ps11_description)/urdf/ps11_core.urdf.xacro" />

  <!-- Camera link: front centre of hull, pitched 45° down (design choice, not modelled in CAD) -->
  <link name="camera_link" />
  <joint name="camera_joint" type="fixed">
    <parent link="base_link" />
    <child link="camera_link" />
    <origin xyz="$(arg camera_x) $(arg camera_y) $(arg camera_z)" rpy="0 $(arg camera_pitch) 0" />
  </joint>

  <!-- Standard optical frame (Z forward, X right, Y down) -->
  <link name="camera_optical_frame" />
  <joint name="camera_optical_joint" type="fixed">
    <parent link="camera_link" />
    <child link="camera_optical_frame" />
    <origin xyz="0 0 0" rpy="-1.57079632679 0 -1.57079632679" />
  </joint>

  <!-- Navigation sensor frames at hull centre -->
  <link name="imu_link" />
  <joint name="imu_joint" type="fixed">
    <parent link="base_link" />
    <child link="imu_link" />
    <origin xyz="0 0 0" rpy="0 0 0" />
  </joint>

  <!-- Altimeter link pointing straight down (pitched 90° down) -->
  <link name="altimeter_link" />
  <joint name="altimeter_joint" type="fixed">
    <parent link="base_link" />
    <child link="altimeter_link" />
    <origin xyz="0 0 0" rpy="0 1.57079632679 0" />
  </joint>

  <!-- Depth sensor link -->
  <link name="depth_sensor_link" />
  <joint name="depth_sensor_joint" type="fixed">
    <parent link="base_link" />
    <child link="depth_sensor_link" />
    <origin xyz="0 0 0" rpy="0 0 0" />
  </joint>
</robot>
"""
        with open(main_xacro_path, "w") as f:
            f.write(main_xacro_content)
        print(f"Created {main_xacro_path}")

    # 7. Update ps11_bringup/config/vehicle.yaml
    vehicle_yaml_path = Path("ros2_ws/src/ps11_bringup/config/vehicle.yaml")
    if vehicle_yaml_path.exists():
        vehicle_yaml_content = f"""# Vehicle configuration (§7.1, §7.4, T1.1)
num_thrusters: 6
max_thrust_n: null           # TODO: from CAD export / thruster testing
thruster_datasheet_url: null # TODO: datasheet link
mass_kg: {hull_mass:.2f}
dimensions_m:
  length: {dim_length:.3f}
  width: {dim_width:.3f}
  height: {dim_height:.3f}

# Thruster naming and mapping:
#   thruster_1: Link_FR (front right)
#   thruster_2: Link_FL (front left)
#   thruster_3: Link_MR (middle right)
#   thruster_4: Link_ML (middle left)
#   thruster_5: Link_BR (back right)
#   thruster_6: Link_BL (back left)
#
# Intended thruster design angles:
#   front pair (thruster_1, thruster_2): 25° below horizontal (-25°)
#   middle pair (thruster_3, thruster_4): 45° above horizontal (+45°)
#   back pair (thruster_5, thruster_6): 10° above horizontal (+10°)
#   left/right mirror-symmetric
"""
        with open(vehicle_yaml_path, "w") as f:
            f.write(vehicle_yaml_content)
        print(f"Updated {vehicle_yaml_path}")

    print("==================================================================")
    print("Conversion completed successfully!")
    print(f"Total Mass: {hull_mass:.2f} kg")
    print("==================================================================")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Convert SolidWorks URDF export to ps11_description."
    )
    parser.add_argument(
        "--in",
        dest="in_dir",
        default="tools/sw_export_raw/ps11_vehicle_export",
        help="Input SolidWorks export directory",
    )
    parser.add_argument(
        "--out",
        dest="out_dir",
        default="ros2_ws/src/ps11_description",
        help="Output ps11_description directory",
    )
    args = parser.parse_args()

    convert_sw_export(Path(args.in_dir), Path(args.out_dir))
