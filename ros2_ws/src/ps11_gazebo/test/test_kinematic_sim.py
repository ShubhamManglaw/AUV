"""Unit and integration tests for ps11_gazebo kinematic simulation setup (§7.4, §8.1, §8.4, T1.2)."""

import xml.etree.ElementTree as ET
from pathlib import Path

import yaml


def test_ocean_demo_kinematic_sdf():
    """Verify ocean_demo_kinematic.sdf conforms to §8.1 specifications."""
    world_path = Path("ros2_ws/src/ps11_gazebo/worlds/ocean_demo_kinematic.sdf")
    assert world_path.exists(), f"World file not found: {world_path}"

    tree = ET.parse(world_path)
    root = tree.getroot()

    world = root.find("world")
    assert world is not None, "Missing <world> element"
    assert world.attrib.get("name") == "ocean_demo_kinematic"

    # Gravity must be 0 0 0 for kinematic mode
    gravity = world.find("gravity")
    assert gravity is not None, "Missing <gravity> tag"
    assert gravity.text.strip() == "0 0 0", (
        f"Expected gravity '0 0 0', got '{gravity.text.strip()}'"
    )

    # Physics step must be 4 ms (0.004 s)
    step = world.find("./physics/max_step_size")
    assert step is not None, "Missing <max_step_size>"
    assert float(step.text.strip()) == 0.004

    # Required plugins: Physics, Sensors (ogre2), Imu
    plugins = {p.attrib.get("filename"): p for p in world.findall("plugin")}
    assert "gz-sim-physics-system" in plugins, "Missing physics system plugin"
    assert "gz-sim-sensors-system" in plugins, "Missing sensors system plugin"
    assert "gz-sim-imu-system" in plugins, "Missing imu system plugin"

    sensors_plug = plugins["gz-sim-sensors-system"]
    render_engine = sensors_plug.find("render_engine")
    assert render_engine is not None and render_engine.text.strip() == "ogre2", (
        "Sensors render engine must be ogre2"
    )

    # Seabed plane at z = -15 m
    seabed = world.find("./model[@name='seabed_placeholder']")
    assert seabed is not None, "Missing seabed model"
    pose = seabed.find("pose")
    assert pose is not None, "Missing seabed pose"
    parts = pose.text.strip().split()
    assert float(parts[2]) == -15.0, f"Expected seabed z=-15.0, got {parts[2]}"


def test_bridge_yaml_config():
    """Verify bridge.yaml maps all required topics (§8.3, §8.4)."""
    bridge_path = Path("ros2_ws/src/ps11_gazebo/config/bridge.yaml")
    assert bridge_path.exists(), f"Bridge config not found: {bridge_path}"

    with open(bridge_path, "r") as f:
        bridges = yaml.safe_load(f)

    assert isinstance(bridges, list), "bridge.yaml must be a list of topic mappings"
    ros_topics = {b["ros_topic_name"]: b for b in bridges}

    expected_topics = [
        "/clock",
        "/vehicle/cmd_vel",
        "/vehicle/camera/image_raw",
        "/vehicle/camera/depth",
        "/vehicle/camera/camera_info",
        "/vehicle/imu",
        "/vehicle/altimeter/scan",
        "/sim/gt/odom",
    ]

    for topic in expected_topics:
        assert topic in ros_topics, f"Missing bridge mapping for {topic}"

    # Verify directions
    assert ros_topics["/vehicle/cmd_vel"]["direction"] == "ROS_TO_GZ"
    assert ros_topics["/clock"]["direction"] == "GZ_TO_ROS"
    assert ros_topics["/sim/gt/odom"]["direction"] == "GZ_TO_ROS"
    assert ros_topics["/vehicle/camera/image_raw"]["direction"] == "GZ_TO_ROS"


def test_urdf_kinematic_mode():
    """Verify ps11.urdf.xacro has all required sensors and kinematic plugins (§7.4)."""
    import subprocess

    cmd = [
        "xacro",
        "ros2_ws/src/ps11_description/urdf/ps11.urdf.xacro",
        "mode:=kinematic",
    ]
    res = subprocess.run(cmd, capture_output=True, text=True, check=True)
    urdf_xml = res.stdout

    root = ET.fromstring(urdf_xml)

    # Check frames exist
    link_names = {l.attrib["name"] for l in root.findall("link")}
    required_links = {
        "base_link",
        "camera_link",
        "camera_optical_frame",
        "imu_link",
        "altimeter_link",
        "depth_sensor_link",
        "thruster_1",
        "thruster_2",
        "thruster_3",
        "thruster_4",
        "thruster_5",
        "thruster_6",
    }
    for req in required_links:
        assert req in link_names, f"Missing link '{req}' in URDF"

    # Check sensors exist in gazebo tags
    sensors = {s.attrib.get("name"): s for s in root.findall(".//sensor")}
    assert "rgbd_camera" in sensors, "Missing rgbd_camera sensor"
    assert "imu_sensor" in sensors, "Missing imu_sensor"
    assert "altimeter" in sensors, "Missing altimeter sensor"

    rgbd = sensors["rgbd_camera"]
    assert rgbd.attrib.get("type") == "rgbd_camera"
    assert float(rgbd.find("update_rate").text) == 10.0

    imu = sensors["imu_sensor"]
    assert imu.attrib.get("type") == "imu"
    assert float(imu.find("update_rate").text) == 100.0

    altimeter = sensors["altimeter"]
    assert altimeter.attrib.get("type") == "gpu_lidar"
    assert float(altimeter.find("update_rate").text) == 10.0

    # Check kinematic plugins: OdometryPublisher and VelocityControl
    plugins = {p.attrib.get("filename"): p for p in root.findall(".//plugin")}
    assert "gz-sim-odometry-publisher-system" in plugins, "Missing OdometryPublisher"
    assert "gz-sim-velocity-control-system" in plugins, "Missing VelocityControl"
