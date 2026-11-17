"""Generate an offline, primitive-geometry Gazebo world; no Fuel downloads."""

from pathlib import Path
import xml.etree.ElementTree as ET
import yaml

from .data import WALLS
from .temporary_block import GATE_X, GATE_Y, GATE_SIZE


def build_world(path):
    sdf = ET.Element("sdf", version="1.9")
    world = ET.SubElement(sdf, "world", name="validation")
    physics = ET.SubElement(world, "physics", name="fixed", type="ignored")
    ET.SubElement(physics, "max_step_size").text = "0.001"
    ET.SubElement(physics, "real_time_factor").text = "1.0"
    for library, cls in [
        ("physics", "Physics"),
        ("user-commands", "UserCommands"),
        ("scene-broadcaster", "SceneBroadcaster"),
        ("contact", "Contact"),
        ("sensors", "Sensors"),
    ]:
        plugin = ET.SubElement(
            world, "plugin", filename=f"gz-sim-{library}-system", name=f"gz::sim::systems::{cls}"
        )
        if cls == "Sensors":
            ET.SubElement(plugin, "render_engine").text = "ogre2"
    light = ET.SubElement(world, "light", type="directional", name="sun")
    ET.SubElement(light, "pose").text = "0 0 10 0 0 0"
    ET.SubElement(light, "direction").text = "-0.5 0.1 -0.9"
    ET.SubElement(light, "diffuse").text = "0.8 0.8 0.8 1"

    def box(name, pose, size):
        model = ET.SubElement(world, "model", name=name)
        ET.SubElement(model, "static").text = "true"
        ET.SubElement(model, "pose").text = pose
        link = ET.SubElement(model, "link", name="body")
        for kind in ("collision", "visual"):
            element = ET.SubElement(link, kind, name=kind)
            geometry = ET.SubElement(element, "geometry")
            ET.SubElement(ET.SubElement(geometry, "box"), "size").text = size
        return model

    box("floor", "4 2.5 -0.1 0 0 0", "16 13 0.2")
    for i, (axis, at, lo, hi) in enumerate(WALLS):
        x, y = (at, (lo + hi) / 2) if axis == 0 else ((lo + hi) / 2, at)
        sx, sy = (0.1, hi - lo) if axis == 0 else (hi - lo, 0.1)
        box(f"wall_{i}", f"{x - 2} {y - 2} 0.75 0 0 0", f"{sx} {sy} 1.5")
    box(
        "temporary_block",
        f"{GATE_X} {GATE_Y} -3 0 0 0",
        " ".join(str(v) for v in GATE_SIZE),
    )
    robot = ET.fromstring("""<model name="robot"><pose>0 0 0.12 0 0 0</pose>
<link name="base_link"><inertial><mass>5</mass><inertia><ixx>0.05</ixx><iyy>0.05</iyy><izz>0.08</izz></inertia></inertial>
<collision name="body"><geometry><box><size>0.38 0.32 0.16</size></box></geometry></collision>
<visual name="body"><geometry><box><size>0.38 0.32 0.16</size></box></geometry></visual>
<sensor name="contact" type="contact"><always_on>true</always_on><update_rate>50</update_rate><topic>/contacts</topic><contact><collision>body</collision></contact></sensor>
<sensor name="laser" type="gpu_lidar"><pose>0 0 0.16 0 0 0</pose><always_on>true</always_on><update_rate>10</update_rate><topic>/scan</topic><gz_frame_id>laser</gz_frame_id>
<lidar><scan><horizontal><samples>361</samples><resolution>1</resolution><min_angle>-2.09439510239</min_angle><max_angle>2.09439510239</max_angle></horizontal></scan><range><min>0.05</min><max>12</max><resolution>0.01</resolution></range></lidar></sensor></link>
<link name="caster"><pose>-0.15 0 -0.08 0 0 0</pose><inertial><mass>0.1</mass><inertia><ixx>0.001</ixx><iyy>0.001</iyy><izz>0.001</izz></inertia></inertial><collision name="caster"><geometry><sphere><radius>0.04</radius></sphere></geometry><surface><friction><ode><mu>0</mu><mu2>0</mu2></ode></friction></surface></collision></link>
<joint name="caster_joint" type="fixed"><parent>base_link</parent><child>caster</child></joint>
<plugin filename="gz-sim-diff-drive-system" name="gz::sim::systems::DiffDrive"><left_joint>left_joint</left_joint><right_joint>right_joint</right_joint><wheel_separation>0.38</wheel_separation><wheel_radius>0.10</wheel_radius><topic>/cmd_vel</topic><odom_topic>/odom</odom_topic><tf_topic>/tf</tf_topic><frame_id>odom</frame_id><child_frame_id>base_link</child_frame_id><odom_publish_frequency>50</odom_publish_frequency></plugin>
<plugin filename="gz-sim-odometry-publisher-system" name="gz::sim::systems::OdometryPublisher"><odom_frame>world</odom_frame><robot_base_frame>base_link</robot_base_frame><odom_topic>/ground_truth</odom_topic><tf_topic>/ground_truth_tf</tf_topic><dimensions>3</dimensions><odom_publish_frequency>50</odom_publish_frequency></plugin>
</model>""")
    for name, y in (("left", 0.19), ("right", -0.19)):
        robot.append(
            ET.fromstring(
                f'''<link name="{name}_wheel"><pose>0 {y} -0.02 1.57079632679 0 0</pose><inertial><mass>0.5</mass><inertia><ixx>0.002</ixx><iyy>0.002</iyy><izz>0.002</izz></inertia></inertial><collision name="wheel"><geometry><cylinder><radius>0.1</radius><length>0.04</length></cylinder></geometry></collision><visual name="wheel"><geometry><cylinder><radius>0.1</radius><length>0.04</length></cylinder></geometry></visual></link>'''
            )
        )
        robot.append(
            ET.fromstring(
                f'''<joint name="{name}_joint" type="revolute"><parent>base_link</parent><child>{name}_wheel</child><axis><xyz expressed_in="__model__">0 1 0</xyz></axis></joint>'''
            )
        )
    # A native observer emits empty contact frames too; the stock sensor is event-only.
    for sensor in list(robot.findall(".//sensor[@type='contact']")):
        for link in robot.findall("link"):
            if sensor in list(link):
                link.remove(sensor)
    ET.SubElement(
        world,
        "plugin",
        filename="robot-validation-contacts-system",
        name="robot_validate::ContactsMonitor",
    )
    world.append(robot)
    ET.indent(sdf)
    ET.ElementTree(sdf).write(path, encoding="utf-8", xml_declaration=True)


def bridge_config(path):
    entries = []
    for topic, ros_type, gz_type, direction in [
        ("/clock", "rosgraph_msgs/msg/Clock", "Clock", "GZ_TO_ROS"),
        ("/scan", "sensor_msgs/msg/LaserScan", "LaserScan", "GZ_TO_ROS"),
        ("/odom", "nav_msgs/msg/Odometry", "Odometry", "GZ_TO_ROS"),
        ("/tf", "tf2_msgs/msg/TFMessage", "Pose_V", "GZ_TO_ROS"),
        ("/ground_truth", "nav_msgs/msg/Odometry", "Odometry", "GZ_TO_ROS"),
        ("/contacts", "ros_gz_interfaces/msg/Contacts", "Contacts", "GZ_TO_ROS"),
        ("/cmd_vel", "geometry_msgs/msg/Twist", "Twist", "ROS_TO_GZ"),
    ]:
        entries.append(
            dict(
                topic_name=topic,
                ros_type_name=ros_type,
                gz_type_name=f"gz.msgs.{gz_type}",
                direction=direction,
            )
        )
    entries.append(
        dict(
            topic_name="/contact_monitor_count",
            ros_type_name="std_msgs/msg/UInt32",
            gz_type_name="gz.msgs.UInt32",
            direction="GZ_TO_ROS",
        )
    )
    Path(path).write_text(yaml.safe_dump(entries))


def nav_config(source, destination):
    config = yaml.safe_load(Path(source).read_text())

    def walk(value):
        if not isinstance(value, dict):
            return
        for key, child in list(value.items()):
            if isinstance(child, dict):
                walk(child)
            if key in ("base_frame_id", "robot_base_frame", "base_frame"):
                value[key] = "base_link"
            if key == "ros__parameters":
                child["use_sim_time"] = True
                child["enable_stamped_cmd_vel"] = False

    walk(config)
    # Action-request acknowledgement, in milliseconds (Nav2 BtActionNode).
    # The upstream 20 ms default is too short for software rendering in the VM.
    # This does not change the 60/120 s scenario acceptance deadlines.
    config["bt_navigator"]["ros__parameters"]["default_server_timeout"] = 200
    # The approach policy slows predicted collisions but still permits immediate
    # reverse/reroute motion. Hold translation at a sensed gate while preserving
    # safe pure rotation. RotationShim explicitly produces zero-linear commands
    # for goal alignment, avoiding curvature-coupled LIMIT zeroing both axes.
    controller = config["controller_server"]["ros__parameters"]
    follow = controller["FollowPath"]
    follow["primary_controller"] = follow["plugin"]
    follow["plugin"] = "nav2_rotation_shim_controller::RotationShimController"
    follow["rotate_to_goal_heading"] = True
    follow["rotate_to_heading_once"] = True
    follow["rotate_to_heading_angular_vel"] = 0.6
    controller["general_goal_checker"]["yaw_goal_tolerance"] = 0.15
    monitor = config["collision_monitor"]["ros__parameters"]
    monitor["polygons"] = ["FrontHold", *monitor["polygons"]]
    monitor["FrontHold"] = dict(
        type="polygon",
        points="[[0.55, 0.16], [0.55, -0.16], [0.0, -0.16], [0.0, 0.16]]",
        action_type="limit",
        linear_limit=0.0,
        angular_limit=0.2,
        min_points=4,
        visualize=False,
        enabled=True,
    )
    config["planner_server"]["ros__parameters"]["GridBased"]["allow_unknown"] = False
    config["planner_server"]["ros__parameters"]["GridBased"]["tolerance"] = 0.1
    for name in ("local_costmap", "global_costmap"):
        params = config[name][name]["ros__parameters"]
        params["robot_radius"] = 0.26
    Path(destination).write_text(yaml.safe_dump(config))


def slam_config(path, loop=True):
    params = dict(
        use_sim_time=True,
        solver_plugin="solver_plugins::CeresSolver",
        ceres_linear_solver="SPARSE_NORMAL_CHOLESKY",
        ceres_preconditioner="SCHUR_JACOBI",
        ceres_trust_strategy="LEVENBERG_MARQUARDT",
        ceres_dogleg_type="TRADITIONAL_DOGLEG",
        ceres_loss_function="None",
        odom_frame="odom",
        map_frame="map",
        base_frame="base_link",
        scan_topic="/scan",
        mode="mapping",
        resolution=0.04,
        max_laser_range=12.0,
        minimum_travel_distance=0.05,
        minimum_travel_heading=0.05,
        transform_publish_period=0.02,
        map_update_interval=1.0,
        throttle_scans=1,
        do_loop_closing=loop,
        loop_search_maximum_distance=3.0,
        loop_match_minimum_chain_size=10,
        use_scan_matching=True,
        use_scan_barycenter=True,
    )
    Path(path).write_text(yaml.safe_dump({"slam_toolbox": {"ros__parameters": params}}))
