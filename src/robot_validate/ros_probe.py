"""ROS-only telemetry and action client. Truth topics are never published as TF/odometry."""

import math
import time
from pathlib import Path
import numpy as np

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from geometry_msgs.msg import Twist, PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry, OccupancyGrid, Path as NavPath
from nav2_msgs.action import NavigateToPose
from nav2_msgs.msg import CollisionMonitorState
from ros_gz_interfaces.msg import Contacts
from sensor_msgs.msg import LaserScan
from std_msgs.msg import UInt32
from lifecycle_msgs.srv import GetState
from lifecycle_msgs.msg import State
from rcl_interfaces.srv import GetParameters
from tf2_ros import Buffer, TransformListener, TransformException

from .data import write_tum
from .model import dump_json
from .slam import Grid
from .route_control import waypoint_command
from .navigation_scenarios import navigation_waypoints, crossed_narrow_corridor, NARROW_CORRIDOR
from .temporary_block import insertion_ready, StopWindow, recovery_evidence


def yaw(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def stamp(message):
    return message.sec + message.nanosec / 1e9


class Probe(Node):
    def __init__(self, output):
        super().__init__(
            "validation_probe",
            parameter_overrides=[rclpy.parameter.Parameter("use_sim_time", value=True)],
        )
        self.output = Path(output)
        self.output.mkdir(parents=True, exist_ok=True)
        self.scan = self.odom = self.truth = self.map = None
        self.contacts = self.contact_samples = 0
        self.contact_times = []
        self.collision_geometries = 0
        self.count_samples = []
        self.scan_times, self.estimates, self.truths, self.events, self.paths = [], [], [], [], []
        self.odometry = []
        self.motion = []
        self.monitor_states = []
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.subscriptions_owned = [
            self.create_subscription(LaserScan, "/scan", self.on_scan, qos_profile_sensor_data),
            self.create_subscription(Odometry, "/odom", self.on_odometry, qos_profile_sensor_data),
            self.create_subscription(
                Odometry, "/ground_truth", self.on_truth, qos_profile_sensor_data
            ),
            self.create_subscription(
                Contacts, "/contacts", self.on_contacts, qos_profile_sensor_data
            ),
            self.create_subscription(
                OccupancyGrid,
                "/map",
                lambda m: setattr(self, "map", m),
                QoSProfile(
                    depth=1,
                    durability=DurabilityPolicy.TRANSIENT_LOCAL,
                    reliability=ReliabilityPolicy.RELIABLE,
                ),
            ),
            self.create_subscription(NavPath, "/plan", self.on_path, 10),
            self.create_subscription(
                CollisionMonitorState, "/collision_monitor_state", self.on_monitor_state, 10
            ),
        ]
        self.cmd = self.create_publisher(Twist, "/cmd_vel", 10)
        self.subscriptions_owned.append(
            self.create_subscription(UInt32, "/contact_monitor_count", self.on_count, 10)
        )
        self.initial_pose = self.create_publisher(PoseWithCovarianceStamped, "/initialpose", 10)
        self.client = ActionClient(self, NavigateToPose, "/navigate_to_pose")
        self.timer = self.create_timer(0.1, self.collect_tf)

    def on_scan(self, msg):
        self.scan = msg
        self.scan_times.append(stamp(msg.header.stamp))

    def on_truth(self, msg):
        self.truth = msg
        p = msg.pose.pose
        self.truths.append(
            [stamp(msg.header.stamp), p.position.x, p.position.y, yaw(p.orientation)]
        )
        v = msg.twist.twist
        self.motion.append(
            [
                stamp(msg.header.stamp),
                p.position.x,
                p.position.y,
                yaw(p.orientation),
                v.linear.x,
                v.linear.y,
                v.angular.z,
            ]
        )

    def on_odometry(self, msg):
        self.odom = msg
        p = msg.pose.pose
        self.odometry.append(
            [stamp(msg.header.stamp), p.position.x, p.position.y, yaw(p.orientation)]
        )

    def on_count(self, msg):
        self.collision_geometries = msg.data
        self.count_samples.append((self.now_s(), msg.data))

    def on_monitor_state(self, msg):
        self.monitor_states.append(
            dict(time_s=self.now_s(), action_type=msg.action_type, polygon=msg.polygon_name)
        )

    def on_contacts(self, msg):
        self.contact_samples += 1
        self.contact_times.append(stamp(msg.header.stamp))
        for contact in msg.contacts:
            a, b = contact.collision1.name, contact.collision2.name
            if "floor" in a or "floor" in b:
                continue
            # The native observer supplies only robot-associated contacts, including ID-only records.
            self.contacts += 1
            self.events.append(dict(kind="collision", time_s=self.now_s(), objects=[a, b]))

    def on_path(self, msg):
        self.paths.append(
            dict(
                time_s=stamp(msg.header.stamp),
                frame=msg.header.frame_id,
                points=[[p.pose.position.x, p.pose.position.y] for p in msg.poses],
            )
        )

    def collect_tf(self):
        try:
            tf = self.buffer.lookup_transform("map", "base_link", Time())
        except TransformException:
            return
        ts = stamp(tf.header.stamp)
        if self.estimates and ts <= self.estimates[-1][0]:
            return
        p, q = tf.transform.translation, tf.transform.rotation
        self.estimates.append([ts, p.x, p.y, yaw(q)])

    def now_s(self):
        return self.get_clock().now().nanoseconds / 1e9

    def wait(self, condition, timeout, description):
        deadline = time.monotonic() + timeout
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            if condition():
                return
        raise TimeoutError(f"Timed out waiting for {description}")

    def run_for(self, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.05)

    def ready(self, mapping=False):
        self.wait(
            lambda: self.now_s() > 0 and self.scan is not None and self.odom is not None,
            60,
            "simulation clock, scan, odometry",
        )
        if mapping:
            self.wait(
                lambda: self.map is not None and len(self.estimates) >= 3,
                60,
                "SLAM map and map→base_link TF",
            )

    def velocity(self, v=0.0, w=0.0):
        msg = Twist()
        msg.linear.x, msg.angular.z = float(v), float(w)
        self.cmd.publish(msg)

    def drive_mapping_route(self):
        # Map-frame feedback corrects wheel slip; ground truth remains evaluation-only.
        for x, y in [(8, 0), (8, 5), (0, 5), (0, 0)]:
            self.events.append(
                dict(
                    kind="mapping_waypoint",
                    time_s=self.now_s(),
                    goal=[x, y],
                    feedback="slam_map_tf",
                )
            )
            deadline = time.monotonic() + 100
            while time.monotonic() < deadline:
                rclpy.spin_once(self, timeout_sec=0.05)
                pose = self.estimates[-1] if self.estimates else None
                v, w, reached = waypoint_command(pose, (x, y), self.now_s())
                self.velocity(v, w)
                if reached:
                    break
                if self.contacts:
                    raise RuntimeError("Collision during fixed mapping route")
            else:
                raise TimeoutError("Mapping route waypoint timeout")
        self.velocity()
        self.run_for(3)

    def grid(self):
        info = self.map.info
        origin = info.origin
        return Grid(
            np.asarray(self.map.data, np.int8).reshape(info.height, info.width),
            info.resolution,
            (origin.position.x, origin.position.y, yaw(origin.orientation)),
        )

    def export(self):
        if self.estimates:
            write_tum(self.output / "estimated.tum", np.asarray(self.estimates))
        if self.truths:
            write_tum(self.output / "ground-truth-raw.tum", np.asarray(self.truths))
        if self.odometry:
            write_tum(self.output / "wheel-odometry.tum", np.asarray(self.odometry))
        dump_json(self.output / "events.json", self.events)
        dump_json(self.output / "paths.json", self.paths)
        dump_json(self.output / "scan-times.json", self.scan_times)
        dump_json(self.output / "collision-monitor.json", self.monitor_states)
        dump_json(
            self.output / "motion.json",
            dict(columns=["time_s", "x", "y", "yaw", "vx", "vy", "wz"], samples=self.motion),
        )
        dump_json(
            self.output / "contact-telemetry.json",
            dict(contact_times_s=self.contact_times, geometry_count_samples=self.count_samples),
        )

    def initialize_localization(self):
        message = PoseWithCovarianceStamped()
        message.header.frame_id = "map"
        message.pose.pose.orientation.w = 1.0
        message.pose.covariance[0] = message.pose.covariance[7] = 0.01
        message.pose.covariance[35] = 0.01
        for _ in range(5):
            message.header.stamp = self.get_clock().now().to_msg()
            self.initial_pose.publish(message)
            self.run_for(0.3)
        self.wait(
            lambda: len(self.estimates) > 3 and self.client.server_is_ready(),
            60,
            "AMCL TF and navigation action server",
        )
        # An action endpoint exists while its lifecycle node is still inactive.
        # Wait for every motion-producing part of this pipeline before sending a goal.
        readiness = {}
        deadline = time.monotonic() + 60
        for name in (
            "amcl",
            "controller_server",
            "planner_server",
            "bt_navigator",
            "velocity_smoother",
            "collision_monitor",
        ):
            client = self.create_client(GetState, f"/{name}/get_state")
            try:
                self.wait(
                    client.service_is_ready,
                    max(0.1, deadline - time.monotonic()),
                    f"{name} lifecycle service",
                )
                while time.monotonic() < deadline:
                    future = client.call_async(GetState.Request())
                    self.wait(
                        future.done,
                        min(5, max(0.1, deadline - time.monotonic())),
                        f"{name} lifecycle response",
                    )
                    state = future.result().current_state
                    if state.id == State.PRIMARY_STATE_ACTIVE:
                        readiness[name] = dict(state=state.label, observed_at_s=self.now_s())
                        break
                    self.run_for(0.1)
                else:
                    raise TimeoutError(f"{name} did not become active")
            finally:
                self.destroy_client(client)
        dump_json(self.output / "readiness.json", readiness)
        parameter_client = self.create_client(GetParameters, "/controller_server/get_parameters")
        try:
            self.wait(parameter_client.service_is_ready, 10, "controller parameter service")
            request = GetParameters.Request()
            request.names = [
                "FollowPath.plugin",
                "FollowPath.primary_controller",
                "FollowPath.rotate_to_goal_heading",
                "FollowPath.rotate_to_heading_once",
                "general_goal_checker.xy_goal_tolerance",
                "general_goal_checker.yaw_goal_tolerance",
            ]
            future = parameter_client.call_async(request)
            self.wait(future.done, 10, "active controller parameters")
            actual = {}
            for name, value in zip(request.names, future.result().values):
                fields = {1: "bool_value", 2: "integer_value", 3: "double_value", 4: "string_value"}
                actual[name] = getattr(value, fields[value.type]) if value.type in fields else None
            if any(value is None for value in actual.values()):
                raise RuntimeError("Active controller is missing required rotation parameters")
            dump_json(self.output / "controller-configuration.json", actual)
        finally:
            self.destroy_client(parameter_client)
        # Lifecycle activation precedes completion of DDS endpoint discovery.
        # Keep spinning through that discovery interval before any motion request.
        self.run_for(1.5)
        self.wait(
            lambda: len(self.contact_times) >= 3
            and len(self.count_samples) >= 3
            and self.collision_geometries == 4
            and self.now_s() - self.contact_times[-1] <= 0.5
            and self.now_s() - self.count_samples[-1][0] <= 0.5,
            15,
            "fresh contact telemetry before navigation",
        )

    def navigate(self, scenario, map_hash, set_block):
        goals = {
            "healthy": [8, 0, 0],
            "temporary-block": [8, 0, 0],
            "unreachable": [3, 2, 0],
            "narrow": [7.2, 5.2, 1.57],
        }
        goal = goals[scenario]
        waypoints = navigation_waypoints(scenario, goal)
        pending_waypoints = iter(waypoints)

        def feedback(msg):
            f = msg.feedback
            if f.number_of_recoveries:
                self.events.append(
                    dict(kind="recovery", time_s=self.now_s(), count=f.number_of_recoveries)
                )

        begin, wall_begin = self.now_s(), time.monotonic()

        def send_goal(point):
            target = PoseStamped()
            target.header.frame_id = "map"
            target.header.stamp = self.get_clock().now().to_msg()
            target.pose.position.x, target.pose.position.y = float(point[0]), float(point[1])
            target.pose.orientation.z = math.sin(point[2] / 2)
            target.pose.orientation.w = math.cos(point[2] / 2)
            message = NavigateToPose.Goal()
            message.pose = target
            self.events.append(dict(kind="navigation_waypoint", time_s=self.now_s(), goal=point))
            request_start = time.monotonic()
            future = self.client.send_goal_async(message, feedback_callback=feedback)
            request_elapsed = time.monotonic() - request_start
            self.wait(future.done, 15, "goal acceptance")
            handle = future.result()
            if not handle.accepted:
                raise RuntimeError("Nav2 rejected goal before execution")
            result_start = time.monotonic()
            completion = handle.get_result_async()
            self.events.append(
                dict(
                    kind="action_request_timing",
                    time_s=self.now_s(),
                    goal_request_wall_s=request_elapsed,
                    result_request_wall_s=time.monotonic() - result_start,
                )
            )
            return handle, completion

        before_contacts, before_samples = self.contacts, self.contact_samples
        handle, completion = send_goal(next(pending_waypoints))
        timeout = 60 if scenario == "unreachable" else 120
        inserted = removed = False
        inserted_s = removal_requested_s = removed_s = None
        release_window = StopWindow(3.0)
        minimum_clearance = float("inf")
        while self.now_s() - begin < timeout and time.monotonic() - wall_begin < timeout * 3:
            if completion.done():
                point = next(pending_waypoints, None)
                if completion.result().status != 4 or point is None:
                    break
                handle, completion = send_goal(point)
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.scan is not None:
                ranges = np.asarray(self.scan.ranges)
                valid = ranges[np.isfinite(ranges) & (ranges >= self.scan.range_min)]
                if len(valid):
                    minimum_clearance = min(minimum_clearance, float(valid.min()) - 0.26)
            if scenario == "temporary-block":
                pose = self.estimates[-1] if self.estimates else None
                if not inserted and insertion_ready(pose, self.now_s()):
                    requested_s = self.now_s()
                    set_block(True)
                    self.run_for(0.1)
                    inserted = True
                    inserted_s = self.now_s()
                    self.events.append(
                        dict(
                            kind="block_inserted",
                            time_s=inserted_s,
                            request_time_s=requested_s,
                            feedback="amcl_map_tf",
                        )
                    )
                if inserted and not removed and self.odom is not None:
                    # Fault scheduling uses algorithm pose and wheel odometry, never truth.
                    velocity = self.odom.twist.twist.linear
                    stopped = False
                    front_hold = bool(
                        self.monitor_states
                        and self.monitor_states[-1]["action_type"] == CollisionMonitorState.LIMIT
                        and self.monitor_states[-1]["polygon"] == "FrontHold"
                    )
                    if front_hold and pose is not None and 0 <= self.now_s() - pose[0] <= 0.5:
                        stopped = release_window.update(
                            stamp(self.odom.header.stamp),
                            pose[1],
                            pose[2],
                            math.hypot(velocity.x, velocity.y),
                            self.now_s(),
                        )
                    else:
                        release_window.start = release_window.end = None
                    if stopped:
                        removal_requested_s = self.now_s()
                        set_block(False)
                        self.run_for(0.1)
                        removed_s = self.now_s()
                        removed = True
                        self.events.append(
                            dict(
                                kind="block_removed",
                                time_s=removed_s,
                                request_time_s=removal_requested_s,
                                odometry_stop_interval_s=[release_window.start, release_window.end],
                            )
                        )
        timed_out = (
            not completion.done()
            or self.now_s() - begin >= timeout
            or time.monotonic() - wall_begin >= timeout * 3
        )
        if not completion.done():
            canceled = handle.cancel_goal_async()
            self.wait(canceled.done, 10, "goal cancellation")
            self.wait(completion.done, 10, "canceled action result")
        status = {4: "succeeded", 5: "canceled", 6: "aborted"}.get(
            completion.result().status, "unknown"
        )
        if timed_out:
            status = "timeout"
        # Observe the controller's stop before sending any test-harness stop command.
        self.run_for(1)
        actual = self.truth.pose.pose if self.truth is not None else None
        end = self.now_s()
        samples = np.asarray([t for t in self.contact_times if begin <= t <= end])
        coverage = (
            (float(samples[-1] - samples[0]) / max(0.001, end - begin)) if len(samples) > 1 else 0.0
        )
        max_gap = float(np.max(np.diff(np.r_[begin, samples, end])))
        counts = [(t, count) for t, count in self.count_samples if begin <= t <= end]
        count_times = np.asarray([t for t, _ in counts])
        count_coverage = (
            (float(count_times[-1] - count_times[0]) / max(0.001, end - begin))
            if len(count_times) > 1
            else 0.0
        )
        coverage = min(coverage, count_coverage)
        max_gap = max(max_gap, float(np.max(np.diff(np.r_[begin, count_times, end]))))
        speed = self.truth.twist.twist.linear if self.truth is not None else None
        proof = recovery_evidence(self.motion, inserted_s, removal_requested_s, removed_s)
        if proof["observed"]:
            self.events.append(
                dict(
                    kind="block_observed",
                    time_s=proof["stop_interval_s"][0],
                    end_time_s=proof["stop_interval_s"][1],
                    source="independent_truth",
                )
            )
        if proof["resumed"]:
            self.events.append(
                dict(kind="block_resumed", time_s=proof["resumed_at_s"], source="independent_truth")
            )
        self.events.sort(key=lambda event: event["time_s"])
        record = dict(
            name=scenario,
            expected="controlled_failure" if scenario == "unreachable" else "succeeded",
            action_status=status,
            final_pose=[actual.position.x, actual.position.y, yaw(actual.orientation)]
            if actual
            else None,
            goal=goal,
            contacts=self.contacts - before_contacts,
            contact_samples=self.contact_samples - before_samples,
            contact_coverage=coverage,
            contact_max_gap_s=max_gap,
            contact_age_s=float(end - samples[-1]) if len(samples) else None,
            collision_geometries_observed=min((count for _, count in counts), default=0),
            duration_s=self.now_s() - begin,
            timeout_s=timeout,
            final_speed=math.hypot(speed.x, speed.y) if speed else None,
            pose_age_s=self.now_s() - stamp(self.truth.header.stamp) if self.truth else None,
            map_sha256=map_hash,
            temporary_block=scenario == "temporary-block",
            block_observed=proof["observed"],
            block_removed=removed,
            resumed_after_removal=proof["resumed"],
            min_lidar_clearance_m=None
            if not math.isfinite(minimum_clearance)
            else minimum_clearance,
            window_s=[begin, self.now_s()],
            evidence="navigation.json",
        )
        record["waypoints"] = waypoints
        if scenario == "temporary-block":
            record["block_episode"] = dict(
                inserted_s=inserted_s,
                removal_requested_s=removal_requested_s,
                removed_s=removed_s,
                **proof,
                scheduling_source="AMCL map TF + wheel odometry",
                evaluation_source="independent Gazebo ground truth",
            )
        if scenario == "narrow":
            record["narrow_corridor"] = NARROW_CORRIDOR
            record["corridor_crossed"] = crossed_narrow_corridor(self.truths, begin, end)
        dump_json(self.output / "navigation.json", record)
        self.velocity()
        self.export()
        return record
