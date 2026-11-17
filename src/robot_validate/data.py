"""Raw sensors only; reference poses are returned separately to the evaluator."""

from dataclasses import dataclass
from pathlib import Path

import numpy as np
from rosbags.highlevel import AnyReader
from rosbags.rosbag1 import Writer as Writer1
from rosbags.rosbag2 import Writer as Writer2
from rosbags.typesys import Stores, get_typestore


@dataclass
class Scan:
    stamp: float
    ranges: np.ndarray
    distance: float
    yaw: float
    dt: float
    beam_dt_s: float = 0.0


@dataclass
class LaserSpec:
    size: int = 361
    rate: float = 10.0
    fov: float = 240.0
    max_range: float = 12.0
    margin: int = 8
    offset: float = 0.0


def public_frames(path):
    from breezyslam.vehicles import WheeledVehicle

    class Rover(WheeledVehicle):
        def extractOdometry(self, stamp, left, right):
            return stamp / 1e6, left * 180 / 2000, right * 180 / 2000

    rover = Rover(77, 165)
    frames = []
    for line in Path(path).read_text().splitlines():
        cells = line.split()
        distance, yaw, dt = rover.computePoseChange(int(cells[0]), int(cells[2]), int(cells[3]))
        frames.append(
            Scan(
                int(cells[0]) / 1e6,
                np.asarray(cells[24:-1], float) / 1000,
                distance / 1000,
                np.deg2rad(yaw),
                dt,
            )
        )
    return frames, LaserSpec(682, 10, 240, 4, 70, 0.145)


BOXES = [(0, 0, 12, 9), (3.75, 3.4, 7.1, 5.2), (8, 4, 8.7, 5.65)]
WALLS = [(0, 1.3, 0.0, 1.25), (1, 7.9, 7.7, 9.5)]
for x0, y0, x1, y1 in BOXES:
    WALLS.extend([(0, x0, y0, y1), (0, x1, y0, y1), (1, y0, x0, x1), (1, y1, x0, x1)])


def raycast(pose, spec=LaserSpec()):
    beam = np.linspace(-np.deg2rad(spec.fov) / 2, np.deg2rad(spec.fov) / 2, spec.size)
    direction = np.array([np.cos(pose[2] + beam), np.sin(pose[2] + beam)])
    ranges = np.full(spec.size, spec.max_range)
    for axis, wall, lower, upper in WALLS:
        other = 1 - axis
        candidate = np.divide(
            wall - pose[axis],
            direction[axis],
            out=np.zeros(spec.size),
            where=np.abs(direction[axis]) > 1e-9,
        )
        intersection = pose[other] + candidate * direction[other]
        valid = (candidate > 0) & (intersection >= lower) & (intersection <= upper)
        ranges[valid] = np.minimum(ranges[valid], candidate[valid])
    return ranges


def synthetic_frames(seed=391, wheel_scale=1.0):
    poses = [np.array([2.0, 2.0, 0.0])]
    for distance in [8, 5, 8, 5]:
        for _ in range(round(distance / 0.1)):
            pose = poses[-1].copy()
            pose[:2] += 0.1 * np.array([np.cos(pose[2]), np.sin(pose[2])])
            poses.append(pose)
        for _ in range(15):
            pose = poses[-1].copy()
            pose[2] += np.pi / 30
            poses.append(pose)
    rng = np.random.default_rng(seed)
    frames = []
    for i, pose in enumerate(poses):
        ranges = raycast(pose) + rng.normal(0, 0.008, 361)
        delta = pose - poses[max(0, i - 1)]
        distance = np.linalg.norm(delta[:2]) * (1.04 + rng.normal(0, 0.01)) * wheel_scale
        yaw = delta[2] + np.deg2rad(rng.normal(0.04, 0.04) if i else 0)
        frames.append(Scan(1 + i * 0.1, ranges, distance, yaw, 0.1))
    return frames, LaserSpec(), np.column_stack(([f.stamp for f in frames], poses))


def write_bag(path, frames, spec, missing=(), ros2=False):
    store = get_typestore(Stores.ROS2_HUMBLE if ros2 else Stores.ROS1_NOETIC)
    types = store.types
    make_time = types["builtin_interfaces/msg/Time"]
    header = types["std_msgs/msg/Header"]
    vector = types["geometry_msgs/msg/Vector3"]
    twist = types["geometry_msgs/msg/Twist"]
    twist_type = "geometry_msgs/msg/TwistStamped"
    scan_type = "sensor_msgs/msg/LaserScan"
    writer = Writer2(Path(path)) if ros2 else Writer1(Path(path))
    serialize = store.serialize_cdr if ros2 else store.serialize_ros1
    with writer:
        sc = writer.add_connection("/scan", scan_type, typestore=store)
        wh = writer.add_connection("/wheel_twist", twist_type, typestore=store)
        for i, frame in enumerate(frames):
            stamp = round(frame.stamp * 1e9)
            time = make_time(*divmod(stamp, 10**9))

            def make_header(name):
                return header(time, name) if ros2 else header(0, time, name)

            dt = frame.dt or 1
            velocity = twist(
                vector(frame.distance / dt, 0.0, 0.0), vector(0.0, 0.0, frame.yaw / dt)
            )
            msg = types[twist_type](make_header("base_link"), velocity)
            writer.write(wh, stamp, serialize(msg, twist_type))
            if i in missing:
                continue
            half = np.deg2rad(spec.fov) / 2
            msg = types[scan_type](
                make_header("laser"),
                -half,
                half,
                2 * half / (spec.size - 1),
                frame.beam_dt_s,
                1 / spec.rate,
                0.02,
                spec.max_range,
                frame.ranges.astype(np.float32),
                np.empty(0, np.float32),
            )
            writer.write(sc, stamp, serialize(msg, scan_type))


def read_bag(path, spec, scan_topic="/scan", wheel_topic="/wheel_twist"):
    """Integrate every wheel sample up to each scan, including across scan dropouts."""
    scans, wheels = [], []
    with AnyReader([Path(path)], default_typestore=get_typestore(Stores.ROS2_HUMBLE)) as reader:
        for conn, _, raw in reader.messages():
            if conn.topic not in (scan_topic, wheel_topic):
                continue  # Old map/TF/odom and ground truth are deliberately excluded.
            msg = reader.deserialize(raw, conn.msgtype)
            stamp = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
            if conn.topic == scan_topic:
                if len(msg.ranges) != spec.size:
                    raise ValueError(f"Expected {spec.size} laser beams, got {len(msg.ranges)}")
                measured_fov = np.rad2deg(msg.angle_increment * (spec.size - 1))
                if (
                    abs(measured_fov - spec.fov) > 0.1
                    or abs(msg.angle_min + np.deg2rad(spec.fov) / 2) > 0.01
                ):
                    raise ValueError("Laser angles do not match declared adapter geometry")
                ranges = np.asarray(msg.ranges, float)
                # Preserve explicit +inf no-return rays; invalid readings are not free space.
                valid_hit = (
                    np.isfinite(ranges)
                    & (ranges >= msg.range_min)
                    & (ranges <= min(msg.range_max, spec.max_range))
                )
                ranges = np.where(valid_hit | np.isposinf(ranges), ranges, np.nan)
                scans.append((stamp, ranges, float(msg.time_increment)))
            else:
                wheels.append((stamp, msg.twist.linear.x, msg.twist.angular.z))
    if len(scans) < 2 or not wheels:
        raise ValueError("Input requires at least 2 scans and wheel_twist evidence")
    for name, records in (("scan", scans), ("wheel", wheels)):
        if np.any(np.diff([r[0] for r in records]) <= 0):
            raise ValueError(f"Non-monotonic {name} header timestamps")
    cursor, last = 0, wheels[0][0]
    frames = []
    for stamp, ranges, beam_dt_s in scans:
        distance = yaw = elapsed = 0.0
        while cursor < len(wheels) and wheels[cursor][0] <= stamp + 1e-7:
            ts, v, w = wheels[cursor]
            dt = ts - last
            distance += v * dt
            yaw += w * dt
            elapsed += dt
            last = ts
            cursor += 1
        frames.append(Scan(stamp, ranges, distance, yaw, elapsed, beam_dt_s))
    return frames


def write_tum(path, poses):
    rows = np.column_stack(
        (
            poses[:, :3],
            np.zeros(len(poses)),
            np.zeros((len(poses), 2)),
            np.sin(poses[:, 3] / 2),
            np.cos(poses[:, 3] / 2),
        )
    )
    np.savetxt(path, rows, fmt="%.9f", header="timestamp x y z qx qy qz qw")


def read_tum(path):
    rows = np.loadtxt(path, ndmin=2)
    if rows.shape[1] != 8 or len(rows) < 2 or not np.isfinite(rows).all():
        raise ValueError("TUM trajectory must contain finite timestamp x y z qx qy qz qw rows")
    if np.any(np.diff(rows[:, 0]) <= 0):
        raise ValueError("Trajectory timestamps must be strictly increasing")
    if not np.allclose(np.linalg.norm(rows[:, 4:], axis=1), 1, atol=1e-3):
        raise ValueError("Trajectory quaternion must have unit length")
    qx, qy, qz, qw = rows[:, 4:].T
    return np.column_stack(
        (rows[:, :3], np.arctan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz)))
    )
