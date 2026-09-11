#!/usr/bin/env python3
"""Drive obstacles around their original world poses through ROS 2 state services."""
import copy
import math
from pathlib import Path

import numpy as np
from PIL import Image
from scipy.ndimage import distance_transform_edt
import yaml
from ament_index_python.packages import get_package_share_directory

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from gazebo_msgs.msg import EntityState, ModelStates
from gazebo_msgs.srv import SetEntityState


def obstacle_state(name, base, index, count, t, amplitude, period):
    if period <= 0:
        raise ValueError("Obstacle period must be positive")
    phase = 2.0 * math.pi * index / max(1, count)
    offset = amplitude * math.sin(2.0 * math.pi * t / period + phase)
    state = EntityState()
    state.name = name
    state.reference_frame = "world"
    state.pose = copy.deepcopy(base)
    state.pose.position.x += offset * math.cos(phase)
    state.pose.position.y += offset * math.sin(phase)
    return state


class DynamicObstacleDriver(Node):
    def __init__(self):
        super().__init__("dynamic_obstacle_driver")
        self.declare_parameter("amplitude", 1.2)
        self.declare_parameter("period", 12.0)
        self.declare_parameter("update_rate", 20.0)
        self.declare_parameter("model_prefix", "dyn_obstacle_")
        self.declare_parameter("max_models", 8)
        self.declare_parameter("world", "medium")
        self.declare_parameter("obstacle_clearance", .4)
        self._amp = float(self.get_parameter("amplitude").value)
        self._period = float(self.get_parameter("period").value)
        if self._period <= 0 or self._amp < 0:
            raise ValueError("Invalid obstacle amplitude/period")
        prefix = self.get_parameter("model_prefix").value
        self._models = [f"{prefix}{i}" for i in range(int(self.get_parameter("max_models").value))]
        self._bases = {}
        self._amplitudes = {}
        maps = Path(get_package_share_directory("sim_worlds")) / "maps"
        meta = yaml.safe_load((maps / (self.get_parameter("world").value + ".yaml")).read_text())
        with Image.open(maps / meta["image"]) as image:
            free = np.asarray(image) > 250
        self._resolution = float(meta["resolution"])
        self._origin = meta["origin"]
        self._distance = distance_transform_edt(np.pad(free, 1))[1:-1, 1:-1] * self._resolution
        self._pending = {}
        self._warned = False
        self._client = self.create_client(SetEntityState, "/gazebo/set_entity_state")
        self.create_subscription(ModelStates, "/gazebo/model_states", self._on_states,
                                 qos_profile_sensor_data)
        self.create_timer(1.0 / max(1., float(self.get_parameter("update_rate").value)), self._tick)
        self.get_logger().info("Waiting for Gazebo model states and /gazebo/set_entity_state")

    def _on_states(self, msg):
        for name, pose in zip(msg.name, msg.pose):
            if name in self._models and name not in self._bases:
                self._bases[name] = copy.deepcopy(pose)
                self._amplitudes[name] = self._safe_amplitude(name, pose)
                self.get_logger().info(f"Obstacle origin captured: {name} ({pose.position.x:.2f}, {pose.position.y:.2f})")

    def _safe_amplitude(self, name, pose):
        phase = 2 * math.pi * self._models.index(name) / max(1, len(self._models))
        h, w = self._distance.shape
        clearance = float(self.get_parameter("obstacle_clearance").value)
        safe = 0.
        for amplitude in np.arange(0., self._amp + self._resolution/4, self._resolution/2):
            for sign in (-1, 1):
                x = pose.position.x + sign*amplitude*math.cos(phase)
                y = pose.position.y + sign*amplitude*math.sin(phase)
                ix = int(math.floor((x-self._origin[0])/self._resolution))
                iy = h-1-int(math.floor((y-self._origin[1])/self._resolution))
                if not (0 <= ix < w and 0 <= iy < h) or self._distance[iy, ix] < clearance:
                    return safe
            safe = min(float(amplitude), self._amp)
        return safe

    def _tick(self):
        if not self._client.service_is_ready():
            return
        t = self.get_clock().now().nanoseconds * 1e-9
        for index, name in enumerate(self._models):
            if name not in self._bases:
                continue
            previous = self._pending.get(name)
            if previous is not None:
                if not previous.done():
                    continue
                if (previous.exception() or not previous.result().success) and not self._warned:
                    self.get_logger().error(f"Gazebo rejected obstacle update: {name}")
                    self._warned = True
            request = SetEntityState.Request()
            request.state = obstacle_state(name, self._bases[name], index, len(self._models),
                                           t, self._amplitudes[name], self._period)
            self._pending[name] = self._client.call_async(request)


def main():
    rclpy.init()
    node = DynamicObstacleDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
