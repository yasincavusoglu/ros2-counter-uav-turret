#!/usr/bin/env python3
"""scenario_sim — maneuvering aerial target + synthetic camera (the test world).

The real system ran on real cameras/hardware that cannot be shared, and a person cloning this
repo has no drone and no camera. This node generates a moving aerial target trajectory and renders
a synthetic camera frame from the current gimbal pointing direction, so the whole pipeline runs
end-to-end with no hardware.

Publishes:  /camera/image  (sensor_msgs/Image)     — synthetic view from the turret camera
            /target/truth  (geometry_msgs/PointStamped) — ground-truth position (scoring only)
Subscribes: /gimbal/state  (…)                     — current pan/tilt, to aim the synthetic camera

TODO(M1): target trajectory model; render a small drone against sky given gimbal angles; publish frames.
"""
import rclpy
from rclpy.node import Node


class ScenarioSim(Node):
    def __init__(self):
        super().__init__('scenario_sim')
        self.get_logger().info('scenario_sim started (stub — see TODO).')
        # TODO: params (target speed, camera FOV, fps); publishers; render timer.


def main(args=None):
    rclpy.init(args=args)
    node = ScenarioSim()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
