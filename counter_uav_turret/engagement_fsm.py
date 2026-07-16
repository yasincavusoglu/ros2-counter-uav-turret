#!/usr/bin/env python3
"""engagement_fsm — SEARCH -> TRACK_LOCK -> ENGAGE with safety interlocks.

Subscribes: /tracks, /target/state, /gimbal/state
Publishes:  /engagement/status (…) — current mode + fire-permit flag

Fire is permitted only when ALL hold: a track is locked, aim error is within tolerance, the target
is inside the allowed engagement zone, and a watchdog on perception is fresh. A safety mode forbids
fire entirely (demonstrates the interlock).

TODO(M3): state machine; aim-error/zone/watchdog gates; publish status + fire-permit.
"""
import rclpy
from rclpy.node import Node


class EngagementFsm(Node):
    def __init__(self):
        super().__init__('engagement_fsm')
        self.get_logger().info('engagement_fsm started (stub — see TODO).')
        # TODO: FSM states; interlock conditions; subs; pub /engagement/status.


def main(args=None):
    rclpy.init(args=args)
    node = EngagementFsm()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
