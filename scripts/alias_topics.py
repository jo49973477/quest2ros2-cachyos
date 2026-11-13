#!/usr/bin/env python3
"""Optional *topic-name-only* compatibility shim.

Newer Quest2ROS docs use /right_hand/pose, /right_hand/twist,
/right_hand/inputs; Taokt's monitor expects /q2r_right_hand_pose, etc.
This bridge does NOT fix incompatible serialized wire protocols or custom
message schemas. Start only if the slash-style topics actually exist.
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from quest2ros.msg import OVR2ROSInputs


class AliasTopics(Node):
    def __init__(self):
        super().__init__('quest_topic_aliases')
        # rclpy.node.Node exposes read-only subscriptions/publishers
        # properties, so keep our references under private, non-reserved names.
        self._alias_subscriptions = []
        self._alias_publishers = []
        for side in ('left', 'right'):
            for field, typ in (
                ('pose', PoseStamped),
                ('twist', Twist),
                ('inputs', OVR2ROSInputs),
            ):
                original = f'/{side}_hand/{field}'
                aliased = f'/q2r_{side}_hand_{field}'
                pub = self.create_publisher(typ, aliased, 10)
                sub = self.create_subscription(
                    typ, original, lambda msg, publisher=pub: publisher.publish(msg), 10
                )
                self._alias_publishers.append(pub)
                self._alias_subscriptions.append(sub)
                self.get_logger().info(f'{original} -> {aliased}')


def main():
    rclpy.init()
    node = AliasTopics()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
