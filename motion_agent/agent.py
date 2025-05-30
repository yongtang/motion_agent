import asyncio
import json

import rclpy
import websockets
from geometry_msgs.msg import Pose, PoseStamped
from rclpy.node import Node


def dict_to_ros2_msg(msg_type, data_dict):
    msg = msg_type()

    for key, value in data_dict.items():
        if hasattr(msg, key):
            field_type = type(getattr(msg, key))

            if isinstance(value, dict):  # Nested message
                nested_msg_type = field_type
                setattr(msg, key, dict_to_ros2_msg(nested_msg_type, value))

            elif isinstance(value, list):  # List of messages or primitives
                list_type = getattr(msg, key).__class__.__args__[
                    0
                ]  # Get the element type
                setattr(
                    msg,
                    key,
                    [
                        (
                            dict_to_ros2_msg(list_type, v)
                            if isinstance(v, dict)
                            else list_type(v)
                        )
                        for v in value
                    ],
                )

            else:  # Primitive types (int, float, str, bool, etc.)
                if field_type == float and isinstance(
                    value, int
                ):  # Fix int -> float issue
                    value = float(value)
                setattr(msg, key, value)

    return msg


class Agent(Node):

    def __init__(self):
        super().__init__("agent")
        self.declare_parameter("url", "ws://localhost:8081")
        self.declare_parameter("sub", "test.subject")
        self.declare_parameter("frame", "base_link")

        self.url = self.get_parameter("url").get_parameter_value().string_value
        self.sub = self.get_parameter("sub").get_parameter_value().string_value
        self.frame = self.get_parameter("frame").get_parameter_value().string_value

        self.publisher_ = self.create_publisher(PoseStamped, "pose", 10)

    def callback(self, data):
        pose = dict_to_ros2_msg(Pose, data)
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame
        msg.pose = pose
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg)


async def run(node):
    try:
        while True:
            try:
                async with websockets.connect(node.url) as ws:
                    await ws.send("SUB {} 1\r\n".format(node.sub))
                    while True:
                        try:
                            response = await asyncio.wait_for(ws.recv(), timeout=1.0)
                            node.get_logger().info("Response: {}".format(response))

                            # ✅ Support multiple MSG entries in one response
                            parts = response.split(b"\r\n")
                            for head, body in zip(*[iter(parts)] * 2):
                                if head.startswith(b"MSG "):
                                    op, sub, sid, count = head.split(b" ", 3)
                                    assert op == b"MSG"
                                    assert sub
                                    assert sid
                                    assert int(count) == len(body)

                                    data = json.loads(body)
                                    node.callback(data)

                        except asyncio.TimeoutError:
                            pass
            except (asyncio.CancelledError, KeyboardInterrupt):
                raise
            except Exception as e:
                node.get_logger().info("Exception: {}".format(e))
                await asyncio.sleep(1)
    except (asyncio.CancelledError, KeyboardInterrupt):
        node.get_logger().info("Cancel")
    finally:
        node.get_logger().info("Exit")


def main(args=None):
    rclpy.init(args=args)

    agent = Agent()

    asyncio.run(run(agent))

    agent.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
