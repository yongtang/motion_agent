import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Pose

import asyncio, websockets, json

import json


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
        self.url = "ws://localhost:8081"
        self.sub = "test.subject"
        self.publisher_ = self.create_publisher(Pose, "pose", 10)

    def callback(self, data):
        msg = dict_to_ros2_msg(Pose, data)
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
                            head, body = response.split(b"\r\n", 1)
                            if head.startswith(b"MSG "):
                                assert body.endswith(b"\r\n")
                                body = body[:-2]

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
