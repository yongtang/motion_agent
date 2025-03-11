import rclpy
from rclpy.node import Node

from std_msgs.msg import String

import asyncio, websockets, json


class Agent(Node):

    def __init__(self):
        super().__init__("agent")
        self.url = "ws://localhost:8081"
        self.sub = "test.subject"
        self.publisher_ = self.create_publisher(String, "topic", 10)

    def callback(self, data):
        msg = String()
        msg.data = "Received: {}".format(data)
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)


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
