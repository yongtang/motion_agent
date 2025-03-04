import rclpy
from rclpy.node import Node

from std_msgs.msg import String

import asyncio


class Agent(Node):

    def __init__(self):
        super().__init__("agent")
        self.publisher_ = self.create_publisher(String, "topic", 10)
        self.i = 0

    def callback(self):
        msg = String()
        msg.data = "Hello World: %d" % self.i
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1


async def run(node):
    try:
        while True:
            node.callback()
            await asyncio.sleep(1)
    except Exception as e:
        node.get_logger().info("Exception: {}".format(e))


def main(args=None):
    rclpy.init(args=args)

    agent = Agent()

    asyncio.run(run(agent))

    agent.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
