import asyncio
import json
import os
import shutil
import subprocess
import threading
import time

import pytest
import rclpy
import websockets
from geometry_msgs.msg import Pose
from rclpy.node import Node

# The test message to send from the mock WebSocket server
TEST_MESSAGE = {
    "position": {"x": 1.0, "y": 2.0, "z": 3.0},
    "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
}

received_messages = []


@pytest.fixture(scope="module")
def websocket_server():
    """Start a mock WebSocket server that sends a valid message."""
    stop_event = threading.Event()

    async def ws_main():
        async def handler(websocket):
            await websocket.recv()  # Expect SUB command
            body = json.dumps(TEST_MESSAGE).encode("utf-8")
            msg = f"MSG test.subject 1 {len(body)}\r\n".encode() + body + b"\r\n"
            await websocket.send(msg)

        async with websockets.serve(handler, "localhost", 8089):
            while not stop_event.is_set():
                await asyncio.sleep(0.1)

    def start_server():
        asyncio.run(ws_main())

    thread = threading.Thread(target=start_server, daemon=True)
    thread.start()
    time.sleep(1)  # Let the server start

    yield  # Run the test

    stop_event.set()
    thread.join()


@pytest.fixture(scope="module")
def agent_process():
    """Launch the ROS 2 agent node using ros2 run."""
    assert shutil.which("ros2"), "'ros2' command not found in PATH"

    cmd = [
        "ros2",
        "run",
        "motion_agent",  # ROS 2 package name
        "agent",  # Entry point defined in setup.py
        "--ros-args",
        "-p",
        "url:=ws://localhost:8089",
        "-p",
        "sub:=test.subject",
    ]

    proc = subprocess.Popen(
        cmd,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        env={**os.environ, "RCUTILS_LOGGING_BUFFERED_STREAM": "1"},
    )

    time.sleep(2)  # Give time to connect
    yield proc

    proc.terminate()
    try:
        proc.wait(timeout=5)
    except subprocess.TimeoutExpired:
        proc.kill()


@pytest.fixture(scope="module")
def ros2_node():
    """Start a ROS 2 test node."""
    rclpy.init()
    node = Node("test_agent_subscriber")
    yield node
    node.destroy_node()
    rclpy.shutdown()


def test_pose_message_received(websocket_server, agent_process, ros2_node):
    """Verify that a Pose message is published to the /pose topic."""

    def callback(msg):
        received_messages.append(msg)

    ros2_node.create_subscription(Pose, "pose", callback, 10)

    # Wait up to 5 seconds for the message
    start = time.time()
    while not received_messages and time.time() - start < 5:
        rclpy.spin_once(ros2_node, timeout_sec=0.1)

    assert received_messages, "No message received on /pose topic"

    msg = received_messages[0]
    assert msg.position.x == pytest.approx(TEST_MESSAGE["position"]["x"])
    assert msg.position.y == pytest.approx(TEST_MESSAGE["position"]["y"])
    assert msg.position.z == pytest.approx(TEST_MESSAGE["position"]["z"])
    assert msg.orientation.w == pytest.approx(TEST_MESSAGE["orientation"]["w"])
