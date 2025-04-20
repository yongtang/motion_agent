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
from geometry_msgs.msg import PoseStamped  # ✅ Updated import
from rclpy.node import Node

# The test message to send from the mock WebSocket server
TEST_MESSAGE = {
    "position": {"x": 1.0, "y": 2.0, "z": 3.0},
    "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
}

received_messages = []


@pytest.fixture(scope="module")
def websocket_server():
    stop_event = threading.Event()

    async def ws_main():
        async def handler(websocket):
            await websocket.recv()  # Expect SUB command

            body = json.dumps(TEST_MESSAGE).encode("utf-8")
            msg = f"MSG test.subject 1 {len(body)}\r\n".encode() + body + b"\r\n"

            # Send 1 message
            await websocket.send(msg)
            # Send 2 messages in one frame
            await websocket.send(msg + msg)

        async with websockets.serve(handler, "localhost", 8089):
            while not stop_event.is_set():
                await asyncio.sleep(0.1)

    def start_server():
        asyncio.run(ws_main())

    thread = threading.Thread(target=start_server, daemon=True)
    thread.start()
    time.sleep(1)

    yield
    stop_event.set()
    thread.join()


@pytest.fixture(scope="module")
def agent_process():
    assert shutil.which("ros2"), "'ros2' command not found in PATH"

    cmd = [
        "ros2",
        "run",
        "motion_agent",
        "agent",
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

    time.sleep(2)
    yield proc

    proc.terminate()
    try:
        proc.wait(timeout=5)
    except subprocess.TimeoutExpired:
        proc.kill()


@pytest.fixture(scope="module")
def ros2_node():
    rclpy.init()
    node = Node("test_agent_subscriber")
    yield node
    node.destroy_node()
    rclpy.shutdown()


def test_pose_message_received(websocket_server, agent_process, ros2_node):
    """Verify that multiple PoseStamped messages are published to the /pose topic."""

    received_messages.clear()

    def callback(msg):
        print(f"[DEBUG] Received message of type: {type(msg)}")
        received_messages.append(msg)

    ros2_node.create_subscription(PoseStamped, "pose", callback, 10)

    # Wait up to 5 seconds for 3 messages
    start = time.time()
    while len(received_messages) < 3 and time.time() - start < 5:
        rclpy.spin_once(ros2_node, timeout_sec=0.1)

    print(f"[DEBUG] Total received messages: {len(received_messages)}")

    assert (
        len(received_messages) >= 3
    ), f"Expected at least 3 messages, got {len(received_messages)}"

    for msg in received_messages[:3]:
        pose = msg.pose
        assert pose.position.x == pytest.approx(TEST_MESSAGE["position"]["x"])
        assert pose.position.y == pytest.approx(TEST_MESSAGE["position"]["y"])
        assert pose.position.z == pytest.approx(TEST_MESSAGE["position"]["z"])
        assert pose.orientation.w == pytest.approx(TEST_MESSAGE["orientation"]["w"])
