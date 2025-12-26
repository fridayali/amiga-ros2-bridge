# Copyright (c) farm-ng, inc.
#
# Licensed under the Amiga Development Kit License (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     https://github.com/farm-ng/amiga-dev-kit/blob/main/LICENSE
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
from __future__ import annotations


# added libraries for ros2
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import BatteryState
from farm_ng.core.event_client import EventClient
from farm_ng.core.event_service_pb2 import SubscribeRequest
from .farmng_ros_conversions import farmng_path_to_ros_type
from .farmng_ros_conversions import farmng_to_ros_msg

from rclpy.qos import QoSProfile

# public symbols
__all__ = [
    "create_ros_publisher",
]


async def create_ros_publisher(
    node: Node,
    client: EventClient,
    subscribe_request: SubscribeRequest,
    publish_topic: str = "",
):
    farm_ng_topic: str = f"/{client.config.name}{subscribe_request.uri.path}"


    if subscribe_request.uri.path == "/motor_states":

        temp_topic = "/canbus/motor_states/temperatures"
        battery_topic = "/canbus/battery_state"

        node.get_logger().info(
            f"Subscribing to farm-ng topic: {farm_ng_topic} and publishing on ROS topics:\n"
            f"  {temp_topic} (Float32MultiArray)\n"
            f"  {battery_topic} (BatteryState)"
        )

        temp_pub = node.create_publisher(
            Float32MultiArray, temp_topic, QoSProfile(depth=10)
        )

        battery_pub = node.create_publisher(
            BatteryState, battery_topic, QoSProfile(depth=10)
        )

        async for event, message in client.subscribe(subscribe_request, decode=True):
            ros_msgs = farmng_to_ros_msg(event, message)

            temp_pub.publish(ros_msgs["temperatures"])
            battery_pub.publish(ros_msgs["battery"])

        return  

 

    topic: str = publish_topic if publish_topic else farm_ng_topic

    node.get_logger().info(
        f"Subscribing to farm-ng topic: {farm_ng_topic} and publishing on ROS topic: {topic}"
    )

    ros_msg_type = farmng_path_to_ros_type(subscribe_request.uri)
    ros_publisher = node.create_publisher(
        ros_msg_type, topic, QoSProfile(depth=10)
    )

    async for event, message in client.subscribe(subscribe_request, decode=True):
        ros_msgs = farmng_to_ros_msg(event, message)
        for msg in ros_msgs:
            ros_publisher.publish(msg)