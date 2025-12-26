#!/usr/bin/env python3

"""

This python script receive command from websocket, such as goal pose, camera request, some config files and missions
and also send current pose, battery, motor temp, tool status etc.
and listen ros2 topics that are gps, rtk/odom, motor_state, battery state, camera1 etc. 





"""
import numpy as np 
from sensor_msgs.msg import CompressedImage
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
import asyncio
import threading
import requests
import websockets
import json
import time
from sensor_msgs.msg import NavSatFix, BatteryState,Image
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32MultiArray, String
import math
from tf_transformations import quaternion_from_euler
import cv2
import base64
from geometry_msgs.msg import TwistStamped
from datetime import datetime
from pathlib import Path





ROBOT_ID = 6
BASE_URL = "https://backend.agrobrain.com.tr"
REGISTER_URL = f"{BASE_URL}/amiga/register"
ROBOT_PAYLOAD = {
    "robot_id": ROBOT_ID,
    "sensor_states": ["GNSS", "LIDAR", "RGB_CAMERA1", "RGB_CAMERA2","DEPTH_CAMERA","SOIL_SENSOR"]
}
ROBOT_SENSORS = {"sensor_states": ["GNSS", "LIDAR", "RGB_CAMERA1", "RGB_CAMERA2", "DEPTH_CAMERA"]}

from enum import Enum

class CameraState(Enum):
    RGB_CAMERA1_ON = "RGB_CAMERA1_ON"
    RGB_CAMERA1_OFF = "RGB_CAMERA1_OFF"
    RGB_CAMERA1_GET = "RGB_CAMERA1_GET"


class TelemetryListener(Node):
    def __init__(self):
        super().__init__('telemetry_listener')
        self.ws_url = None
        self.config_data = None
        self.telemetry_data = {
            "lat": None,
            "lon": None,
            "heading": None,
            "motor_temps": [0.0, 0.0, 0.0, 0.0],#TODO ADD MOTOR TEMPS AND BATTERY STATUS DEPENDS RECEIVED FRAME 
            "battery_state": 0.0,
            "tool_status": "STATUS_UNKNOWN",
        }
        self.topic_timeout_sec = 6.0   
        self.topic_check_period = 3.0  

        self.watchdog_topics = [
            '/gps/pvt',
            '/filter/state',
            '/oak1/rgb/image_raw',
            "/canbus/twist"
        ]

        now = self.get_clock().now()
        self.last_rx_time = {t: now for t in self.watchdog_topics}
        self.seen_once = {t: False for t in self.watchdog_topics}

        self.watchdog_timer = self.create_timer(
            self.topic_check_period,
            self._check_topics
        )

        

        self.camera_data={
            "RGB_CAMERA1":None
        }
        self.cameras_state=CameraState.RGB_CAMERA1_OFF
        self.telemetry_lock = threading.Lock()
        
        self.create_subscription(NavSatFix, '/gps/pvt', self.gps_callback, 10)
        self.create_subscription(Odometry, '/filter/state', self.heading_callback, 10)
        self.create_subscription(Float32MultiArray, '/canbus/motor_states/temperatures', self.motor_callback, 10) 
        self.create_subscription(BatteryState, '/canbus/battery_state', self.battery_callback, 10)
        self.goal_name_pub=self.create_publisher(String,'/goal_name',10)
        self.create_subscription(
            TwistStamped,
            '/canbus/twist',
            self.canbus_twist_callback,
            10
        )

        self.subscription = self.create_subscription(
            CompressedImage,
            "/oak1/rgb/image_raw",     
            self.image_callback,
            10
)

        self.mission_pub=self.create_publisher(String,"/mission",10)
        # self.goal_pose_pub=self.create_publisher(Float32MultiArray,'/goal_pose_webserver',10)
        self.camera_state_pub=self.create_publisher(String,'/camera_state',10)
        self.goal_state_pub=self.create_publisher(String,'/goal_state',10)
        

        self.loop = asyncio.new_event_loop()
        threading.Thread(target=self._run_async_loop, daemon=True).start()

        asyncio.run_coroutine_threadsafe(self.run(), self.loop)

        self.get_logger().info("Telemetry Listener node initialized")


    def _run_async_loop(self):
        asyncio.set_event_loop(self.loop)
        self.loop.run_forever()


    def image_callback(self, msg: CompressedImage):
        self.last_rx_time['/oak1/rgb/image_raw'] = self.get_clock().now()
        self.seen_once['/oak1/rgb/image_raw'] = True

        self.camera_data["RGB_CAMERA1"] = None

        if self.cameras_state == CameraState.RGB_CAMERA1_OFF:
            return

        try:
            np_arr = np.frombuffer(msg.data, np.uint8)

            cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

            if cv_image is None:
                self.get_logger().warn("Failed to decode compressed image")
                return

            # OpenCV image → JPEG encode (again, for base64)
            ret, buffer = cv2.imencode(".jpg", cv_image)
            if not ret:
                self.get_logger().warn("Failed to encode image")
                return

            jpg_as_text = base64.b64encode(buffer).decode("utf-8")

            with self.telemetry_lock:
                self.camera_data["RGB_CAMERA1"] = jpg_as_text

        except Exception as e:
            self.get_logger().error(f"Image callback error: {e}")
    def canbus_twist_callback(self, msg: TwistStamped):
        self.last_rx_time['/canbus/twist'] = self.get_clock().now()
        self.seen_once['/canbus/twist'] = True
        pass
              
    def gps_callback(self, msg: NavSatFix):
        self.last_rx_time['/gps/pvt'] = self.get_clock().now()
        self.seen_once['/gps/pvt'] = True
        with self.telemetry_lock:
            self.telemetry_data["lat"] = msg.latitude
            self.telemetry_data["lon"] = msg.longitude

    def heading_callback(self, msg: Odometry):
        self.last_rx_time['/filter/state'] = self.get_clock().now()
        self.seen_once['/filter/state'] = True
        q = msg.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y**2 + q.z**2)
        heading = math.degrees(math.atan2(siny_cosp, cosy_cosp))
        with self.telemetry_lock:
            self.telemetry_data["heading"] = heading

    def motor_callback(self, msg: Float32MultiArray):
        with self.telemetry_lock:
            self.telemetry_data["motor_temps"] = list(msg.data[:4])

    def battery_callback(self, msg: BatteryState):
        with self.telemetry_lock:

            self.telemetry_data["battery_state"] = int((msg.percentage)) 

    def tool_callback(self, msg: String):
        with self.telemetry_lock:
            self.telemetry_data["tool_status"] = msg.data

    def is_all_null(self, data):
        return all(v is None or v == [None, None, None, None] for v in data.values())
    
    def _log_to_file(self, message: str):
        log_path = Path(__file__).parent / "log.txt"
        timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")

        with open(log_path, "a") as f:
            f.write(f"[{timestamp}] {message}\n")

    
    def _check_topics(self):
        now = self.get_clock().now()

        for topic in self.watchdog_topics:
            dt = (now - self.last_rx_time[topic]).nanoseconds / 1e9

            if dt > self.topic_timeout_sec:
                if self.seen_once[topic]:
                    msg = f"[WATCHDOG] Topic timeout: {topic} | {dt:.1f}s"
                    self.get_logger().error(msg)
                    self._log_to_file(msg)
                else:
                    msg = f"[WATCHDOG] Topic NEVER received: {topic}"
                    self.get_logger().error(msg)
                    self._log_to_file(msg)



    # def send_goal(self, lat, lon, heading, mission_name):

    #     goal_msg = Float32MultiArray()
    #     goal_msg.data = [float(lat), float(lon), float(heading)]
    #     self.goal_pose_pub.publish(goal_msg)


    #     mission_msg = String()
    #     mission_msg.data = str(mission_name) 
    #     self.mission_pub.publish(mission_msg)

    #     self.get_logger().info(f"Sending: {goal_msg.data}, Mission: {mission_msg.data}")

    def feedback_callback(self, feedback_msg):
        with self.telemetry_lock:
            pass 

    def register_robot(self, retries=5, delay=2):
        for attempt in range(1, retries + 1):
            try:

                response = requests.post(REGISTER_URL, json=ROBOT_PAYLOAD, timeout=5)
                response.raise_for_status()
                self.config_data = response.json()
                self.ws_url = self.config_data.get("websocket_url")
                self.get_logger().info(f"Robot registered. WS URL: {self.ws_url}")
                return self.ws_url
            except requests.exceptions.RequestException as e:
                self.get_logger().warn(f"Attempt {attempt}/{retries} - HTTP register failed: {e}")
                if attempt < retries:
                    time.sleep(delay) 
                else:
                    self.get_logger().error("Max retries reached. Could not register robot.")
                    return None

    async def send_telemetry(self, websocket):
        while True:
            try:
                with self.telemetry_lock:
                    data_to_send_telemetry = self.telemetry_data.copy()
                    data_to_send_camera = self.camera_data.copy()
                    print(data_to_send_telemetry)
                    await websocket.send(json.dumps(data_to_send_telemetry))
                    print(data_to_send_telemetry)
    
                #await websocket.send(json.dumps(data_to_send_camera))
                if self.cameras_state==CameraState.RGB_CAMERA1_OFF:
                    print("camera_off")
                    pass
                elif self.cameras_state==CameraState.RGB_CAMERA1_GET:
                    print("camera_get")
                    await websocket.send(json.dumps(data_to_send_camera))
                    self.cameras_state=CameraState.RGB_CAMERA1_OFF
                elif self.cameras_state==CameraState.RGB_CAMERA1_ON:
                    print("camera_on")
                    await websocket.send(json.dumps(data_to_send_camera))

                await asyncio.sleep(2)
            except websockets.exceptions.ConnectionClosed:
                self.get_logger().warn("WebSocket closed while sending telemetry")
                break

    async def receive_commands(self, websocket):
        try:
            async for message in websocket:
                self.get_logger().info(f"Command received: {message}")
                data = json.loads(message)

                goal_name = data.get("goal")          
                mission_name = data.get("mission")    
                cameras_state_str = data.get("camera_state") 
                
                # zone_id = data.get("zone_id")
                # tool_id = data.get("tool_id")

                if cameras_state_str:
                    try:
                        with self.telemetry_lock:
                            self.cameras_state = CameraState(cameras_state_str)
                        self.get_logger().info(f"Camera state updated: {self.cameras_state}")
                    except ValueError:
                        self.get_logger().warn(f"Invalid camra state: {cameras_state_str}")
                        with self.telemetry_lock:
                            self.cameras_state = CameraState.RGB_CAMERA1_OFF 

                target_pose_data = None  

                if self.config_data is None:
                    self.get_logger().error("Config was not found.")
                    continue 
                

                if goal_name is not None:
                    goal_pub_msg = String()
                    goal_pub_msg.data = str(goal_name)
                    self.get_logger().info(f"Goal name: {goal_name}")
                    self.goal_name_pub.publish(goal_pub_msg)

                if mission_name is not None:
                    mission_pub_msg = String()
                    mission_pub_msg.data = str(mission_name)
                    self.mission_pub.publish(mission_pub_msg)
                    self.get_logger().info(f"Mission published: {mission_name}")
                  

        except: 
            pass

    async def run(self, reconnect_delay=5):
        while rclpy.ok(): 
            ws_url = self.register_robot() 
            if not ws_url:
                self.get_logger().error(f"Could not get WS URL. Retrying in {reconnect_delay}s...")
                await asyncio.sleep(reconnect_delay) 
                continue
            try:
                self.get_logger().info(f"Connecting to WS: {ws_url}")
                async with websockets.connect(ws_url) as websocket:
                    await websocket.send(json.dumps(ROBOT_PAYLOAD))
                    print("bağlantı başarılı")
                    send_task = asyncio.create_task(self.send_telemetry(websocket))
                    recv_task = asyncio.create_task(self.receive_commands(websocket))
                    
                    await asyncio.gather(send_task, recv_task)
            except (websockets.exceptions.ConnectionClosed, ConnectionRefusedError) as e:
                self.get_logger().warn(f"WebSocket disconnected: {e}. Reconnecting in {reconnect_delay}s...")
                await asyncio.sleep(reconnect_delay) 
            except Exception as e:
                self.get_logger().error(f"Unexpected WebSocket error: {e}")
                await asyncio.sleep(reconnect_delay) 
 


def main():
    rclpy.init()
    node = TelemetryListener()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Closing...')
    finally:

        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
