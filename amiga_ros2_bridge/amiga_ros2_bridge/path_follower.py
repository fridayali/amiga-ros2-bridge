#!/usr/bin/env python3
import asyncio
import threading
from pathlib import Path
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time
from farm_ng.core.event_client import EventClient
from farm_ng.core.event_service_pb2 import EventServiceConfig
from farm_ng.core.events_file_reader import proto_from_json_file
from farm_ng.track.track_pb2 import Track, TrackFollowerState, TrackFollowRequest,TRACK_COMPLETE
from google.protobuf.empty_pb2 import Empty

from tool_control import send_hbridge_command 

TRACKS_DIR = Path(__file__).resolve().parent.parent / "tracks"
TRACK_HOME_TO_ZONE1 = TRACKS_DIR / "nhometozone1.json"
TRACK_ZONE1_TO_P1   = TRACKS_DIR / "agaconu1.json"
TRACK_P1_TO_P2      = TRACKS_DIR / "agacarkasi.json"
TRACK_P2_TO_P3      = TRACKS_DIR / "agacarkasi2.json"
TRACK_P3_TO_P4      = TRACKS_DIR / "nzonep2.json"
TRACK_P4_TO_P5      = TRACKS_DIR / "nzone1p3.json"
TRACK_P5_TO_ZONE1   = TRACKS_DIR / "nzone1p3top1.json"
TRACK_P6_TO_HOME    = TRACKS_DIR / "nzone1p3home.json"

SERVICE_CFG = (
    Path(__file__).resolve().parent
    / "include"
    / "path_follower_config.json"
)
### Async event queue ###
MISSION_EVENTS = asyncio.Queue()

current_mission = None

class MissionBridge(Node):
    def __init__(self):
        super().__init__("mission_bridge")

        # farm-ng config
        self.config: EventServiceConfig = proto_from_json_file(SERVICE_CFG, EventServiceConfig())
        self.client: EventClient = EventClient(self.config)

        # asyncio loop for farm-ng
        self.loop = asyncio.new_event_loop()
        threading.Thread(target=self._run_loop, daemon=True).start()

        # ROS2 subscribers
        self.create_subscription(String, "/mission", self.mission_cb, 10)
        self.create_subscription(String, "/goal_name", self.goal_cb, 10)

        self.get_logger().info("MissionBridge running. Listening for ROS2 commands...")

        # start asyncio task
        asyncio.run_coroutine_threadsafe(
            process_missions(self.config),
            self.loop
        )

    def _run_loop(self):
        asyncio.set_event_loop(self.loop)
        self.loop.run_forever()

    def mission_cb(self, msg):
        global current_mission
        current_mission=msg.data

    def goal_cb(self, msg):
        asyncio.run_coroutine_threadsafe(
            MISSION_EVENTS.put(("goal", msg.data)),
            self.loop
        )


async def load_track(client: EventClient, path: str):
    print("loading the track file")
    track = proto_from_json_file(Path(path), Track())
    print("setting the track file  to the amiga")
    await client.request_reply("/set_track", TrackFollowRequest(track=track))
    print("starting the track")
    await client.request_reply("/start", Empty())
    print("track is started")

async def wait_track_complete(client: EventClient):
    async for ev, msg in client.subscribe(client.config.subscriptions[0], decode=True):
        try:
            if msg.status.track_status == TRACK_COMPLETE:
                print(msg)
                
                return
        except:
            print("message format is wrong")


async def timed_hbridge(direction: str, duration: float = 7.0):
    """
    direction: "forward" (down) or "reverse" (up)
    duration: kaç saniye çalışacağı
    """
    start = time.time()
    while True:
        await send_hbridge_command(direction)
        await asyncio.sleep(0.1)  

        if time.time() - start >= duration:
            await send_hbridge_command("stop")
            print(f"{duration} saniyelik {direction} işlemi bitti.")
            break


async def do_full_sequence(client: EventClient):
    global current_mission
    if current_mission=="cultivation":
        print("→ Tool DOWN (7s)")
        await timed_hbridge("forward", 12)

    print("→ Going to Point 1")
    await load_track(client, TRACK_ZONE1_TO_P1)
    await wait_track_complete(client)

    if current_mission=="cultivation":    
        print("→ Tool UP (7s)")
        await timed_hbridge("reverse",14 )

    print("→ Going to Point 2")
    await load_track(client, TRACK_P1_TO_P2)
    await wait_track_complete(client)

    if current_mission=="cultivation":
        print("→ Tool DOWN (7s)")
        await timed_hbridge("forward", 12)

    print("→ Going to Point 3")
    await load_track(client, TRACK_P2_TO_P3)
    await wait_track_complete(client)

    if current_mission=="cultivation":
        print("→ Tool UP (7s)")
        await timed_hbridge("reverse", 14)

    await load_track(client, TRACK_P3_TO_P4)
    await wait_track_complete(client)

    if current_mission=="cultivation":
        print("→ Tool UP (7s)")
        await timed_hbridge("forward", 12)

    await load_track(client, TRACK_P4_TO_P5)
    await wait_track_complete(client)
    if current_mission=="cultivation":
        print("→ Tool UP (7s)")
        await timed_hbridge("reverse", 14)


    print("FULL SEQUENCE COMPLETE.")




async def process_missions(service_cfg: EventServiceConfig):
    client = EventClient(service_cfg)

    current_goal = None
    current_mission = None
    current_position = "home"
    while True:
        event_type, value = await MISSION_EVENTS.get()

        if event_type == "goal":
            current_goal = value

        elif event_type == "mission":
            current_mission = value

        print(f"[EVENT] goal={current_goal} | mission={current_mission}")


        if current_goal == "home" and current_position !="home":
            print("→ Going HOME")
            await load_track(client, TRACK_P6_TO_HOME)
            await wait_track_complete(client)
            current_goal=None
            current_position= "home"

            print("Arrived home.")

        if current_goal == "zone" and current_position!="zone":
            print("→ Going to Zone 1")
            await load_track(client, TRACK_HOME_TO_ZONE1)
            await wait_track_complete(client)
            current_goal=None
            await do_full_sequence(client)
            current_mission = None
            current_position = "zone"
            
        if current_goal == "zone" and current_position=="zone":
            print("→ Going to Zone 1")
            await load_track(client, TRACK_P5_TO_ZONE1)
            await wait_track_complete(client)
            current_goal=None
            await do_full_sequence(client,)
            current_mission = None
            current_position = "zone"


def main():

    rclpy.init()
    node = MissionBridge()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
