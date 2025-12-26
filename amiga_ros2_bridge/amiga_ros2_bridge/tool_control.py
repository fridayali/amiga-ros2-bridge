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

import asyncio
from pathlib import Path

from farm_ng.canbus.tool_control_pb2 import (
    ActuatorCommands,
    HBridgeCommand,
    HBridgeCommandType,
    ToolStatuses,
)
from farm_ng.core.event_client import EventClient
from farm_ng.core.event_service_pb2 import EventServiceConfig
from farm_ng.core.events_file_reader import proto_from_json_file



try:
    from pynput import keyboard
    KEYBOARD_AVAILABLE = True
except Exception as e:
    print(f"[WARN] Keyboard control disabled (headless mode): {e}")
    KEYBOARD_AVAILABLE = False


class KeyboardListener:
    def __init__(self):
        self.pressed_keys = set()
        if not KEYBOARD_AVAILABLE:
            self.listener = None
            return

        self.listener = keyboard.Listener(
            on_press=self.on_press,
            on_release=self.on_release,
        )

    def on_press(self, key):
        try:
            self.pressed_keys.add(key.char)
        except AttributeError:
            self.pressed_keys.add(key.name)

    def on_release(self, key):
        try:
            self.pressed_keys.discard(key.char)
        except AttributeError:
            self.pressed_keys.discard(key.name)

    def start(self):
        if self.listener:
            self.listener.start()

    def stop(self):
        if self.listener:
            self.listener.stop()




TOOL_CONTROL_CFG = (
    Path(__file__).resolve().parent
    / "include"
    / "tool_control_config.json"
)




async def send_hbridge_command(direction: str):
    """
    Send a single H-Bridge command (forward / reverse / stop).
    """

    config: EventServiceConfig = proto_from_json_file(
        TOOL_CONTROL_CFG, EventServiceConfig()
    )
    client = EventClient(config)

    commands = ActuatorCommands()

    if direction == "forward":
        commands.hbridges.append(
            HBridgeCommand(id=0, command=HBridgeCommandType.HBRIDGE_FORWARD)
        )
    elif direction == "reverse":
        commands.hbridges.append(
            HBridgeCommand(id=0, command=HBridgeCommandType.HBRIDGE_REVERSE)
        )
    elif direction == "stop":
        commands.hbridges.append(
            HBridgeCommand(id=0, command=HBridgeCommandType.HBRIDGE_STOPPED)
        )
    else:
        raise ValueError("direction must be: forward / reverse / stop")

    await client.request_reply("/control_tools", commands, decode=True)
    print(f"[OK] HBridge -> {direction}")




async def control_tools_loop():
    """
    Continuous control loop.
    Runs forever and periodically sends commands.
    """

    config: EventServiceConfig = proto_from_json_file(
        TOOL_CONTROL_CFG, EventServiceConfig()
    )
    client = EventClient(config)

    print("[INFO] Starting tool control loop")

    while True:
        commands = ActuatorCommands()

        # EXAMPLE: always forward
        commands.hbridges.append(
            HBridgeCommand(id=0, command=HBridgeCommandType.HBRIDGE_FORWARD)
        )

        await client.request_reply("/control_tools", commands, decode=True)
        await asyncio.sleep(0.1)




async def stream_tool_statuses():
    """
    Print tool status messages.
    """

    config: EventServiceConfig = proto_from_json_file(
        TOOL_CONTROL_CFG, EventServiceConfig()
    )

    async for event, message in EventClient(config).subscribe(
        config.subscriptions[0], decode=True
    ):
        print("[STATUS]", message)




async def run():
    tasks = [
        asyncio.create_task(control_tools_loop()),
        asyncio.create_task(stream_tool_statuses()),
        asyncio.create_task(send_hbridge_command("reverse")),
    ]
    await asyncio.gather(*tasks)




if __name__ == "__main__":
    keyboard_listener = KeyboardListener()

    if KEYBOARD_AVAILABLE:
        keyboard_listener.start()
        print("[INFO] Keyboard control enabled")
    else:
        print("[INFO] Running headless (no keyboard)")

    try:
        asyncio.run(run())
    except KeyboardInterrupt:
        print("[INFO] Shutdown requested")
    finally:
        keyboard_listener.stop()
