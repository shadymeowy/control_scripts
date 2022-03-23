#!/usr/bin/env python3

"""
Caveat when attempting to run the examples in non-gps environments:
`drone.offboard.stop()` will return a `COMMAND_DENIED` result because it
requires a mode switch to HOLD, something that is currently not supported in a
non-gps environment.
"""

import asyncio
from math import sqrt

from mavsdk import System
from mavsdk.offboard import (OffboardError, PositionNedYaw)
from time import perf_counter

from math import sqrt, cos, sin, pi
from multiprocessing import Process, Queue


class Drone:
    def __init__(self):
        self.msg_queue = Queue()
        self.msg_queue2 = Queue()
        self.msg_queue3 = Queue()

    def _run(self):
        asyncio.run(self.run())

    def _send(self, *args):
        self.msg_queue.put(tuple(args))

    def _recv(self):
        return self.msg_queue.get()

    def _send2(self, *args):
        self.msg_queue2.put(tuple(args))

    def _recv2(self):
        return self.msg_queue2.get()

    def _send3(self):
        self.msg_queue3.put(None)

    def _recv3(self):
        if self.msg_queue3.empty():
            return False
        else:
            self.msg_queue3.get()
            return True

    def start(self):
        self.p = Process(target=type(self)._run, args=((self),))
        self.p.start()
        return self.move(0, 0, 0)

    def land(self):
        self._send(2)
        self.p.join()

    def move(self, dx, dy, dz, speed=1):
        self._send(0, dx, dy, dz, speed)
        return self._recv2()

    def turn(self, dyaw, speed=180):
        self._send(1, dyaw, speed)
        return self._recv2()

    def move_nowait(self, dx, dy, dz, speed=1):
        self._send(0, dx, dy, dz, speed)

    def turn_nowait(self, dyaw, speed=1):
        self._send(1, dyaw, speed)

    def wait(self):
        return self._recv2()

    def stop(self):
        self._send3()
        return self._recv2()

    async def run(self):
        drone = System()
        # await drone.connect(system_address="serial:///dev/ttyAMA0:460800")
        await drone.connect(system_address="udp://:14551")

        print("Waiting for drone to connect...")
        async for state in drone.core.connection_state():
            if state.is_connected:
                print("Drone discovered!")
                break

        print("Waiting for drone to have a global position estimate...")
        async for health in drone.telemetry.health():
            if health.is_global_position_ok:
                print("Global position estimate ok")
                break

        print("Fetching amsl altitude at home location....")
        async for terrain_info in drone.telemetry.home():
            pg0 = terrain_info.latitude_deg, terrain_info.longitude_deg, terrain_info.absolute_altitude_m
            break

        print("Fetching current heading...")
        async for ahrs in drone.telemetry.attitude_euler():
            yaw0 = ahrs.yaw_deg*pi/180
            print("Current heading: %.2f" % yaw0)
            break

        print("-- Wait for arming")
        async for armed in drone.telemetry.armed():
            if armed:
                break

        print("-- Setting initial setpoint")
        await drone.offboard.set_position_ned(PositionNedYaw(0.0, 0.0, 0.0, yaw0))

        print("-- Starting offboard")
        try:
            await drone.offboard.start()
        except OffboardError as error:
            print(f"Starting offboard mode failed with error code: {error._result.result}")
            print("-- Disarming")
            await drone.action.disarm()
            return

        async def get_position():
            async for position in drone.telemetry.position_velocity_ned():
                return [position.position.north_m, position.position.east_m, position.position.down_m]

        async def get_yaw():
            async for ahrs in drone.telemetry.attitude_euler():
                return ahrs.yaw_deg

        await drone.offboard.set_position_ned(PositionNedYaw(*(await get_position())[:-1], -1, (await get_yaw())))
        await asyncio.sleep(10)

        dl = 0.01
        dw = 0.1

        while True:
            if self.msg_queue.empty():
                await asyncio.sleep(0)
                continue
            msg = self._recv()

            if msg[0] == 0:
                _, dx, dy, dz, speed = msg
                yaw = (await get_yaw())*pi/180
                dx, dy = cos(yaw) * dx - sin(yaw) * dy, sin(yaw) * dx + cos(yaw) * dy
                x, y, z = await get_position()
                tx, ty, tz = x + dx, y + dy, z + dz
                start = perf_counter()
                t = 0
                while (x - tx)**2 + (y - ty)**2 + (z - tz)**2 > dl**2:
                    current = perf_counter()
                    while current - start > t:
                        ll = sqrt((x - tx)**2 + (y - ty)**2 + (z - tz)**2)
                        x -= dl * (x - tx) / ll
                        y -= dl * (y - ty) / ll
                        z -= dl * (z - tz) / ll
                        t += dl / speed
                    await drone.offboard.set_position_ned(PositionNedYaw(x, y, z, yaw*180/pi))
                    await get_position()
                    if self._recv3():
                        break
                else:
                    await drone.offboard.set_position_ned(PositionNedYaw(tx, ty, tz, yaw*180/pi))

            elif msg[0] == 1:
                _, dyaw, speed = msg
                x, y, z = await get_position()
                yaw = await get_yaw()
                tyaw = yaw + dyaw
                start = perf_counter()
                t = 0
                while abs(yaw - tyaw) > dw:
                    current = perf_counter()
                    while current - start > t:
                        yaw -= dw * (yaw - tyaw) / abs(yaw - tyaw)
                        t += dw / speed
                    await drone.offboard.set_position_ned(PositionNedYaw(x, y, z, yaw))
                    await get_yaw()
                    if self._recv3():
                        break
                else:
                    await drone.offboard.set_position_ned(PositionNedYaw(x, y, z, tyaw))

            elif msg[0] == 2:
                break

            self._send2([*(await get_position()), await get_yaw()])
            await asyncio.sleep(0)

        await asyncio.sleep(1)

        print("-- Stopping offboard")
        try:
            await drone.offboard.stop()
        except OffboardError as error:
            print(f"Stopping offboard mode failed with error code: {error._result.result}")

        await asyncio.sleep(1)

        print("-- Landing")
        await drone.action.land()


if __name__ == "__main__":
    drone = Drone()
    drone.start()
    while True:
        try:
            msg = input(">> ")
            if msg == "stop":
                drone.stop()
            elif msg == "amove":
                dx, dy, dz = map(float, input("dx dy dz: ").split())
                drone.move_nowait(dx, dy, dz)
            elif msg == "aturn":
                dyaw = float(input("dyaw: "))
                drone.turn_nowait(dyaw)
            elif msg == "move":
                dx, dy, dz = map(float, input("dx dy dz: ").split())
                drone.move(dx, dy, dz)
            elif msg == "turn":
                dyaw = float(input("dyaw: "))
                drone.turn(dyaw)
            elif msg == "wait":
                drone.wait()
            elif msg == "land":
                drone.land()
                break
            
        except Exception as e:
            print(e)
