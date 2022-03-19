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
import numpy as np

from math import sqrt, cos, sin

WAIT_TIME = 3


def calc_path(pointss, dl=0.005, speed=20, x0=0, y0=0, z0=-2, yaw0=0, focal_points=[]):
    x = []
    y = []
    z = []
    yaw = []
    t = []

    x_ = x0
    y_ = y0
    z_ = z0
    yaw_ = yaw0
    t_ = 0
    for points, focal_point in zip(pointss, focal_points):
        for point in points:
            while True:
                tx, ty, tz = point
                if (x_ - tx)**2 + (y_ - ty)**2 + (z_ - tz)**2 <= dl**2:
                    break
                ll = sqrt((x_ - tx)**2 + (y_ - ty)**2 + (z_ - tz)**2)
                x_ -= dl * (x_ - tx) / ll
                y_ -= dl * (y_ - ty) / ll
                z_ -= dl * (z_ - tz) / ll
                t_ += dl / speed
                if focal_point is not None:
                    yaw_ = np.arctan2(y_ - focal_point[1], x_ - focal_point[0]) / np.pi * 180 + 180
                else:
                    yaw_ = 0
                x.append(x_)
                y.append(y_)
                z.append(z_)
                yaw.append(yaw_)
                t.append(t_)
            x.append(x[-1])
            y.append(y[-1])
            z.append(z[-1])
            yaw.append(yaw[-1])
            t_ += WAIT_TIME
            t.append(t_)

    return x, y, z, yaw, t


def calc_shape(path, length=(5, 5, 5), offset=(0, 0, 1), rvrs=(1, 1, -1)):
    return [[rvrs[k] * (p[k] * length[k] + offset[k]) for k in range(3)] for p in path]


triangle = [
    [0.866, 0.5, 0],
    [0.866, -0.5, 0],
    [1.732, 0, 0],
    [0.866, 0.5, 0],
    [0.866, -0.5, 0],
    [1.732, 0, 0],
    [0.866, 0.5, 0],
    [0.866, -0.5, 0],
    [1.732, 0, 0],
    [0.866, 0.5, 0],
    [0.866, -0.5, 0],
    [1.732, 0, 0],
    [0.866, 0.5, 0],
    [0, 0, 0]
]
focal_point = calc_shape([[1.1547, 0, 0]])[0]

triangle2 = [
    [0.43301270189, 0, 0],
    [-0.43301270189, 0.5, 0],
    [-0.43301270189, -0.5, 0],

    [0.43301270189, 0, 0],
    [-0.43301270189, 0.5, 0],
    [-0.43301270189, -0.5, 0],

    [0.43301270189, 0, 0],
    [-0.43301270189, 0.5, 0],
    [-0.43301270189, -0.5, 0],

    [0.43301270189, 0, 0],
    [-0.43301270189, 0.5, 0],
    [-0.43301270189, -0.5, 0],

    [0, 0, 0]
]
focal_point2 = calc_shape([[-0.1443375673, 0, 0]])[0]

cube = [
    [-0.5, -0.5, 0],
    [-0.5, 0.5, 0],
    [-0.5, 0.5, 1],
    [0.5, 0.5, 1],
    [0.5, -0.5, 1],
    [-0.5, -0.5, 1],
    [-0.5, 0.5, 1],
    [-0.5, -0.5, 1],
    [-0.5, -0.5, 0],
    [0.5, -0.5, 0],
    [0.5, -0.5, 1],
    [0.5, 0.5, 1],
    [0.5, 0.5, 0],
    [-0.5, 0.5, 0],
    [0.5, -0.5, 0],
    [0.5, 0.5, 0],
    [0.43301270189, 0, 0],
]


x, y, z, yaw, t = calc_path(
    [calc_shape(cube), calc_shape(triangle2)],
    focal_points=[None, focal_point2]
)

N = len(x)


async def run():
    drone = System()
    await drone.connect(system_address="udp://:14551")
    #await drone.connect(system_address="serial:///dev/ttyAMA0:460800")

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
        yaw0 = ahrs.yaw_deg
        print("Current heading: %.2f" % yaw0)
        yaw0 *= np.pi / 180
        break

    print("-- Wait for arming")
    async for armed in drone.telemetry.armed():
        if armed:
            break

    # print("-- Arming")
    # await drone.action.arm()

    await drone.action.takeoff()
    await asyncio.sleep(10)

    print("-- Setting initial setpoint")
    await drone.offboard.set_position_ned(PositionNedYaw(0.0, 0.0, 0.0, 0.0))

    print("-- Starting offboard")
    try:
        await drone.offboard.start()
    except OffboardError as error:
        print(f"Starting offboard mode failed with error code: {error._result.result}")
        print("-- Disarming")
        await drone.action.disarm()
        return

    for i in range(N):
        x[i], y[i] = cos(yaw0) * x[i] - sin(yaw0) * y[i], sin(yaw0) * x[i] + cos(yaw0) * y[i]
        yaw[i] += yaw0 * 180 / np.pi
        yaw[i] %= 360

    wp_counter = 0
    stime = perf_counter()
    rtime = 0

    async for position in drone.telemetry.position_velocity_ned():
        position = position.position.north_m, position.position.east_m, position.position.down_m
        if abs(position[0] - x[wp_counter]) < 1 and abs(position[1] - y[wp_counter]) < 1 and abs(position[2] - z[wp_counter]) < 1:
            rtime = perf_counter() - stime
            while rtime >= t[wp_counter]:
                wp_counter += 1
                if wp_counter == N:
                    break
        else:
            stime = perf_counter() - rtime

        if wp_counter == N:
            break

        await drone.offboard.set_position_ned(PositionNedYaw(x[wp_counter], y[wp_counter], z[wp_counter], yaw[wp_counter]))

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
    asyncio.run(run())
