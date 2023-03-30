import asyncio
from math import sqrt

from mavsdk import System
from mavsdk.offboard import (OffboardError, PositionNedYaw)
from mavsdk.action import ActionError
from time import perf_counter
import numpy as np

from math import sqrt, cos, sin

WAIT_TIME = 2
ROUNDS = 2


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


def calc_shape(path, length=(3, 3, 3), offset=(0, 0, 1), rvrs=(1, 1, -1)):
    return [[rvrs[k] * (p[k] * length[k] + offset[k]) for k in range(3)] for p in path]


triangle = [
    *(
        [
            [1.3535533905932737, 1.3535533905932737, 0],
            [0.6464466094067263, 0.6464466094067263, 0],
            [0.38762756430420553, 1.6123724356957947, 0]
        ] * ROUNDS
    ),

    [1.3535533905932737, 1.3535533905932737, 0],
    [1.6123724356957945, 0.3876275643042054, 0]
]
focal_point = calc_shape([[0.7958758547680685, 1.2041241452319316, 0]])[0]

cube = [
    [0.5, 0.5, 0],
    [0.5, 1.5, 0],
    [0.5, 1.5, 1],
    [1.5, 1.5, 1],
    [1.5, 0.5, 1],
    [0.5, 0.5, 1],
    [0.5, 1.5, 1],
    [0.5, 0.5, 1],
    [0.5, 0.5, 0],
    [1.5, 0.5, 0],
    [1.5, 0.5, 1],
    [1.5, 1.5, 1],
    [1.5, 1.5, 0],
    [0.5, 1.5, 0],
    [1.5, 0.5, 0],
    [1.5, 1.5, 0],
    [1.6123724356957945, 0.3876275643042054, 0]
]

x, y, z, yaw, t = calc_path(
    [calc_shape(cube), calc_shape(triangle)],
    focal_points=[None, focal_point]
)

N = len(x)


async def run():
    drone = System()
    #await drone.connect(system_address="udp://:14540")
    print("hi")
    await drone.connect(system_address="serial:///dev/ttyACM0:921600")

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

    async for is_armed in drone.telemetry.armed():
        if is_armed:
            print('-- The vehicle is already armed.')
            break
        else:
            print("-- Arming")
            try:
                await drone.action.arm()
                break
            except ActionError as error:
                print(f"Arming failed with error code:{error._result.result}")
                await asyncio.sleep(1)

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
