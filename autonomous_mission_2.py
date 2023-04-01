import asyncio

from mavsdk import System
from mavsdk.offboard import (OffboardError, PositionNedYaw)
from mavsdk.action import ActionError
from time import perf_counter
import numpy as np

from math import sqrt, cos, sin, atan2
import time

WAIT_TIME = 0.3
ROUNDS = 5
LENGTH = (4.5, 4.5, 4.5)

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


def calc_shape(path, length=LENGTH, offset=(0, 0, 1), rvrs=(1, 1, -1)):
    return [[rvrs[k] * (p[k] * length[k] + offset[k]) for k in range(3)] for p in path]

def circulate_shape(shape, length=LENGTH, radius=0.14, dlc=0.02, drp=np.pi/4):
    '''
    dlr: The length of change of position for the center of rotation in each step
    dlc: The rotation angle applied in each step
    If there is a high risk of failure use smaller values for the radius.
    '''
    center_point = [0,0,0]
    focals = []
    shape_ = []
    rotation = 0
    for target_point in shape:
        tx, ty, tz = target_point
        cx, cy, cz = center_point
        heading_angle = atan2(tx-cx, ty-cy) #Not in a formal way
        while True:
            if (cx - tx)**2 + (cy - ty)**2 + (cz - tz)**2 <= dlc**2:
                target_point = center_point
                break
            nx = cx+radius*sin(rotation)
            ny = cy+radius*cos(rotation)
            nz = cz 
            new_point = [nx, ny, nz]
            shape_.append(new_point)

            rotation += drp
            cx += dlc*sin(heading_angle)
            cy += dlc*cos(heading_angle)
            cz = cz 
            center_point = [cx, cy, cz]
            focals.append(center_point)
    return shape_, focals

def mission_decomposition(mission):
    item_list = []     
    for point in mission:
        item = [point]
        item_list.append(item)
    return item_list

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

mission_2_path = [
    [1.1, 0,  0], #Radial 1 start
    [1.1 + 0.2*sin(5*np.pi/6), 0.2 + 0.2*cos(5*np.pi/6), 0],
    [1.1 + 0.2*sin(4*np.pi/6), 0.2 + 0.2*cos(4*np.pi/6), 0],
    [1.3, 0.2, 0], #Radial 1 end
    [1.3, 0.6, 0],
    [0.5, 0.6, 0], #Radial 2 start
    [0.5 + 0.4*sin(7*np.pi/6), 1 + 0.4*cos(7*np.pi/6), 0],
    [0.5 + 0.4*sin(8*np.pi/6), 1 + 0.4*cos(8*np.pi/6), 0],
    [0.5 + 0.4*sin(9*np.pi/6), 1 + 0.4*cos(9*np.pi/6), 0], #[0.1, 1, 0],
    [0.5 + 0.4*sin(10*np.pi/6), 1 + 0.4*cos(10*np.pi/6), 0],
    [0.5 + 0.4*sin(11*np.pi/6), 1 + 0.4*cos(11*np.pi/6), 0],
    [0.5, 1.4, 0], #Radial 2 end
    [1.4, 1.4, 0]
]

mission_shape, focals = circulate_shape(mission_2_path)
mission_shape = mission_decomposition(mission_shape)

x, y, z, yaw, t = calc_path(
    [calc_shape(mission_shape[i]) for i in range(len(mission_shape))],
    focal_points=calc_shape([focals[i] for i in range(len(focals))])
)

N = len(x)


async def run():
    drone = System()
    #await drone.connect(system_address="udp://:14540")
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
        if abs(position[0] - x[wp_counter]) < 1 and abs(position[1] - y[wp_counter]) < 1 and abs(position[2] - z[wp_counter]) < 1: #TODO: Lower threshold ?
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
