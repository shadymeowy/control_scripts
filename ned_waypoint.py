import asyncio
from mavsdk import System
from pymap3d import *
from math import sqrt, cos, sin, pi

def calc_shape(path, length=(5, 5, 5), offset=(0, 0, 1), rvrs=(1, 1, -1)):
    return [[rvrs[k] * (p[k] * length[k] + offset[k]) for k in range(3)] for p in path]

points = [
    [0, 0, 0],
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
    [0.6123724356957945, -0.6123724356957946, 0],

    [0.3535533905932738, 0.35355339059327373, 0],
    [-0.3535533905932738, -0.35355339059327373, 0],
    [-0.6123724356957945, 0.6123724356957946, 0],

    [0.3535533905932738, 0.35355339059327373, 0],
    [-0.3535533905932738, -0.35355339059327373, 0],
    [-0.6123724356957945, 0.6123724356957946, 0],

    [0.3535533905932738, 0.35355339059327373, 0],
    [-0.3535533905932738, -0.35355339059327373, 0],
    [-0.6123724356957945, 0.6123724356957946, 0],

    [0.3535533905932738, 0.35355339059327373, 0],
    [0.6123724356957945, -0.6123724356957946, 0],
]

points = calc_shape(points)


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
    async for position in drone.telemetry.position():
        pg0 = position.latitude_deg, position.longitude_deg, position.absolute_altitude_m
        print(pg0)
        break

    print("Fetching current heading...")
    async for ahrs in drone.telemetry.attitude_euler():
        yaw0 = ahrs.yaw_deg
        print("Current heading: %.2f" % yaw0)
        yaw0 *= pi / 180
        break

    print("-- Arming")
    await drone.action.arm()

    print("-- Taking off")
    await drone.action.takeoff()

    await asyncio.sleep(2)

    for point in points:
        point[0], point[1] = cos(yaw0) * point[0] - sin(yaw0) * point[1], sin(yaw0) * point[0] + cos(yaw0) * point[1]

    for point in points:
        target = ned2geodetic(*point, *pg0)
        print("-- Going to: {}".format(target))

        await drone.action.goto_location(*target, 0)
        async for position in drone.telemetry.position():
            position = geodetic2ned(position.latitude_deg, position.longitude_deg, position.absolute_altitude_m, *pg0)
            if abs(position[0] - point[0]) < 0.4 and abs(position[1] - point[1]) < 0.4 and abs(position[2] - point[2]) < 0.4:
                print("-- Reached target")
                break
            print("\f-- Position: ({})".format(position))
        await asyncio.sleep(0.2)

    print("-- Landing")
    await drone.action.land()


if __name__ == "__main__":
    asyncio.run(run())
