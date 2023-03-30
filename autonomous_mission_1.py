import asyncio

from mavsdk import System
from mavsdk.offboard import (OffboardError, PositionNedYaw)
from mavsdk.action import ActionError
from time import perf_counter
import numpy as np
from dataclasses import dataclass

from math import sqrt, cos, sin, atan2, copysign
import time

@dataclass
class CameraIntrinsics:
    hfov: float
    vfov: float
    w_px: int
    h_px: int
    fx: float
    fy: float

def calculate_gain(error, max_error_abs):
    gain = copysign(1, error)*(error/max_error_abs)**2
    return gain

def calculate_step(camera_model, xc, yc, step_limit = 0.01):
    step_x_m = 0 
    step_y_m = 0 
    x_error_px = xc - camera_model.w_px/2
    y_error_px = -yc + camera_model.h_px/2
    step_x_m = step_limit * calculate_gain(x_error_px, camera_model.w_px/2)
    step_y_m = step_limit * calculate_gain(y_error_px, camera_model.h_px/2)
    return step_x_m, step_y_m

def calculate_next_position(position_ned, yaw_deg, step_right, step_front):
    a = np.deg2rad(yaw_deg) 
    next_position = [0,0,0]
    delta_position = np.matmul(np.matrix([[cos(a), -sin(a)],
                                          [sin(a), cos(a)]]),
                                np.matrix([[step_front],
                                           [step_right]]))
    next_position[0] = position_ned[0] + delta_position.item(0,0)
    next_position[1] = position_ned[1] + delta_position.item(1,0)
    next_position[2] = position_ned[2]
    print(next_position)
    return next_position

position = [0,0,5]
camera_model = CameraIntrinsics(62.2, 48.8, 640, 480, 250, 250) #fx and fy are unknown
step_right, step_front = calculate_step(camera_model, 320, 120)
print(step_front, step_right)
calculate_next_position(position, 120, step_right, step_front)

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


    async for position in drone.telemetry.position_velocity_ned():
        position = position.position.north_m, position.position.east_m, position.position.down_m
        


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
