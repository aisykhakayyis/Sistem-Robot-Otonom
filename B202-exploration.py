from coppeliasim_zmqremoteapi_client import RemoteAPIClient
import numpy as np
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
import math
import time
import os

# ============================================================
# CONFIGURATION
# ============================================================

ROBOT = '/PioneerP3DX'
LEFT_MOTOR = '/PioneerP3DX/leftMotor'
RIGHT_MOTOR = '/PioneerP3DX/rightMotor'

SENSOR_PATHS = [
    '/PioneerP3DX/ultrasonicSensor[1]',  # front-left
    '/PioneerP3DX/ultrasonicSensor[3]',  # front
    '/PioneerP3DX/ultrasonicSensor[4]',  # front-right
    '/PioneerP3DX/ultrasonicSensor[6]',  # right
]

MAP_SIZE = 250
MAP_RESOLUTION = 0.1
MAP_CENTER = MAP_SIZE // 2

WHEEL_BASE = 0.38

# FAST
BASE_SPEED = 10.0
MAX_SPEED = 15.0
LOOKAHEAD = 2.5

# obstacle influence
SAFE_DISTANCE = 0.45
EMERGENCY_DISTANCE = 0.15

# exploration
WORKSPACE_WIDTH = 6.0
WORKSPACE_HEIGHT = 5.0
LANE_SPACING = 2.0

# timing
DT = 0.01
MAX_RUNTIME = 120


# ============================================================
# UTILITY
# ============================================================

def normalize_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def world_to_grid(x, y):
    gx = int(x / MAP_RESOLUTION) + MAP_CENTER
    gy = int(y / MAP_RESOLUTION) + MAP_CENTER
    return gx, gy


def in_bounds(gx, gy):
    return 0 <= gx < MAP_SIZE and 0 <= gy < MAP_SIZE


# ============================================================
# PATH
# ============================================================

def generate_lawnmower_path(start_x, start_y):
    path = []

    y = start_y
    direction = 1

    while y <= start_y + WORKSPACE_HEIGHT:
        if direction == 1:
            path.append((start_x, y))
            path.append((start_x + WORKSPACE_WIDTH, y))
        else:
            path.append((start_x + WORKSPACE_WIDTH, y))
            path.append((start_x, y))

        y += LANE_SPACING
        direction *= -1

    return path


def find_lookahead(robot_x, robot_y, path, current_idx):
    for i in range(current_idx, len(path)):
        px, py = path[i]
        dist = math.hypot(px - robot_x, py - robot_y)

        if dist >= LOOKAHEAD:
            return px, py, i

    return path[-1][0], path[-1][1], len(path) - 1


# ============================================================
# PURE PURSUIT
# ============================================================

def pure_pursuit(robot_x, robot_y, robot_theta, tx, ty):
    dx = tx - robot_x
    dy = ty - robot_y

    target_heading = math.atan2(dy, dx)
    heading_error = normalize_angle(target_heading - robot_theta)

    curvature = (2 * math.sin(heading_error)) / LOOKAHEAD
    omega = BASE_SPEED * curvature

    left = BASE_SPEED - (omega * WHEEL_BASE / 2)
    right = BASE_SPEED + (omega * WHEEL_BASE / 2)

    return left, right


# ============================================================
# SENSOR
# ============================================================

def read_sensor(sim, sensor):
    result, distance, detectedPoint, detectedObject, detectedNormal = sim.readProximitySensor(sensor)

    if result:
        dist = math.sqrt(
            detectedPoint[0]**2 +
            detectedPoint[1]**2 +
            detectedPoint[2]**2
        )
        return True, dist, detectedPoint

    return False, 3.0, None


# ============================================================
# SENSOR-BASED STEERING (SMOOTH AVOIDANCE)
# ============================================================

def obstacle_steering(sensor_distances, left_cmd, right_cmd):
    front_left = sensor_distances[0]
    front = sensor_distances[1]
    front_right = sensor_distances[2]
    right = sensor_distances[3]

    # emergency
    if front < EMERGENCY_DISTANCE:
        return -5.0, 8.0

    steering_bias = 0.0

    if front_left < SAFE_DISTANCE:
        steering_bias += 4.0

    if front_right < SAFE_DISTANCE:
        steering_bias -= 4.0

    if right < SAFE_DISTANCE:
        steering_bias += 2.0

    left_cmd += steering_bias
    right_cmd -= steering_bias

    left_cmd = np.clip(left_cmd, -MAX_SPEED, MAX_SPEED)
    right_cmd = np.clip(right_cmd, -MAX_SPEED, MAX_SPEED)

    return left_cmd, right_cmd


# ============================================================
# MAPPING
# ============================================================

def update_map(sim, occupancy, sensors):
    for sensor in sensors:
        detected, dist, point = read_sensor(sim, sensor)

        if detected:
            sensor_pos = sim.getObjectPosition(sensor, -1)
            sensor_ori = sim.getObjectOrientation(sensor, -1)

            local_x = point[0]
            local_y = point[1]

            angle = sensor_ori[2]

            global_x = sensor_pos[0] + local_x * math.cos(angle) - local_y * math.sin(angle)
            global_y = sensor_pos[1] + local_x * math.sin(angle) + local_y * math.cos(angle)

            gx, gy = world_to_grid(global_x, global_y)

            if in_bounds(gx, gy):
                occupancy[gy, gx] = 1


# ============================================================
# SAVE MAP
# ============================================================

def save_map(occupancy):
    np.save("occupancy_map.npy", occupancy)

    plt.figure(figsize=(8, 8))
    plt.imshow(occupancy, cmap='gray', origin='lower')
    plt.title("Occupancy Grid Map")
    plt.xlabel("X")
    plt.ylabel("Y")
    plt.savefig("occupancy_map.png")
    plt.show()


# ============================================================
# MAIN
# ============================================================

def main():
    client = RemoteAPIClient()
    sim = client.require('sim')

    robot = sim.getObject(ROBOT)
    left_motor = sim.getObject(LEFT_MOTOR)
    right_motor = sim.getObject(RIGHT_MOTOR)

    sensors = [sim.getObject(path) for path in SENSOR_PATHS]

    occupancy = np.zeros((MAP_SIZE, MAP_SIZE))

    sim.startSimulation()
    time.sleep(1)

    print("Simulation started")
    print("Working directory:", os.getcwd())

    start_time = time.time()
    last_printed_second = -1

    start_pos = sim.getObjectPosition(robot, -1)
    path = generate_lawnmower_path(start_pos[0], start_pos[1])

    waypoint_idx = 0

    try:
        while True:
            elapsed = time.time() - start_time

            current_second = int(elapsed)
            if current_second != last_printed_second:
                print(f"Simulation runtime: {current_second} seconds")
                last_printed_second = current_second

            if elapsed >= MAX_RUNTIME:
                print("Maximum runtime reached (120 seconds)")
                break

            if sim.getSimulationState() == sim.simulation_stopped:
                break

            # pose
            pos = sim.getObjectPosition(robot, -1)
            ori = sim.getObjectOrientation(robot, -1)

            robot_x = pos[0]
            robot_y = pos[1]
            robot_theta = ori[2]

            # sensors
            sensor_distances = []
            for sensor in sensors:
                detected, dist, point = read_sensor(sim, sensor)
                sensor_distances.append(dist)

            # mapping
            update_map(sim, occupancy, sensors)

            # pure pursuit
            target_x, target_y, waypoint_idx = find_lookahead(
                robot_x,
                robot_y,
                path,
                waypoint_idx
            )

            left_speed, right_speed = pure_pursuit(
                robot_x,
                robot_y,
                robot_theta,
                target_x,
                target_y
            )

            # smooth avoidance
            left_speed, right_speed = obstacle_steering(
                sensor_distances,
                left_speed,
                right_speed
            )

            sim.setJointTargetVelocity(left_motor, left_speed)
            sim.setJointTargetVelocity(right_motor, right_speed)

            time.sleep(DT)

    except KeyboardInterrupt:
        print("Stopped manually")

    finally:
        sim.setJointTargetVelocity(left_motor, 0)
        sim.setJointTargetVelocity(right_motor, 0)

        sim.stopSimulation()
        print("Simulation stopped")

        save_map(occupancy)


if __name__ == "__main__":
    main()