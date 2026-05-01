import time
import math
import numpy as np
from coppeliasim_zmqremoteapi_client import RemoteAPIClient

# ======================
# CONNECT
# ======================
client = RemoteAPIClient()
sim = client.require('sim')

sim.setStepping(False)
sim.startSimulation()

# ======================
# HANDLE OBJECT
# ======================
striker = sim.getObject('/Robot_Pemain')

lawan1 = sim.getObject('/Robot_Lawan_01')
lawan2 = sim.getObject('/Robot_Lawan_02')

ball = sim.getObject('/Bola_Merah')
goal = sim.getObject('/Gawang_Putih')

# motors striker
lm_s = sim.getObject('/Robot_Pemain/leftMotor')
rm_s = sim.getObject('/Robot_Pemain/rightMotor')

# motors lawan
lm_l1 = sim.getObject('/Robot_Lawan_01/leftMotor')
rm_l1 = sim.getObject('/Robot_Lawan_01/rightMotor')

lm_l2 = sim.getObject('/Robot_Lawan_02/leftMotor')
rm_l2 = sim.getObject('/Robot_Lawan_02/rightMotor')

def world_to_robot(robot_pos, robot_theta, target_pos):
    dx = target_pos[0] - robot_pos[0]
    dy = target_pos[1] - robot_pos[1]

    R = np.array([
        [ math.cos(robot_theta),  math.sin(robot_theta)],
        [-math.sin(robot_theta),  math.cos(robot_theta)]
    ])

    rel = R @ np.array([dx, dy])
    return rel[0], rel[1]

# ======================
# STRIKER
# ======================
def striker_logic():

    robot_pos = np.array(sim.getObjectPosition(striker, -1))
    ball_pos = np.array(sim.getObjectPosition(ball, -1))
    goal_pos = np.array(sim.getObjectPosition(goal, -1))

    theta = sim.getObjectOrientation(striker, -1)[2]

    x_rel, y_rel = world_to_robot(robot_pos, theta, ball_pos)

    base = 4.0
    k_turn = 5.0

    if x_rel > 0:
        left = base - k_turn * y_rel
        right = base + k_turn * y_rel
    else:
        left = -2
        right = 2

    sim.setJointTargetVelocity(lm_s, left)
    sim.setJointTargetVelocity(rm_s, right)

    # ======================
    # DRIBBLE
    # ======================
    dist_ball = np.linalg.norm(ball_pos - robot_pos)

    if dist_ball < 0.3:

        xg_rel, yg_rel = world_to_robot(robot_pos, theta, goal_pos)

        base = 3.0
        k_turn = 4.0

        left = base - k_turn * yg_rel
        right = base + k_turn * yg_rel

        sim.setJointTargetVelocity(lm_s, left)
        sim.setJointTargetVelocity(rm_s, right)

# ======================
# GOALKEEPER 
# ======================
def goalkeeper_vertical(robot, lm, rm):

    robot_pos = np.array(sim.getObjectPosition(robot, -1))
    ball_pos = np.array(sim.getObjectPosition(ball, -1))
    theta = sim.getObjectOrientation(robot, -1)[2]

    # ======================
    # BATAS GAWANG
    # ======================
    min_y = -0.6
    max_y = 0.6

    target_y = max(min_y, min(max_y, ball_pos[1]))
    error_y = target_y - robot_pos[1]

    # ======================
    # GERAK VERTIKAL
    # ======================
    k_move = 8
    forward = k_move * error_y * math.sin(theta)

    # ======================
    # JAGA ORIENTASI
    # ======================
    desired_theta = 0  # menghadap lapangan

    error_theta = desired_theta - theta
    error_theta = math.atan2(math.sin(error_theta), math.cos(error_theta))

    k_turn = 3.0
    turn = k_turn * error_theta

    left = forward - turn
    right = forward + turn

    # ======================
    # LIMIT SPEED
    # ======================
    max_speed = 4.0
    left = max(-max_speed, min(max_speed, left))
    right = max(-max_speed, min(max_speed, right))

    sim.setJointTargetVelocity(lm, left)
    sim.setJointTargetVelocity(rm, right)

# ======================
# MAIN LOOP
# ======================
try:
    while True:

        striker_logic()

        goalkeeper_vertical(lawan1, lm_l1, rm_l1)
        goalkeeper_vertical(lawan2, lm_l2, rm_l2)

        time.sleep(0.05)

except KeyboardInterrupt:
    print("Stopped")

sim.stopSimulation()