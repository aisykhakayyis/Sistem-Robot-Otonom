# %%
import time
import math
from coppeliasim_zmqremoteapi_client import RemoteAPIClient

# CONNECT
client = RemoteAPIClient()
sim = client.require('sim')

# START
sim.startSimulation()
print("Simulation Started")

# HANDLE OBJECT
robot = sim.getObject('/Robot_Pemain')
leftMotor = sim.getObject('/Robot_Pemain/leftMotor')
rightMotor = sim.getObject('/Robot_Pemain/rightMotor')

defender = sim.getObject('/Robot_Lawan_01')
leftMotor_d = sim.getObject('/Robot_Lawan_01/leftMotor')
rightMotor_d = sim.getObject('/Robot_Lawan_01/rightMotor')

goalkeeper = sim.getObject('/Robot_Lawan_02')
leftMotor_gk = sim.getObject('/Robot_Lawan_02/leftMotor')
rightMotor_gk = sim.getObject('/Robot_Lawan_02/rightMotor')

ball = sim.getObject('/Bola_Merah')

# PARAMETER
rw = 0.195/2
rb = 0.381/2

# posisi gawang
goal_enemy = [5, 0]
goal_self = [-5, 0]

# 🔥 TIMER START
start_time = time.time()

try:
    while True:

        # =========================
        # ⏱️ TIMEOUT CHECK (15 DETIK)
        # =========================
        elapsed_time = time.time() - start_time

        if elapsed_time >= 15:
            print("⏱️ TIME OUT - Tidak ada goal")

            sim.setJointTargetVelocity(leftMotor, 0)
            sim.setJointTargetVelocity(rightMotor, 0)
            sim.setJointTargetVelocity(leftMotor_d, 0)
            sim.setJointTargetVelocity(rightMotor_d, 0)
            sim.setJointTargetVelocity(leftMotor_gk, 0)
            sim.setJointTargetVelocity(rightMotor_gk, 0)

            sim.stopSimulation()
            break

        # =========================
        # 🥅 CEK GOL (AREA)
        # =========================
        ball_pos = sim.getObjectPosition(ball, -1)

        if (5 < ball_pos[0] < 5.5) and (-0.4 < ball_pos[1] < 0.4):
            print("⚽ GOAL !!!")

            sim.setJointTargetVelocity(leftMotor, 0)
            sim.setJointTargetVelocity(rightMotor, 0)
            sim.setJointTargetVelocity(leftMotor_d, 0)
            sim.setJointTargetVelocity(rightMotor_d, 0)
            sim.setJointTargetVelocity(leftMotor_gk, 0)
            sim.setJointTargetVelocity(rightMotor_gk, 0)

            sim.stopSimulation()
            break

        # =========================
        # ⚽ STRIKER
        # =========================
        robot_pos = sim.getObjectPosition(robot, -1)
        robot_ori = sim.getObjectOrientation(robot, -1)

        dx = ball_pos[0] - robot_pos[0]
        dy = ball_pos[1] - robot_pos[1]
        dist = math.sqrt(dx**2 + dy**2)
        theta = robot_ori[2]

        if dist > 0.3:
            target_angle = math.atan2(dy, dx)
        else:
            dxg = goal_enemy[0] - robot_pos[0]
            dyg = goal_enemy[1] - robot_pos[1]
            target_angle = math.atan2(dyg, dxg)

        error = target_angle - theta
        error = math.atan2(math.sin(error), math.cos(error))

        v = 0.4
        w = 2.0 * error

        v = max(min(v, 1.0), -1.0)
        w = max(min(w, 2.0), -2.0)

        wr = (v + rb*w)/rw
        wl = (v - rb*w)/rw

        sim.setJointTargetVelocity(rightMotor, wr)
        sim.setJointTargetVelocity(leftMotor, wl)

        # =========================
        # 🛡️ DEFENDER
        # =========================
        def_pos = sim.getObjectPosition(defender, -1)
        def_ori = sim.getObjectOrientation(defender, -1)

        theta_p = robot_ori[2]

        d = 0.35
        target_x = robot_pos[0] + d * math.cos(theta_p)
        target_y = robot_pos[1] + d * math.sin(theta_p)

        dist_player = math.sqrt(
            (robot_pos[0] - def_pos[0])**2 +
            (robot_pos[1] - def_pos[1])**2
        )

        dx = target_x - def_pos[0]
        dy = target_y - def_pos[1]
        dist = math.sqrt(dx**2 + dy**2)
        theta = def_ori[2]

        safe_distance = 0.35

        if dist_player < safe_distance:
            v_d = 0.0
            w_d = 0.0
        else:
            target_angle = math.atan2(dy, dx)
            error = target_angle - theta
            error = math.atan2(math.sin(error), math.cos(error))

            v_d = 0.3
            w_d = 1.2 * error

            if dist < 0.2:
                v_d *= 0.5
                w_d *= 0.5

        v_d = max(min(v_d, 0.4), -0.4)
        w_d = max(min(w_d, 1.0), -1.0)

        wr_d = (v_d + rb*w_d)/rw
        wl_d = (v_d - rb*w_d)/rw

        sim.setJointTargetVelocity(rightMotor_d, wr_d)
        sim.setJointTargetVelocity(leftMotor_d, wl_d)

        # =========================
        # 🧤 GOALKEEPER
        # =========================
        gk_pos = sim.getObjectPosition(goalkeeper, -1)
        gk_ori = sim.getObjectOrientation(goalkeeper, -1)

        target_x = goal_self[0] + 0.3
        target_y = ball_pos[1]

        target_y = max(min(target_y, 0.5), -0.5)

        dy = target_y - gk_pos[1]

        theta = gk_ori[2]
        target_angle = math.atan2(dy, 0.0001)
        error = target_angle - theta
        error = math.atan2(math.sin(error), math.cos(error))

        v_gk = 0.3
        w_gk = 1.2 * error

        if abs(dy) < 0.05:
            v_gk = 0
            w_gk = 0

        v_gk = max(min(v_gk, 0.3), -0.3)
        w_gk = max(min(w_gk, 1.0), -1.0)

        wr_gk = (v_gk + rb*w_gk)/rw
        wl_gk = (v_gk - rb*w_gk)/rw

        sim.setJointTargetVelocity(rightMotor_gk, wr_gk)
        sim.setJointTargetVelocity(leftMotor_gk, wl_gk)

        time.sleep(0.05)

except KeyboardInterrupt:
    sim.stopSimulation()
    print("Simulation Stopped")