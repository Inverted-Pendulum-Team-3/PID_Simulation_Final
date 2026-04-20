import math
import time
import pybullet as p
import pybullet_data
import csv
import os

def log_to_csv(filename, value):
    print(f"Trying to open: {repr(filename)}")
    print(f"Directory exists: {os.path.exists(os.path.dirname(filename))}")
    file_exists = os.path.exists(filename)
    with open(filename, 'a', newline='') as f:
        writer = csv.writer(f)
        if not file_exists:
            writer.writerow(['value'])
        writer.writerow([value])
        
def clamp(x, lo, hi):
    return max(lo, min(hi, x))

def deadband(x, db):
    return 0.0 if abs(x) < db else x

def wrap_angle(a):
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


def get_min_z(body_id):
    zmins = []
    aabb_min, _ = p.getAABB(body_id, -1)
    zmins.append(aabb_min[2])
    for link in range(p.getNumJoints(body_id)):
        aabb_min, _ = p.getAABB(body_id, link)
        zmins.append(aabb_min[2])
    return min(zmins)


def place_on_ground(body_id, clearance=0.002):
    pos, quat = p.getBasePositionAndOrientation(body_id)
    min_z = get_min_z(body_id)
    dz = clearance - min_z
    p.resetBasePositionAndOrientation(
        body_id,
        [pos[0], pos[1], pos[2] + dz],
        quat
    )


def main():
    URDF_PATH = "balro4.urdf"
    dt = 1.0 / 240.0

    p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.resetSimulation()
    p.setGravity(0, 0, -9.81)
    p.setTimeStep(dt)
    p.setPhysicsEngineParameter(
        fixedTimeStep=dt,
        numSolverIterations=200,
        enableConeFriction=1,
        erp=0.2
    )
    
    plane = p.loadURDF("plane.urdf")
    p.changeDynamics(
        plane, -1,
        lateralFriction=3.0,
        rollingFriction=0.0,
        spinningFriction=0.0,
        restitution=0.0
    )

    robot = p.loadURDF(
        URDF_PATH,
        basePosition=[0, 0, 0.25],
        baseOrientation=p.getQuaternionFromEuler([0.2, 0.0, 0.0]),
        useFixedBase=False
    )
    name_to_idx = {}
    for j in range(p.getNumJoints(robot)):
        info = p.getJointInfo(robot, j)
        name_to_idx[info[1].decode("utf-8")] = j

    j_left = name_to_idx["joint_wheel1"]
    j_right = name_to_idx["joint_wheel2"]
    MASS_SCALE = 1

    base_mass = p.getDynamicsInfo(robot, -1)[0]
    if base_mass > 0:
        p.changeDynamics(
            robot, -1,
            mass=base_mass * MASS_SCALE,
            linearDamping=0.0,
            angularDamping=0.0,
            restitution=0.0
        )

    for link in range(p.getNumJoints(robot)):
        mass = p.getDynamicsInfo(robot, link)[0]
        kwargs = {
            "linearDamping": 0.0,
            "angularDamping": 0.0,
            "restitution": 0.0,
            
        }
        if mass > 0:
            kwargs["mass"] = mass * MASS_SCALE

        if link in [j_left, j_right]:
            kwargs.update({
                "lateralFriction": 3.0,
                "rollingFriction": 0.0,
                "spinningFriction": 0.0,
                "jointDamping": 0.05
            })

        p.changeDynamics(robot, link, **kwargs)

    place_on_ground(robot, clearance=0.002)

    p.resetJointState(robot, j_left, targetValue=0.0, targetVelocity=0.0)
    p.resetJointState(robot, j_right, targetValue=0.0, targetVelocity=0.0)

    p.getKeyboardEvents()

    FORCE_MAX = 1.56
    WHEEL_CMD_MAX = 22.8

    KP_PITCH = 3000.0
    KD_PITCH = 0.0
    KI_PITCH = 500.0
    I_PITCH_MAX = 2
    pitch_int = 0.0
    K_WHEEL_AVG = 0

    KP_ROLL = 20.0
    KD_ROLL = 4.0
    ROLL_TORQUE_MAX = 12.0

    FWD_MAX = 6.0
    FWD_ACCEL = 10.0
    FWD_DECAY = 0.20

    YAW_MAX = 4.0
    YAW_ACCEL = 30.0

    K_YAW_HOLD = 2.5
    K_YAW_RATE = 0.35
    YAW_CMD_MAX = 2.0
    heading_target = None

    drive_fwd = 0.0
    drive_yaw = 0.0

    PITCH_TIP_LIMIT = math.radians(70)
    ROLL_TIP_LIMIT = math.radians(75)

    print("balro4 with heading hold")
    print("UP/DOWN = forward/back, LEFT/RIGHT = turn, SPACE = zero commands")

    last_print = time.time()

    try:
        while True:
            if p.isConnected() == 0:
                break

            keys = p.getKeyboardEvents()

            up = (p.B3G_UP_ARROW in keys) and (keys[p.B3G_UP_ARROW] & p.KEY_IS_DOWN)
            down = (p.B3G_DOWN_ARROW in keys) and (keys[p.B3G_DOWN_ARROW] & p.KEY_IS_DOWN)
            left = (p.B3G_LEFT_ARROW in keys) and (keys[p.B3G_LEFT_ARROW] & p.KEY_IS_DOWN)
            right = (p.B3G_RIGHT_ARROW in keys) and (keys[p.B3G_RIGHT_ARROW] & p.KEY_IS_DOWN)

            if p.B3G_SPACE in keys and (keys[p.B3G_SPACE] & p.KEY_WAS_TRIGGERED):
                drive_fwd = 0.0
                drive_yaw = 0.0
                heading_target = None

            if up and not down:
                drive_fwd += FWD_ACCEL * dt
            elif down and not up:
                drive_fwd -= FWD_ACCEL * dt
            else:
                drive_fwd *= math.exp(-dt / FWD_DECAY)
                #drive_fwd = 0.0
            if left and not right:
                if heading_target is None:
                    heading_target = 0.0
                drive_yaw += YAW_ACCEL * dt
            elif right and not left:
                if heading_target is None:
                    heading_target = 0.0
                drive_yaw -= YAW_ACCEL * dt
            else:
                drive_yaw = 0.0

            drive_fwd = clamp(drive_fwd, -FWD_MAX, FWD_MAX)
            drive_yaw = clamp(drive_yaw, -YAW_MAX, YAW_MAX)

            _, quat = p.getBasePositionAndOrientation(robot)
            pitch, roll, yaw = p.getEulerFromQuaternion(quat)
            _, ang_vel_world = p.getBaseVelocity(robot)
            
            # Rotate world-frame angular velocity into body frame
            rot_matrix = p.getMatrixFromQuaternion(quat)  # 3x3 as flat list
            R = [rot_matrix[0:3], rot_matrix[3:6], rot_matrix[6:9]]

            # Transpose of R converts world -> body frame
            ang_vel = [
                R[0][0]*ang_vel_world[0] + R[1][0]*ang_vel_world[1] + R[2][0]*ang_vel_world[2],
                R[0][1]*ang_vel_world[0] + R[1][1]*ang_vel_world[1] + R[2][1]*ang_vel_world[2],
                R[0][2]*ang_vel_world[0] + R[1][2]*ang_vel_world[1] + R[2][2]*ang_vel_world[2],
            ]
            roll_dot = ang_vel[1]
            pitch_dot = ang_vel[0]
            yaw_rate = ang_vel[2]
            

            wheel_left_vel = p.getJointState(robot, j_left)[1]
            wheel_right_vel = p.getJointState(robot, j_right)[1]
            wheel_avg = clamp(0.5 * (wheel_left_vel + wheel_right_vel), -WHEEL_CMD_MAX, WHEEL_CMD_MAX)
            
            if abs(pitch) > PITCH_TIP_LIMIT or abs(roll) > ROLL_TIP_LIMIT:
                drive_fwd = 0.0
                drive_yaw = 0.0
                pitch_int = 0.0
                heading_target = None

                p.setJointMotorControl2(robot, j_left, p.VELOCITY_CONTROL, targetVelocity=0.0, force=0.0)
                p.setJointMotorControl2(robot, j_right, p.VELOCITY_CONTROL, targetVelocity=0.0, force=0.0)

                p.stepSimulation()
                time.sleep(dt)
                continue

            pitch_setpoint = clamp(-drive_fwd * 0.012, -0.05, 0.05)
            pitch_error = pitch - pitch_setpoint
            pitch_int = clamp(pitch_int + pitch_error * dt, -I_PITCH_MAX, I_PITCH_MAX)

            wheel_balance_cmd = (
                KP_PITCH * pitch_error
                + KD_PITCH * pitch_dot
                + KI_PITCH * pitch_int
                + K_WHEEL_AVG * wheel_avg
            )
            wheel_balance_cmd = clamp(wheel_balance_cmd, -WHEEL_CMD_MAX, WHEEL_CMD_MAX)

            wheel_avg_cmd = wheel_balance_cmd

            if left or right:
                yaw_cmd = drive_yaw
                heading_target = yaw
            else:
                if heading_target is None:
                    heading_target = yaw
                yaw_error = wrap_angle(yaw - heading_target)
                yaw_cmd = clamp(
                    -(K_YAW_HOLD * yaw_error + K_YAW_RATE * yaw_rate),
                    -YAW_CMD_MAX,
                    YAW_CMD_MAX
                )

            left_cmd = clamp(wheel_avg_cmd - yaw_cmd, -WHEEL_CMD_MAX, WHEEL_CMD_MAX)
            right_cmd = clamp(wheel_avg_cmd + yaw_cmd, -WHEEL_CMD_MAX, WHEEL_CMD_MAX)

            roll_torque = -(KP_ROLL * roll + KD_ROLL * roll_dot)
            roll_torque = clamp(roll_torque, -ROLL_TORQUE_MAX, ROLL_TORQUE_MAX)

            #p.applyExternalTorque(
                #objectUniqueId=robot,
                #linkIndex=-1,
                #torqueObj=[roll_torque, 0.0, 0.0],
                #flags=p.LINK_FRAME
            #)

            p.setJointMotorControl2(robot, j_left, p.VELOCITY_CONTROL, targetVelocity=left_cmd, force=FORCE_MAX)
            p.setJointMotorControl2(robot, j_right, p.VELOCITY_CONTROL, targetVelocity=right_cmd, force=FORCE_MAX)

            if time.time() - last_print > 0.25:
                last_print = time.time()
                #print(
                 #   f"roll={roll:+.3f} pitch={pitch:+.3f} yaw={yaw:+.3f} "
                  #  f"fwd={drive_fwd:+.2f} yaw_cmd={yaw_cmd:+.2f} "
                   # f"cmd=({left_cmd:+.2f},{right_cmd:+.2f})"
                #)
                print(f"pitch={pitch:+.4f} wheel_balance_cmd={wheel_balance_cmd:+.4f} left_cmd={left_cmd:+.4f} wheel_avg={wheel_avg:+.4f}")
            p.stepSimulation()
            #log_to_csv("C:/Users/dylan/Desktop/kp_5000.csv", pitch)
            p.addUserDebugText("Pitch: {:.2f}".format(pitch * 57), [0, 0, 1.2], textColorRGB=[1,0,0], textSize=1.5, lifeTime=0.041)
            time.sleep(dt)

    except KeyboardInterrupt:
        pass

    if p.isConnected():
        p.disconnect()


if __name__ == "__main__":
    main()
    
