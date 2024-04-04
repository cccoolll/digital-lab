import pybullet as p
import pybullet_data
import time
import math
from scipy.spatial.transform import Rotation as R

def initialize_camera():
    camera_distance = 3  # Distance from the camera to the look-at point
    camera_yaw = 50  # Rotation around the vertical axis, in degrees
    camera_pitch = -30  # Pitch angle. Positive values point the camera down
    camera_target_position = [0, 0, 0.5]  # XYZ coordinates that the camera is looking at
    
    p.resetDebugVisualizerCamera(camera_distance, camera_yaw, camera_pitch, camera_target_position)

def initialize_simulation():
    p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, 0)
    flags = p.URDF_USE_SELF_COLLISION
    lab_id = p.loadURDF("SimulatedLab/urdf/SimulatedLab.urdf", [-1, 1, 0], useFixedBase=True, flags=flags)
    initialize_camera()
    return lab_id

def check_collision(lab_id, collision_threshold=0.0001):
    contact_points = p.getContactPoints(bodyA=lab_id, bodyB=lab_id)
    for contact in contact_points:
        penetration_depth = contact[8]
        if True:#abs(penetration_depth) >= collision_threshold:
            bodyA, bodyB = contact[1], contact[2]
            linkA, linkB = contact[3], contact[4]

            bodyA_name = p.getBodyInfo(bodyA)[1].decode('utf-8')
            bodyB_name = p.getBodyInfo(bodyB)[1].decode('utf-8')
            linkA_name = 'base' if linkA == -1 else p.getJointInfo(bodyA, linkA)[12].decode('utf-8')
            linkB_name = 'base' if linkB == -1 else p.getJointInfo(bodyB, linkB)[12].decode('utf-8')
            collision_msg = f"Collision detected between '{linkA_name}' and '{linkB_name}'."
            print(collision_msg)
            p.addUserDebugText(collision_msg, [0, 0, 0.5], textColorRGB=[1, 0, 0], textSize=1.5, lifeTime=3)

            return True
    return False

def slide_to(lab_id, position, speed, wait_for_completion=True):
    rail_joint_index = 1 # Slide joint index
    
    p.setJointMotorControl2(bodyUniqueId=lab_id,
                            jointIndex=rail_joint_index,
                            controlMode=p.POSITION_CONTROL,
                            targetPosition=position,
                            positionGain=speed,
                            )
    if wait_for_completion:
        current_position = p.getJointState(lab_id, rail_joint_index)[0]
        while abs(current_position - position) > 0.001:  # Tolerance of 1mm
            if check_collision(lab_id):
                print("Collision detected, stopping execution.")
                return True  # Return True to indicate collision
            p.stepSimulation()
            current_position = p.getJointState(lab_id, rail_joint_index)[0]
            time.sleep(1. / 24.)  # Simulation step delay adjusted for more frequent checks

def dorna_execute(lab_id, sequence):
    print(f"Executing sequence: {sequence['sequence_id']} - {sequence['description']}")
    for action in sequence["actions"]:
        if action["cmd"] == "jmove":
            joint_positions = [action["j0"], action["j1"], action["j2"], action["j3"], action["j4"]]
            for i, joint_position in enumerate(joint_positions):
                jointIndex = i + 3  # Starting from arm-base joint
                targetPosition = math.radians(joint_position)
                currentPosition = p.getJointState(lab_id, jointIndex)[0]
                
                # Move towards the target position with small increments
                while abs(currentPosition - targetPosition) > 0.001:  # Tolerance of 1mm
                    # Incremental step towards the target position
                    step = max(min((targetPosition - currentPosition), 0.1), -0.1)
                    p.setJointMotorControl2(bodyUniqueId=lab_id,
                                            jointIndex=jointIndex,
                                            controlMode=p.POSITION_CONTROL,
                                            targetPosition=currentPosition + step,
                                            positionGain=0.03
                                            )
                    p.stepSimulation()
                    if check_collision(lab_id):
                        print("Collision detected, stopping execution.")
                        return  # Stop the entire sequence if collision detected
                    currentPosition += step
                    time.sleep(1. / 24.)  # Increased frequency of simulation steps for smoother movement

def main():
    lab_id = initialize_simulation()
    time.sleep(10)  # Wait for simulation to load
    if slide_to(lab_id, position=-1.2, speed=0.06):
        print("Slide movement stopped due to collision.")
        return  # Exit if collision occurred during sliding
    print('Slided to -1.2')
    
    # Example sequence
    sequences = [
      {
        "sequence_id": 1,
        "description": "Preparation and Initial Positioning",
        "note": ["This sequence is used to prepare the robot for the next steps."],
        "actions": [
            {"id":1,"cmd":"jmove","rel":0,"j0":5,"j1":-45,"j2":111.51123,"j3":10,"j4":-10,"vel":20,"accel":500,"jerk":2000}
        ]
      },
     {
        "sequence_id": 2,
        "description": "Further Positioning",
        "note": ["This sequence is used for further precise positioning."],
        "actions": [
            {"id":2,"cmd":"jmove","rel":0,"j0":5,"j1":45,"j2":411.51123,"j3":1.120605,"j4":-0.395508,"vel":20,"accel":500,"jerk":2000}
        ]
      }
    ]
    for sequence in sequences:
        dorna_execute(lab_id, sequence)
        time.sleep(1)  # Delay between sequences
    
    while True:
        p.stepSimulation()
        if check_collision(lab_id):
            print("Collision detected.")
        time.sleep(1. / 24.)  # Adjusted to match the frequency of checks

main()
