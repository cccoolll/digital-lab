import pybullet as p
import pybullet_data
import numpy as np
import time
import math
from scipy.spatial.transform import Rotation as R

def initialize_simulation():
    p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, 0)
    flags = p.URDF_USE_SELF_COLLISION
    lab_id = p.loadURDF("SimulatedLab/urdf/SimulatedLab.urdf", [-1, 1, 0], useFixedBase=True, flags=flags)
    return lab_id

def check_collision(lab_id, collision_threshold=0.003):
    contact_points = p.getContactPoints(bodyA=lab_id, bodyB=lab_id)
    for contact in contact_points:
        penetration_depth = contact[8]
        if abs(penetration_depth) >= collision_threshold:
            bodyA, bodyB = contact[1], contact[2]
            linkA, linkB = contact[3], contact[4]

            bodyA_name = p.getBodyInfo(bodyA)[1].decode('utf-8')
            bodyB_name = p.getBodyInfo(bodyB)[1].decode('utf-8')
            linkA_name = 'base' if linkA == -1 else p.getJointInfo(bodyA, linkA)[12].decode('utf-8')
            linkB_name = 'base' if linkB == -1 else p.getJointInfo(bodyB, linkB)[12].decode('utf-8')

            print(f"Collision detected between '{bodyA_name}:{linkA_name}' and '{bodyB_name}:{linkB_name}' with penetration depth of {abs(penetration_depth):.3f} meters.")
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
            p.stepSimulation()
            current_position = p.getJointState(lab_id, rail_joint_index)[0]
            time.sleep(1. / 24.)  # Simulation step delay

def dorna_execute(lab_id, sequence):
    print(f"Executing sequence: {sequence['sequence_id']} - {sequence['description']}")
    for action in sequence["actions"]:
        if action["cmd"] == "jmove":
            joint_positions = [action["j0"], action["j1"], action["j2"], action["j3"], action["j4"]]
            for i, joint_position in enumerate(joint_positions):
                jointIndex=i+3
                p.setJointMotorControl2(bodyUniqueId=lab_id,
                                        jointIndex=jointIndex, # Starting from arm-base joint
                                        controlMode=p.POSITION_CONTROL,
                                        targetPosition=math.radians(joint_position),
                                        positionGain=0.1
                                        )
            time.sleep(1)  # Wait for the arm to reach the target position

        # Check for collisions after each action
        if check_collision(lab_id):
            print("Collision detected, stopping execution.")
            return

def main():
    lab_id = initialize_simulation()
    time.sleep(2)  # Wait for simulation to load
    slide_to(lab_id, position=-1.2, speed=0.1)
    print('slided to -1.2')
    
    # Example sequence
    sequences = [
      {
        "sequence_id": 1,
        "description": "Preparation and Initial Positioning",
        "note": ["This sequence is used to prepare the robot for the next steps."],
        "actions": [
            {"id":101,"cmd":"jmove","rel":0,"j0":5,"j1":27.13623,"j2":311.51123,"j3":1.120605,"j4":-0.395508,"vel":20,"accel":500,"jerk":2000}
        ]
      }
    ]
    for sequence in sequences:
        dorna_execute(lab_id, sequence)
    
    while True:
        p.stepSimulation()
        time.sleep(1. / 24.)

main()
