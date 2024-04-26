import cv2
import numpy as np
import pybullet as p
import pybullet_data
import time

p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, 0)
dorna_path ='dorna2-rebuild/urdf/dorna2-rebuild.urdf'
robot = p.loadURDF(dorna_path, [0, 0, 0], useFixedBase=True)
def linear_move_to(lab_id, gripper_link_index, target_position, target_orientation, vel=1):
    # Removed the reconnection logic
    print("Target Position:", target_position)
    print("Target Orientation:", target_orientation)

    ik_joints = p.calculateInverseKinematics(lab_id, gripper_link_index, target_position, target_orientation)
    print("IK Joints:", ik_joints)

    current_positions = [p.getJointState(lab_id, i)[0] for i in range(len(ik_joints))]
    print("Current Positions:", current_positions)

    # Move joints smoothly to the target configuration
    for joint_index, target_position in enumerate(ik_joints):
        print("Joint Index:", joint_index)
        p.setJointMotorControl2(bodyUniqueId=lab_id,
                                jointIndex=joint_index,
                                controlMode=p.POSITION_CONTROL,
                                targetPosition=target_position,
                                positionGain=0.1,  # Tuned for stability
                                maxVelocity=vel)
        p.stepSimulation()  # simulate the environment to reflect changes
 

    return True


def linear_move_relative(lab_id, gripper_link_index, rel_position, vel):
    current_position, current_orn = p.getLinkState(lab_id, gripper_link_index)[4:6]
    target_position = [current_position[i] + rel_position[i] for i in range(3)]
    target_orientation = current_orn  # Keep the orientation the same
    
    return linear_move_to(lab_id, gripper_link_index, target_position, target_orientation)

linear_move_relative(robot, 4, [0,0,0.1], 10)

while True:
    p.stepSimulation()
    time.sleep(1./240)