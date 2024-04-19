import pybullet as p
import pybullet_data
import time
import math
import json

# Connect to PyBullet
p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, 0)

wellplateNum = -1
#flags = p.URDF_USE_SELF_COLLISION
optical_table = p.loadURDF("C:/Users/songtao.cheng/Documents/myphd/imaging_farm/Digital Twin Lab/Digital_twin_lab-3/urdf/Digital_twin_lab-3.urdf", [0, 0, 0], useFixedBase=True)#, flags=flags)
arm_id = p.loadURDF("dorna2-rebuild/urdf/dorna2-rebuild.urdf", [0, 0, 0])
# Simulation parameters
num_joints = p.getNumJoints(optical_table)
print("Number of joints:", num_joints)

# Find the index of the "slide-j" prismatic joint
for joint_index in range(num_joints):
    joint_info = p.getJointInfo(optical_table, joint_index)
    if joint_info[1].decode('utf-8') == 'slide-j':
        slide_joint_index = joint_index
        print("Found 'slide-j' at index:", slide_joint_index)

# Control parameters for the prismatic joint
target_position = -1  # Target position for the slider, within the limits [-3.5, 0]

# Control the slide
p.setJointMotorControl2(bodyUniqueId=optical_table,
                        jointIndex=slide_joint_index,
                        controlMode=p.POSITION_CONTROL,
                        targetPosition=target_position)

# Run the simulation for a few seconds
for _ in range(240):  # 240 steps, 4 seconds if simulation step is set to 1/60
    p.stepSimulation()
    time.sleep(1./60.)  # Optional, to match real time

# Disconnect from PyBullet
p.disconnect()
