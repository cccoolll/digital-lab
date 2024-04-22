import pybullet as p
import pybullet_data
import time
import math
import json

# Connect to PyBullet
p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, 0)

# Load the optical table
optical_table = p.loadURDF("Digital_twin_lab-3/urdf/Digital_twin_lab-3.urdf", [0, 0, 0], useFixedBase=True)



def load_sequences_from_json(filepath):
    with open(filepath, 'r') as file:
        data = json.load(file)
    return data['sequences']

def import_positions(filename):
    positions = []
    with open(filename, 'r') as file:
        for line in file:
            parts = line.strip().split('xyz="')[1][:-1]  
            float_coords = list(map(float, parts.split()))  
            positions.append(float_coords)
    return positions

positions = import_positions('def-files/wellplate_pos.def')

# Find the slide rail position
num_joints = p.getNumJoints(optical_table)
slide_joint_index = -1
for joint_index in range(num_joints):
    joint_info = p.getJointInfo(optical_table, joint_index)
    if joint_info[1].decode('utf-8') == 'slide-j':
        slide_joint_index = joint_index
        slide_pos, slide_orn = p.getLinkState(optical_table, slide_joint_index)[:2]
        break


# Define the offset and rotation from the slide rail to the arm base according to the URDF
offset = [0, 0, 0.0557629137973782] #   y,z,-x
rotation = p.getQuaternionFromEuler([0,0, 0])

# Load the robotic arm at the position offset from the slide rail
arm_id = p.loadURDF("dorna2-rebuild/urdf/dorna2-rebuild.urdf", p.multiplyTransforms(slide_pos, slide_orn, offset, rotation)[0], p.multiplyTransforms(slide_pos, slide_orn, offset, rotation)[1])
# Since the arm should be fixed relative to the slide rail, we attach them using a fixed joint
p.createConstraint(optical_table, slide_joint_index, arm_id, -1, p.JOINT_FIXED, [0, 0, 0], offset, [0, 0, 0])

# Set a target position for the slider
target_position = -0.59682
p.setJointMotorControl2(bodyUniqueId=optical_table,
                        jointIndex=slide_joint_index,
                        controlMode=p.POSITION_CONTROL,
                        targetPosition=target_position)

# Run the simulation
while True:
    p.stepSimulation()
    time.sleep(1./60.)

# Disconnect from PyBullet
p.disconnect()
