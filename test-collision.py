import pybullet as p
import pybullet_data
import numpy as np
import time

def matrix_to_quaternion(matrix):
    """
    Convert a 3x3 rotation matrix to a quaternion.
    :param matrix: 3x3 rotation matrix as a numpy array.
    :return: Quaternion as a list [x, y, z, w].
    """
    m = np.array(matrix, dtype=np.float64)
    tr = np.trace(m)
    if tr > 0:
        S = np.sqrt(tr+1.0) * 2  # S=4*qw
        qw = 0.25 * S
        qx = (m[2, 1] - m[1, 2]) / S
        qy = (m[0, 2] - m[2, 0]) / S
        qz = (m[1, 0] - m[0, 1]) / S
    elif (m[0, 0] > m[1, 1]) and (m[0, 0] > m[2, 2]):
        S = np.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2]) * 2  # S=4*qx
        qw = (m[2, 1] - m[1, 2]) / S
        qx = 0.25 * S
        qy = (m[0, 1] + m[1, 0]) / S
        qz = (m[0, 2] + m[2, 0]) / S
    elif m[1, 1] > m[2, 2]:
        S = np.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2]) * 2  # S=4*qy
        qw = (m[0, 2] - m[2, 0]) / S
        qx = (m[0, 1] + m[1, 0]) / S
        qy = 0.25 * S
        qz = (m[1, 2] + m[2, 1]) / S
    else:
        S = np.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1]) * 2  # S=4*qz
        qw = (m[1, 0] - m[0, 1]) / S
        qx = (m[0, 2] + m[2, 0]) / S
        qy = (m[1, 2] + m[2, 1]) / S
        qz = 0.25 * S
    return [qx, qy, qz, qw]

def calculate_position(joint_xyz, joint_rpy, parent_xyz, parent_rpy):
    """
    Calculates the final position in the world coordinates.
    """
    parent_trans = p.getMatrixFromQuaternion(p.getQuaternionFromEuler(parent_rpy))
    joint_trans = p.getMatrixFromQuaternion(p.getQuaternionFromEuler(joint_rpy))
    
    parent_trans = np.array(parent_trans).reshape(3, 3)
    joint_trans = np.array(joint_trans).reshape(3, 3)
    
    world_trans = np.dot(parent_trans, joint_trans)
    
    world_quaternion = matrix_to_quaternion(world_trans)
    
    world_position = np.dot(parent_trans, np.array(joint_xyz)) + np.array(parent_xyz)
    world_position = [world_position[0], world_position[1], world_position[2]]
    #world_position = [world_position[0]+0.0393, world_position[1]-0.07, world_position[2]]
    print(world_position)
    
    return world_position, world_quaternion

# Connect to PyBullet
p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, 0)
flags = p.URDF_USE_SELF_COLLISION
labId = p.loadURDF("C:/Users/songtao.cheng/Documents/myphd/imaging_farm/Digital Twin Lab/Digital_twin_lab/urdf/Digital_twin_lab.urdf", [0, 0, 0], useFixedBase=True, flags=flags)

# Define joint and parent information for the position '1' from URDF (example values)
joint_xyz = [-0.111219143031836, 0.29870945080481, -0.153551639442251]  # joint origin position for '1j'
joint_rpy = [-1.57079, 1.57079, 0]  # joint origin orientation for '1j'
parent_xyz = [-0.0325122942941528, -0.825674017096931, 0.0612003345061164]  # position of microplate_incubator
parent_rpy = [1.5707963267949, 0, 0]  # orientation of microplate_incubator

# Calculate the correct position
world_position, world_orientation = calculate_position(joint_xyz, joint_rpy, parent_xyz, parent_rpy)

# Adjust the mesh scale and location according to the calculated position
collisionShapeId = p.createCollisionShape(shapeType=p.GEOM_MESH,
                                          fileName="cube-meshes/96wellplate.stl",
                                          meshScale=[0.001, 0.001, 0.001])

wellplate = p.createMultiBody(baseCollisionShapeIndex=collisionShapeId,
                           basePosition=world_position, # Use the calculated position
                           baseOrientation=world_orientation) # Use the calculated orientation

collision_threshold = 0.003  # 3mm threshold for collision detection

# Simulation loop
while True:
    p.stepSimulation()
    contact_points = p.getContactPoints(bodyA=labId, bodyB=labId)
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

    time.sleep(1. / 24)
