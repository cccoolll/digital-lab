import pybullet as p
import pybullet_data
import time
import math
import json

# Connect to PyBullet
p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, 0)
#planeId = p.loadURDF("plane.urdf")
flags = p.URDF_USE_SELF_COLLISION
labId = p.loadURDF("C:/Users/songtao.cheng/Documents/myphd/imaging_farm/Digital Twin Lab/Digital_twin_lab-2/urdf/Digital_twin_lab-2.urdf", [0, 0, 0], useFixedBase=True, flags=flags)

def load_sequences_from_json(filepath):
    with open(filepath, 'r') as file:
        data = json.load(file)
    return data['sequences']

def import_positions(filename):
    positions = []
    with open(filename, 'r') as file:
        for line in file:
            parts = line.strip().split('xyz="')[1][:-1]  # Split the line and remove the trailing quote
            float_coords = list(map(float, parts.split()))  # Convert the string coordinates to floats
            positions.append(float_coords)
    return positions

# Example usage
positions = import_positions('def-files/wellplate_pos.def')

collisionShapeId = p.createCollisionShape(shapeType=p.GEOM_MESH,
                                          fileName="parts-meshes/1.stl",
                                          meshScale=[1, 1, 1]) # Adjust mesh scale as needed

wellplates = []
for position in positions:
    wellplate = p.createMultiBody(baseCollisionShapeIndex=collisionShapeId,
                                  basePosition=position,  # Use positions from the list
                                  baseOrientation=p.getQuaternionFromEuler([3.1416, 0, 1.5708]))
    wellplates.append(wellplate)



collision_threshold = 0.0001  # Threshold for collision detection

def print_joint_status(robot_id):
    num_joints = p.getNumJoints(robot_id)
    for j in range(num_joints):
        info = p.getJointState(robot_id, j)
        if j >= 2:
            print(f"Joint {j} status: Pos={math.degrees(info[0]):.3f}, Vel={info[1]:.3f}")
        else:
            print(f"Joint {j} status: Pos={info[0]:.3f}, Vel={info[1]:.3f}")

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


last_time = time.time()

while True:
    p.stepSimulation()
    contact_points = p.getContactPoints()
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

    current_time = time.time()
    if current_time - last_time >= 3:
        print_joint_status(labId)  # Print the status of all joints for the robot every two seconds
        last_time = current_time  # Reset the timer

    time.sleep(1. / 24)  # Maintain the simulation step delay
