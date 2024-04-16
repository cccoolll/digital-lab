import pybullet as p
import pybullet_data
import time
import math
import json

# Connect to PyBullet
p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, 0)
wellplateNum = None

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
            parts = line.strip().split('xyz="')[1][:-1]  
            float_coords = list(map(float, parts.split()))  
            positions.append(float_coords)
    return positions

positions = import_positions('def-files/wellplate_pos.def')

# Adjust mesh scale as needed and check visual representation
collisionShapeId = p.createCollisionShape(shapeType=p.GEOM_MESH,
                                          fileName="parts-meshes/1.stl",
                                          meshScale=[1, 1, 1])  # Adjusted mesh scale

# Debugging: visual representation of the collision shape
visualShapeId = p.createVisualShape(shapeType=p.GEOM_MESH,
                                    fileName="parts-meshes/1.stl",
                                    meshScale=[1, 1, 1],
                                    rgbaColor=[1, 0, 0, 1],
                                    specularColor=[0.4, 0.4, 0])

wellplates = []
for position in positions:
    wellplate = p.createMultiBody(baseMass=0,
                                  baseCollisionShapeIndex=collisionShapeId,
                                  baseVisualShapeIndex=visualShapeId,  # Added visual shape
                                  basePosition=position,
                                  baseOrientation=p.getQuaternionFromEuler([3.1416, 0, 1.5708]))
    wellplates.append(wellplate)


def grab_plate(labId, plateId, gripperLinkIndex=7):
    # Get initial absolute positions and orientations
    gripperPos, gripperOrn = p.getLinkState(labId, gripperLinkIndex)[:2]
    platePos, plateOrn = p.getBasePositionAndOrientation(plateId)
    
    # Calculate initial relative position and orientation
    invGripperPos, invGripperOrn = p.invertTransform(gripperPos, gripperOrn)
    global relativePos, relativeOrn
    relativePos, relativeOrn = p.multiplyTransforms(invGripperPos, invGripperOrn, platePos, plateOrn)







collision_threshold = 0.0001  

def print_joint_status(robot_id):
    num_joints = p.getNumJoints(robot_id)
    for j in range(num_joints):
        info = p.getJointState(robot_id, j)
        if j >= 1:
            print(f"Joint {j} status: Pos={math.degrees(info[0]):.3f}, Vel={info[1]:.3f}")
        else:
            print(f"Joint {j} status: Pos={info[0]:.3f}, Vel={info[1]:.3f}")

def check_collision(lab_id, collision_threshold=0.0001):
    contact_points = p.getContactPoints(bodyA=lab_id, bodyB=lab_id)
    for contact in contact_points:
        penetration_depth = contact[8]
        if abs(penetration_depth) >= collision_threshold:  
            collision_msg = f"Collision detected between parts."
            print(collision_msg)
            p.addUserDebugText(collision_msg, [0, 0, 0.5], textColorRGB=[1, 0, 0], textSize=1.5, lifeTime=3)
            return True
    return False

def slide_to(lab_id, position, speed, wait_for_completion=True):
    rail_joint_index = 1  
    p.setJointMotorControl2(bodyUniqueId=lab_id,
                            jointIndex=rail_joint_index,
                            controlMode=p.POSITION_CONTROL,
                            targetPosition=position,
                            positionGain=speed)
    if wait_for_completion:
        current_position = p.getJointState(lab_id, rail_joint_index)[0]
        while abs(current_position - position) > 0.001:  
            p.stepSimulation()  
            if check_collision(lab_id):
                print("Collision detected, stopping execution.")
                return True
            current_position = p.getJointState(lab_id, rail_joint_index)[0]
            time.sleep(1. / 120)

def dorna_execute(lab_id,wellplates, sequence):
    global wellplateNum
    print(f"Executing sequence: {sequence['sequence_id']} - {sequence['description']}")
    for action in sequence["actions"]:
        print(f"Executing action: {action['cmd']}")
        if action["cmd"] == "jmove":
            joint_positions = [action["j0"], action["j1"], action["j2"], action["j3"], action["j4"]]
            for i, joint_position in enumerate(joint_positions):
                jointIndex = i + 2
                targetPosition = math.radians(joint_position)
                currentPosition = p.getJointState(lab_id, jointIndex)[0]

                # Set the desired position with a high position gain to reduce oscillation
                p.setJointMotorControl2(bodyUniqueId=lab_id,
                                        jointIndex=jointIndex,
                                        controlMode=p.POSITION_CONTROL,
                                        targetPosition=targetPosition,
                                        positionGain=0.1,  # Increased gain for tighter control
                                        force=500)  # Max force applied to hold position

                # Wait loop to ensure the joint reaches and holds the position
                while abs(currentPosition - targetPosition) > 0.001:
                    p.stepSimulation()
                    currentPosition = p.getJointState(lab_id, jointIndex)[0]
                    if check_collision(lab_id):
                        print("Collision detected, stopping execution.")
                        return  # Exiting the function early if a collision occurs
                    time.sleep(1./120)
        
        
        elif action["cmd"] == "grab_plate":
            wellplateNum = action["plate_index"]
            
            grab_plate(lab_id, wellplates[wellplateNum])
        
        elif action["cmd"] == "slide_to":
            slide_to(lab_id, action["position"], action["vel"])



last_time = time.time()


print('Slided to -1.2')
sequences = load_sequences_from_json('def-files/dorna_path.json')
for sequence in sequences:
    dorna_execute(labId,wellplates, sequence)



while True:
    p.stepSimulation()
    # Update gripper state
    gripperPos, gripperOrn = p.getLinkState(labId, 7)[:2]
    
    # Calculate new absolute position for the plate
    newPlatePos, newPlateOrn = p.multiplyTransforms(gripperPos, gripperOrn, relativePos, relativeOrn)
    
    # Set plate's new position and orientation
    if wellplateNum is not None:
        p.resetBasePositionAndOrientation(wellplates[wellplateNum], newPlatePos, newPlateOrn)
    
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
        print_joint_status(labId)  
        last_time = current_time  

    time.sleep(1. / 120)
