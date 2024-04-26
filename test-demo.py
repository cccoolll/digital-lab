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
flags = p.URDF_USE_SELF_COLLISION
labId = p.loadURDF("Digital_twin_lab-2/urdf/Digital_twin_lab-2.urdf", [0, 0, 0], useFixedBase=True, flags=flags)


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
inertia = [0.1, 0, 0, 0, 0.1, 0, 0, 0, 0.1]  # Diagonal inertia tensor for simplicity

for i, position in enumerate(positions):
    if i>=6 and i<22:
        wellplate = p.createMultiBody(baseMass=0.1,
                                    baseCollisionShapeIndex=collisionShapeId,
                                    baseVisualShapeIndex=visualShapeId,  # Added visual shape
                                    basePosition=position,
                                    baseOrientation=p.getQuaternionFromEuler([3.1416, 0, 1.5708])
                                    )
        wellplates.append(wellplate)

time.sleep(3)
def grab_plate(labId, plateId, gripperLinkIndex=7):
    # Get initial absolute positions and orientations
    gripperPos, gripperOrn = p.getLinkState(labId, gripperLinkIndex)[:2]
    platePos, plateOrn = p.getBasePositionAndOrientation(plateId)
    
    # Calculate initial relative position and orientation
    invGripperPos, invGripperOrn = p.invertTransform(gripperPos, gripperOrn)
    global relativePos, relativeOrn
    relativePos, relativeOrn = p.multiplyTransforms(invGripperPos, invGripperOrn, platePos, plateOrn)
    
    # Set plate's initial position relative to the gripper immediately after grabbing
    newPlatePos, newPlateOrn = p.multiplyTransforms(gripperPos, gripperOrn, relativePos, relativeOrn)
    p.resetBasePositionAndOrientation(plateId, newPlatePos, newPlateOrn)


def linear_move_to(lab_id, gripper_link_index, target_position, target_orientation, vel):
    # Removed the reconnection logic
    print("Target Position:", target_position)
    print("Target Orientation:", target_orientation)

    ik_joints = p.calculateInverseKinematics(lab_id, gripper_link_index, target_position, target_orientation)
    print("IK Joints:", ik_joints)

    current_positions = [p.getJointState(lab_id, i)[0] for i in range(len(ik_joints))]

    # Move joints smoothly to the target configuration
    for joint_index, target_position in enumerate(ik_joints):
        p.setJointMotorControl2(bodyUniqueId=lab_id,
                                jointIndex=joint_index,
                                controlMode=p.POSITION_CONTROL,
                                targetPosition=target_position,
                                positionGain=0.05,  # Tuned for stability
                                maxVelocity=vel)
        p.stepSimulation()  # simulate the environment to reflect changes
        if check_collision():
            print("Collision detected during linear move, stopping execution.")
            return False

    return True

def linear_move_relative(lab_id, gripper_link_index, rel_position, vel):
    # Removed the reconnection logic
    
    current_position, current_orn = p.getLinkState(lab_id, gripper_link_index)[4:6]
    target_position = [current_position[i] + rel_position[i] for i in range(3)]
    target_orientation = current_orn  # Keep the orientation the same
    
    return linear_move_to(lab_id, gripper_link_index, target_position, target_orientation, vel)




collision_threshold = 0.0001  

def print_joint_status(robot_id):
    num_joints = p.getNumJoints(robot_id)
    for j in range(num_joints):
        info = p.getJointState(robot_id, j)
        if j >= 1:
            print(f"Joint {j} status: Pos={math.degrees(info[0]):.3f}, Vel={info[1]:.3f}")
        else:
            print(f"Joint {j} status: Pos={info[0]:.3f}, Vel={info[1]:.3f}")

def check_collision(collision_threshold=0.0001):
    contact_points = p.getContactPoints()  # This retrieves all contact points in the simulation
    for contact in contact_points:
        bodyA, bodyB = contact[1], contact[2]
        penetration_depth = contact[8]
        if abs(penetration_depth) >= collision_threshold and bodyA != bodyB:  # Check for substantial penetration and that the bodies are different
            bodyA_name = p.getBodyInfo(bodyA)[1].decode('utf-8')
            bodyB_name = p.getBodyInfo(bodyB)[1].decode('utf-8')
            print(f"Collision detected between {bodyA_name} and {bodyB_name} with depth of {penetration_depth:.3f}")
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
            if wellplateNum != -1:
                gripperPos, gripperOrn = p.getLinkState(lab_id, 7)[:2]
                newPlatePos, newPlateOrn = p.multiplyTransforms(gripperPos, gripperOrn, relativePos, relativeOrn)
                p.resetBasePositionAndOrientation(wellplates[wellplateNum], newPlatePos, newPlateOrn)
            if check_collision():
                print("Collision detected, stopping execution.")
                #return True
            current_position = p.getJointState(lab_id, rail_joint_index)[0]
            time.sleep(1. / 240)

def dorna_execute(lab_id, wellplates, sequence):
    global wellplateNum, relativePos, relativeOrn
    print(f"Executing sequence: {sequence['sequence_id']} - {sequence['description']}")
    for action in sequence["actions"]:
        print(f"Executing action: {action['cmd']}")
        if action["cmd"] == "jmove":
            joint_positions = [math.radians(action["j0"]), math.radians(action["j1"]), math.radians(action["j2"]), math.radians(action["j3"]), math.radians(action["j4"])]
            for i, joint_position in enumerate(joint_positions):
                jointIndex = i + 2  # Adjust joint index according to your URDF file mapping
                p.setJointMotorControl2(bodyUniqueId=lab_id,
                                        jointIndex=jointIndex,
                                        controlMode=p.POSITION_CONTROL,
                                        targetPosition=joint_position,
                                        positionGain=0.05,  # Adjust gain as needed for stability
                                        force=500)  # Adjust maximum force if necessary

            all_in_position = False
            while not all_in_position:
                p.stepSimulation()
                all_in_position = True
                for i, joint_position in enumerate(joint_positions):
                    jointIndex = i + 2
                    currentPosition = p.getJointState(lab_id, jointIndex)[0]
                    if abs(currentPosition - joint_position) > 0.001:
                        all_in_position = False
                        break  # Exit the loop early if any joint is not in position

                # Update well plate position during joint movements
                if wellplateNum != -1:
                    gripperPos, gripperOrn = p.getLinkState(lab_id, 7)[:2]
                    newPlatePos, newPlateOrn = p.multiplyTransforms(gripperPos, gripperOrn, relativePos, relativeOrn)
                    p.resetBasePositionAndOrientation(wellplates[wellplateNum], newPlatePos, newPlateOrn)

                if check_collision():
                    print("Collision detected, stopping execution.")
                    #return  # Exit the function if a collision occurs

                time.sleep(1./240)
        elif action["cmd"] == "lmove" and action["rel"] == 0:
            target_position = [action["x"], action["y"], action["z"]]
            if action["rel"]:  # If the movement is relative
                current_pos, _ = p.getLinkState(lab_id, 7)[:2]
                target_position = [current_pos[i] + target_position[i] for i in range(3)]
            success = linear_move_to(lab_id, 7, target_position, action["vel"])
            if not success:
                print(f"Completed linear move to {target_position}")
            else:
                print("Linear move failed or interrupted by collision.")

        elif action["cmd"] == "lmove" and action["rel"] == 1:
            # Prepare the target relative distance directly from the action parameters
            target_distance = [action["x"], action["y"], action["z"]]
            # Perform the relative linear move
            collision_detected = linear_move_relative(lab_id, 7, target_distance, action["vel"])
            if not collision_detected:
                print(f"Completed linear move distance: {target_distance}")
            else:
                print("Linear move failed or interrupted by collision.")

        elif action["cmd"] == "grab_plate":
            wellplateNum = action["plate_index"]
            print(f"Grabbing plate {wellplateNum}")
            grab_plate(lab_id, wellplates[wellplateNum])

        elif action["cmd"] == "slide_to":
            slide_to(lab_id, action["position"], action["vel"])




last_time = time.time()

sequences = load_sequences_from_json('def-files/dorna_path.json')
for sequence in sequences:
    dorna_execute(labId,wellplates, sequence)



while True:
    p.stepSimulation()

    #print(f'plate{wellplateNum}')
    
    # Set plate's new position and orientation
    if wellplateNum != -1:
        # Update gripper state
        gripperPos, gripperOrn = p.getLinkState(labId, 7)[:2]
        
        # Calculate new absolute position for the plate
        newPlatePos, newPlateOrn = p.multiplyTransforms(gripperPos, gripperOrn, relativePos, relativeOrn)
        p.resetBasePositionAndOrientation(wellplates[wellplateNum], newPlatePos, newPlateOrn)
    
    check_collision()
 
    current_time = time.time()
    if current_time - last_time >= 3:
        print_joint_status(labId)  
        last_time = current_time  

    time.sleep(1. / 240)
