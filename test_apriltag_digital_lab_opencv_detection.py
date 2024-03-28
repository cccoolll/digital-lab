import cv2
import numpy as np

from pupil_apriltags import Detector
import pybullet as p
import pybullet_data
import time
import os
# Get the directory of the current script
script_dir = os.path.dirname(os.path.abspath(__file__))
# TODO [] Add camera offset and add arrow to show the direction of the camera

p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())

dorna_path = os.path.join(script_dir, 'dorna.urdf')
stl_path = os.path.join(script_dir, '96wellplate.urdf')
robot = p.loadURDF(dorna_path)

stl_object = p.loadURDF(stl_path, basePosition=[0.1, 0.1, 0.1])

cap = cv2.VideoCapture(2)  # Adjust the device index if necessary
if not cap.isOpened():
    print("Cannot open camera")
    exit()

cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

#fx=650.9478759765625, fy=650.244140625, cx=646.8314208984375, cy=363.080322265625
fx, fy, cx, cy = 650.9478759765625,650.244140625,646.8314208984375,363.080322265625
camera_matrix = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]])
tag_size = 0.06  # Size of the AprilTag side in meters
tag_to_object_offset_position = [0.1, 0.03, -0.06]  # Adjust based on tag position vs. object center
tag_to_object_offset_orientation = [0, -1.57079632679, 0]  # Adjust if there's an orientation offset
camera_offset_position = [-0.05, 0, -0.05]  # Example offset: adjust accordingly
camera_offset_orientation = [0, 0, 0]  # Example orientation offset in degrees: adjust accordingly

def adjust_tag_pose_to_object_pose(tag_position, tag_orientation, offset_position, offset_orientation):
    # Convert tag orientation to quaternion
    tag_orient_quat = p.getQuaternionFromEuler(np.radians(tag_orientation))
    
    # Convert offset orientation to quaternion
    offset_orient_quat = p.getQuaternionFromEuler(np.radians(offset_orientation))
    
    # Apply offset to the tag's position and orientation
    adjusted_pos, adjusted_orient_quat = p.multiplyTransforms(tag_position, tag_orient_quat, offset_position, offset_orient_quat)
    
    # Convert adjusted orientation back to Euler angles for further processing
    adjusted_orient = np.degrees(p.getEulerFromQuaternion(adjusted_orient_quat))
    
    return np.array(adjusted_pos), adjusted_orient







# Initialize AprilTag detector with camera parameters for pose estimation
detector = Detector(families='tag36h11',
                    nthreads=4,
                    quad_decimate=1.0,
                    quad_sigma=0.0,
                    refine_edges=1,
                    decode_sharpening=0.25,
                    debug=0)

camera_link_index = 5
 # The camera link index in the URDF file

def get_camera_pose(robot,camera_link_index,offset_orientation,offset_position):
    # Get the position and orientation of the camera link
    camera_pos, camera_orient = p.getLinkState(robot, camera_link_index)[4:6]
    offset_orient_quat = p.getQuaternionFromEuler(np.radians(offset_orientation))
    world_pos, world_orient = p.multiplyTransforms(camera_pos, camera_orient, offset_position, offset_orient_quat)
    
    return np.array(world_pos), np.array(world_orient)

def transform_position_orientation(relative_pos_tag, relative_orient_tag, camera_pos, camera_orient,tag_to_object_offset_position, tag_to_object_offset_orientation):
    """
    Transform the relative position and orientation of an object (detected by the camera) to the world frame.
    """
    # Convert the relative orientation (Euler angles) to a quaternion
    relative_pos,relative_orient = adjust_tag_pose_to_object_pose(relative_pos_tag, relative_orient_tag, tag_to_object_offset_position, tag_to_object_offset_orientation)
    relative_orient_quat = p.getQuaternionFromEuler(relative_orient)

    # Transform the relative position and orientation to the world frame
    world_pos = np.array(p.multiplyTransforms(camera_pos, camera_orient, relative_pos, relative_orient_quat)[0])
    world_orient = p.multiplyTransforms(camera_pos, camera_orient, [0, 0, 0], relative_orient_quat)[1]

    return world_pos, world_orient

def euler_from_rotation_matrix(R):
    ''' Calculate Euler angles (yaw,pitch,roll) from a rotation matrix. '''
    sy = np.sqrt(R[0, 0] * R[0, 0] + R[1, 0] * R[1, 0])
    singular = sy < 1e-6
    if not singular:
        x = np.arctan2(R[2, 1], R[2, 2])
        y = np.arctan2(-R[2, 0], sy)
        z = np.arctan2(R[1, 0], R[0, 0])
    else:
        x = np.arctan2(-R[1, 2], R[1, 1])
        y = np.arctan2(-R[2, 0], sy)
        z = 0
    return np.rad2deg(x), np.rad2deg(y), np.rad2deg(z)  # Convert to degrees

def detect_apriltag():
    # Initialize return variables
    detected_position = None
    detected_orientation = None

    ret, color_image = cap.read() 
    if not ret:
        print("Failed to grab frame")
        return None, None


    color_image=cv2.rotate(color_image, cv2.ROTATE_180)

    gray = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)
    tags = detector.detect(gray, estimate_tag_pose=True, camera_params=[fx, fy, cx, cy], tag_size=tag_size)
    for tag in tags:
        # Extract tag position, orientation, and draw bounding box
        (ptA, ptB, ptC, ptD) = tag.corners
        cv2.polylines(color_image, [np.array(tag.corners, np.int32).reshape((-1,1,2))], True, (0,255,0), 2, cv2.LINE_AA)

        ptA, ptB, ptC, ptD = np.array(ptA, dtype=int), np.array(ptB, dtype=int), np.array(ptC, dtype=int), np.array(ptD, dtype=int)
        cv2.line(color_image, tuple(ptA), tuple(ptB), (0, 255, 0), 2)
        cv2.line(color_image, tuple(ptB), tuple(ptC), (0, 255, 0), 2)
        cv2.line(color_image, tuple(ptC), tuple(ptD), (0, 255, 0), 2)
        cv2.line(color_image, tuple(ptD), tuple(ptA), (0, 255, 0), 2)

        tvec = tag.pose_t  # Translation vector
        R = tag.pose_R  # Rotation matrix
        ang1, ang2 , ang3 = euler_from_rotation_matrix(R)

        detected_position = [ tvec[1][0],tvec[0][0], -tvec[2][0]]
        detected_orientation = [ang2, ang1, -ang3]  # Assuming PyBullet uses the same convention

        # Display tag ID and orientation on the frame
        tag_center = np.mean(tag.corners, axis=0).astype(int)
        # Line 1: Tag ID
        cv2.putText(color_image, f"ID: {tag.tag_id}", 
                    (tag_center[0]-150, tag_center[1] - 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
        # Line 2: X, Y, Z positions
        cv2.putText(color_image, f"X: {tvec[0][0]*100:.2f}cm, Y: {tvec[1][0]*100:.2f}cm, Z: {tvec[2][0]*100:.2f}cm", 
                    (tag_center[0]-150, tag_center[1] - 15), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
        # Line 3: Roll, Pitch, Yaw orientations
        cv2.putText(color_image, f"Yaw: {ang2:.2f}, Pitch: {ang1:.2f}, Roll: {ang3:.2f}", 
                    (tag_center[0]-150, tag_center[1]), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

        # Display the resulting frame
    
    cv2.imshow('Frame', color_image)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        cv2.destroyAllWindows()

    return detected_position, detected_orientation


while True:
    # Call the detection function
    position, orientation = detect_apriltag()


    if position is not None and orientation is not None:
        # Convert orientation from Euler angles to a quaternion for PyBullet
        quaternion = p.getQuaternionFromEuler([np.radians(orientation[0]), np.radians(orientation[1]), np.radians(orientation[2])])

        camera_pos, camera_orient = get_camera_pose(robot, camera_link_index, camera_offset_orientation, camera_offset_position)
        relative_pos = position  # The position relative to the camera
        relative_orient = [np.radians(o) for o in orientation]  # The orientation (Euler angles) relative to the camera
        world_pos, world_orient = transform_position_orientation(relative_pos, relative_orient, camera_pos, camera_orient, tag_to_object_offset_position, tag_to_object_offset_orientation)
        
        # Update the stl_object's position and orientation in the simulation
        p.resetBasePositionAndOrientation(stl_object, world_pos, world_orient)

    p.stepSimulation()
    time.sleep(1./24)
