
import pybullet as p
import pybullet_data
import time

# Connect to PyBullet
p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, 0)
#planeId = p.loadURDF("plane.urdf")
flags = p.URDF_USE_SELF_COLLISION
labId = p.loadURDF("/SimulatedLab/urdf/SimulatedLab.urdf", [-1, 1, 0], useFixedBase=True, flags=flags)

collision_threshold = 0.003  # 3mm threshold for collision detection

while True:
    p.stepSimulation()
    contact_points = p.getContactPoints(bodyA=labId,bodyB=labId)
    for contact in contact_points:
        penetration_depth = contact[8]  # Negative value indicating the penetration depth
        if abs(penetration_depth) >= collision_threshold:
            bodyA, bodyB = contact[1], contact[2]
            linkA, linkB = contact[3], contact[4]

            bodyA_name = p.getBodyInfo(bodyA)[1].decode('utf-8')
            bodyB_name = p.getBodyInfo(bodyB)[1].decode('utf-8')
            linkA_name = 'base' if linkA == -1 else p.getJointInfo(bodyA, linkA)[12].decode('utf-8')
            linkB_name = 'base' if linkB == -1 else p.getJointInfo(bodyB, linkB)[12].decode('utf-8')

            print(f"Collision detected between '{bodyA_name}:{linkA_name}' and '{bodyB_name}:{linkB_name}' with penetration depth of {abs(penetration_depth):.3f} meters.")

    time.sleep(1. / 24)  # Adjusted sleep time for more fluid interaction