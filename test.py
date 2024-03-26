import pybullet as p
import pybullet_data
import time

p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -10)
cubeStartOrientation = p.getQuaternionFromEuler([-1.57079632679, 0, 0])
p.setPhysicsEngineParameter(enableFileCaching=0) # Disable caching for development
p.configureDebugVisualizer(p.COV_ENABLE_SHADOWS, 1) # Enable shadows
plane = p.loadURDF("C:/Users/songtao.cheng/Documents/myphd/imaging_farm/Simulated_lab/CAD/SimulatedLab/urdf/SimulatedLab.urdf",[-1,1,0],useFixedBase=True)
#cube = p.loadURDF("cube.urdf", [0, 0, 1])


while True:
    p.stepSimulation()
    time.sleep(1./240)
