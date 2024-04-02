import sys
from PyQt5.QtWidgets import QApplication, QWidget, QVBoxLayout, QLabel, QLineEdit, QPushButton
import pybullet as p
import pybullet_data
import time
from threading import Thread

class RobotController(QWidget):
    def __init__(self):
        super().__init__()
        self.initUI()
        
    def initUI(self):
        self.layout = QVBoxLayout()

        self.info_label = QLabel("Movable Links:")
        self.layout.addWidget(self.info_label)
        
        # Display movable links
        for i in range(p.getNumJoints(labId)):
            joint_info = p.getJointInfo(labId, i)
            joint_name = joint_info[1].decode('utf-8')
            joint_type = joint_info[2]
            if joint_type != p.JOINT_FIXED:
                limit_info = f"{joint_name}: {joint_info[8]} to {joint_info[9]}"
                self.layout.addWidget(QLabel(limit_info))
        
        self.link_input = QLineEdit(self)
        self.link_input.setPlaceholderText("Enter Link Number")
        self.layout.addWidget(self.link_input)
        
        self.position_input = QLineEdit(self)
        self.position_input.setPlaceholderText("Enter Position")
        self.layout.addWidget(self.position_input)
        
        self.move_button = QPushButton('Move Link', self)
        self.move_button.clicked.connect(self.move_link)
        self.layout.addWidget(self.move_button)
        
        self.setLayout(self.layout)
        self.setWindowTitle('Robot Controller')
        self.show()
    
    def move_link(self):
        link_number = int(self.link_input.text())
        position = float(self.position_input.text())
        p.setJointMotorControl2(bodyUniqueId=labId,
                                jointIndex=link_number,
                                controlMode=p.POSITION_CONTROL,
                                targetPosition=position)
        
def run_pybullet():
    p.connect(p.GUI) 
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -10)
    flags = p.URDF_USE_SELF_COLLISION
    global labId
    labId = p.loadURDF("/SimulatedLab/urdf/SimulatedLab.urdf", [-1, 1, 0], useFixedBase=True, flags=flags)
    
    while True:
        p.stepSimulation()
        time.sleep(1. / 240)
    
    p.disconnect()

if __name__ == '__main__':
    # PyBullet Simulation in a separate thread
    bullet_thread = Thread(target=run_pybullet)
    bullet_thread.start()
    time.sleep(1)  # Wait for the simulation to start
    
    # PyQt GUI in main thread
    app = QApplication(sys.argv)
    ex = RobotController()
    sys.exit(app.exec_())
