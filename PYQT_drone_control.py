import sys
import airsim
import time
from scipy.spatial.transform import Rotation
from PyQt5.QtWidgets import QApplication, QWidget, QPushButton

class DroneControlApp:
    def __init__(self):
        # set constants for offset movements
        self.inc_left_right = 10 # meters
        self.inc_up_down = 5 # meters
        self.inc_fwd_rev = 5 #meters
        self.travel_speed = 5 # meters/sec
        self.inc_left_right_theta = 10 #degress

    def init_drone(self):
        # get airsim client
        self.client = airsim.MultirotorClient()
        self.client.confirmConnection()

        # enable api control
        self.client.enableApiControl(True)

    def create_window(self):
        self.app = QApplication(sys.argv)
        self.widget = QWidget()

        # set window geometry and title
        self.widget.setGeometry(50, 50, 420, 420)
        self.widget.setWindowTitle("PyQT Drone Control Application")

        # create buttons
        self.up_button = QPushButton(self.widget)
        self.down_button = QPushButton(self.widget)
        self.right_button = QPushButton(self.widget)
        self.left_button = QPushButton(self.widget)
        self.fwd_button = QPushButton(self.widget)
        self.rev_button = QPushButton(self.widget)
        self.rotate_left_button = QPushButton(self.widget)
        self.rotate_right_button = QPushButton(self.widget)
        self.arm_button = QPushButton(self.widget)
        self.disarm_button = QPushButton(self.widget)
        self.takeoff_button = QPushButton(self.widget)
        self.land_button = QPushButton(self.widget)

        # add text to buttons
        self.up_button.setText("Up")
        self.down_button.setText("Down")
        self.right_button.setText("Strafe Right")
        self.left_button.setText("Strafe Left")
        self.rotate_left_button.setText("Rotate Left")
        self.rotate_right_button.setText("Rotate Right")
        self.fwd_button.setText("Forward")
        self.rev_button.setText("Reverse")
        self.arm_button.setText("Arm UAV")
        self.disarm_button.setText("Disarm UAV")
        self.takeoff_button.setText("Takeoff")
        self.land_button.setText("Land")

        # move buttons into place
        self.up_button.move(160, 32)
        self.right_button.move(244, 64)
        self.left_button.move(75, 64)
        self.down_button.move(160, 96)
        self.rotate_left_button.move(75, 160)
        self.rotate_right_button.move(244, 160)
        self.fwd_button.move(160, 220)
        self.rev_button.move(160, 260)
        self.arm_button.move(75, 320)
        self.disarm_button.move(75, 350)
        self.takeoff_button.move(244, 320)
        self.land_button.move(244, 350)

        # assign onclick callbacks
        self.up_button.clicked.connect(self.up_clicked)
        self.down_button.clicked.connect(self.down_clicked)
        self.right_button.clicked.connect(self.right_clicked)
        self.left_button.clicked.connect(self.left_clicked)
        self.rotate_left_button.clicked.connect(self.rotate_left_clicked)
        self.rotate_right_button.clicked.connect(self.rotate_right_clicked)
        self.fwd_button.clicked.connect(self.fwd_clicked)
        self.rev_button.clicked.connect(self.rev_clicked)
        self.arm_button.clicked.connect(self.arm_clicked)
        self.disarm_button.clicked.connect(self.disarm_clicked)
        self.takeoff_button.clicked.connect(self.takeoff_clicked)
        self.land_button.clicked.connect(self.land_clicked)

        self.widget.show()
        sys.exit(self.app.exec_())

    def up_clicked(self):
        # get state and increment
        state = self.client.getMultirotorState()
        old_x = state.kinematics_estimated.position.x_val
        old_y = state.kinematics_estimated.position.y_val
        new_z = state.kinematics_estimated.position.z_val - self.inc_up_down

        #  move to new state
        print("Moving Up!")
        self.client.moveToPositionAsync(old_x, old_y, new_z, self.travel_speed).join()

    def down_clicked(self):
        # get state and increment
        state = self.client.getMultirotorState()
        old_x = state.kinematics_estimated.position.x_val
        old_y = state.kinematics_estimated.position.y_val
        new_z = state.kinematics_estimated.position.z_val + self.inc_up_down

        #  move to new state
        print("Moving Down!")
        self.client.moveToPositionAsync(old_x, old_y, new_z, self.travel_speed).join()

    def left_clicked(self):
        # get state and increment
        state = self.client.getMultirotorState()
        old_x = state.kinematics_estimated.position.x_val
        new_y = state.kinematics_estimated.position.y_val - self.inc_left_right
        old_z = state.kinematics_estimated.position.z_val

        #  move to new state
        print("Moving Left!")
        self.client.moveToPositionAsync(old_x, new_y, old_z, self.travel_speed).join()

    def right_clicked(self):
        # get state and increment
        state = self.client.getMultirotorState()
        old_x = state.kinematics_estimated.position.x_val
        new_y = state.kinematics_estimated.position.y_val + self.inc_left_right
        old_z = state.kinematics_estimated.position.z_val

        #  move to new state
        print("Moving Right!")
        self.client.moveToPositionAsync(old_x, new_y, old_z, self.travel_speed).join()

    def rotate_left_clicked(self):
        # get current state as quaternion
        state = self.client.getMultirotorState()

        # extract components
        w = state.kinematics_estimated.orientation.w_val
        x = state.kinematics_estimated.orientation.x_val
        y = state.kinematics_estimated.orientation.y_val
        z = state.kinematics_estimated.orientation.z_val

        # create rotation object
        rot = Rotation.from_quat([x,y,z,w])

        # convert to euler angles
        res = rot.as_euler("xyz", degrees=True)

        # extract yaw
        [x_e, y_e, z_e] = res

        z_e -= self.inc_left_right_theta

        # rotate to new yaw
        print("Rotating Left!")
        time.sleep(10)
        self.client.rotateToYawAsync(z_e).join()

    def rotate_right_clicked(self):
        # get current state as quaternion
        state = self.client.getMultirotorState()

        # extract components
        w = state.kinematics_estimated.orientation.w_val
        x = state.kinematics_estimated.orientation.x_val
        y = state.kinematics_estimated.orientation.y_val
        z = state.kinematics_estimated.orientation.z_val

        # create rotation object
        rot = Rotation.from_quat([x, y, z, w])

        # convert to euler angles
        res = rot.as_euler("xyz", degrees=True)

        # extract yaw
        [x_e, y_e, z_e] = res

        z_e += self.inc_left_right_theta

        # rotate to new yaw
        print("Rotating Right!")
        time.sleep(10)
        self.client.rotateToYawAsync(z_e).join()

    def fwd_clicked(self):
        # get state and increment
        state = self.client.getMultirotorState()
        new_x = state.kinematics_estimated.position.x_val + self.inc_fwd_rev
        old_y = state.kinematics_estimated.position.y_val
        old_z = state.kinematics_estimated.position.z_val

        #  move to new state
        print("Moving Forward!")
        self.client.moveToPositionAsync(new_x, old_y, old_z, self.travel_speed).join()

    def rev_clicked(self):
        # get state and increment
        state = self.client.getMultirotorState()
        new_x = state.kinematics_estimated.position.x_val - self.inc_fwd_rev
        old_y = state.kinematics_estimated.position.y_val
        old_z = state.kinematics_estimated.position.z_val

        #  move to new state
        print("Moving Backwards!")
        self.client.moveToPositionAsync(new_x, old_y, old_z, self.travel_speed).join()

    def arm_clicked(self):
        # arm drone
        print("arming drone...")
        self.client.armDisarm(True)

    def disarm_clicked(self):
        # disarm drone
        print("disarming drone...")
        self.client.armDisarm(False)

    def takeoff_clicked(self):
        print("taking off...")
        self.client.takeoffAsync().join()

    def land_clicked(self):
        print("landing...")
        self.client.landAsync().join()

if(__name__ == "__main__"):
    app = DroneControlApp()

    # initize drone control
    app.init_drone()

    # create window
    app.create_window()