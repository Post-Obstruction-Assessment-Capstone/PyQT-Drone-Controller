import sys
import airsim
from PyQt5.QtWidgets import QApplication, QWidget, QPushButton

class DroneControlApp:
    def __init__(self):
        # set constants for offset movements
        self.inc_left_right = 10 # meters
        self.inc_up_down = 5 # meters
        self.travel_speed = 5 # meters/sec
        self.inc_left_right_theta = 15 #degrees

    def init_drone(self):
        # get airsim client
        self.client = airsim.MultirotorClient()
        self.client.confirmConnection()

        # enable api control
        self.client.enableApiControl(True)

        # arm drone
        print("arming drone...")
        self.client.armDisarm(True)

        print("taking off...")
        self.client.takeoffAsync().join()

    def create_window(self):
        self.app = QApplication(sys.argv)
        self.widget = QWidget()

        # set window geometry and title
        self.widget.setGeometry(50, 50, 420, 320)
        self.widget.setWindowTitle("PyQT Drone Control Application")

        # create buttons
        self.up_button = QPushButton(self.widget)
        self.down_button = QPushButton(self.widget)
        self.right_button = QPushButton(self.widget)
        self.left_button = QPushButton(self.widget)
        self.rotate_left_button = QPushButton(self.widget)
        self.rotate_right_button = QPushButton(self.widget)

        # add text to buttons
        self.up_button.setText("Up")
        self.down_button.setText("Down")
        self.right_button.setText("Strafe Right")
        self.left_button.setText("Strafe Left")
        self.rotate_left_button.setText("Rotate Left")
        self.rotate_right_button.setText("Rotate Right")

        # move buttons into place
        self.up_button.move(160, 32)
        self.right_button.move(244, 64)
        self.left_button.move(75, 64)
        self.down_button.move(160, 96)
        self.rotate_left_button.move(75, 160)
        self.rotate_right_button.move(244, 160)

        # assign onclick callbacks
        self.up_button.clicked.connect(self.up_clicked)
        self.down_button.clicked.connect(self.down_clicked)
        self.right_button.clicked.connect(self.right_clicked)
        self.left_button.clicked.connect(self.left_clicked)
        self.rotate_left_button.clicked.connect(self.rotate_left_clicked)
        self.rotate_right_button.clicked.connect(self.rotate_right_clicked)

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
        pass

    def rotate_right_clicked(self):
        pass

if(__name__ == "__main__"):
    app = DroneControlApp()

    # initize drone control
    app.init_drone()

    # create window
    app.create_window()