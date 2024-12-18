from PyQt6.QtGui import QPixmap
from PyQt6.QtCore import QCoreApplication, Qt , QSize, pyqtSignal,QDate,QTimer,QThread
from PyQt6.QtWidgets import QApplication, QWidget, QLabel, QPushButton, QVBoxLayout, \
    QHBoxLayout, QGridLayout, QLineEdit, QMessageBox, QGroupBox, QSpacerItem, QStackedWidget,QMainWindow ,QSlider
from PyQt6.QtGui import QIcon,QImage
from PyQt6.QtGui import QMouseEvent
from time import sleep
from gpiozero import Button

from arm_robot_movement import inverse_kinematics, plot_robot_arm
from servo import (configure_servo, move_servo_smoothly, forward_servo360, backward_servo360, stop_servo360)
from calibration import transform_coordinate

class MainWindow(QWidget):
    def __init__(self):
        super().__init__()

        self.theta1 = 0
        self.theta2 = 0
        self.theta3 = 0
        self.theta4_start = 0
        self.theta4_end = 0

        self.theta1_des = 0 
        self.theta2_des = 0 
        self.theta3_des = 0 
        self.theta4_des_start = 0 
        self.theta4_des_end = 0 

        self.delay_servo = 0.1
        self.step = 1

        self.button_26_state = False
        self.button_27_state = False

        self.setWindowTitle("QSlider with Value Display")
        self.setFixedSize(QSize(800, 400))

        # Main Layout
        self.layout = QVBoxLayout()
        # self.layout.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.setLayout(self.layout)

        # HBox Top
        self.hBox_top = QHBoxLayout()
        self.hBox_top.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.layout.addLayout(self.hBox_top)


        # Hbox Mid
        self.hBox_mid = QHBoxLayout()
        self.hBox_mid.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.layout.addLayout(self.hBox_mid)

        # Hbox Bottom
        self.hBox_bottom = QHBoxLayout()
        self.hBox_bottom.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.layout.addLayout(self.hBox_bottom)


        # vBox1
        self.vBox1 = QVBoxLayout()
        self.hBox_mid.addLayout(self.vBox1)

        # vBox2
        self.vBox2 = QVBoxLayout()
        self.hBox_mid.addLayout(self.vBox2)

        # vBox3
        self.vBox3 = QVBoxLayout()
        self.hBox_mid.addLayout(self.vBox3)

# ----------------------- Configure -------------------------------------------------
        
        configure_servo(0)
        configure_servo(1)
        configure_servo(2)
        configure_servo(4)

        self.button_6 = Button(6, pull_up=True, bounce_time=0.01)  # Reduce debounce time to 10ms
        self.button_5 = Button(5, pull_up=True, bounce_time=0.01)  # Reduce debounce time to 10ms

        self.button_6_state = self.button_6.is_pressed  
        self.button_5_state = self.button_5.is_pressed


# ---------------------- Top components ---------------------------------------------- 
# ---------------------- Input x,y from camera ---------------------------------------

        # layout hBox input calibration workspace
        self.vBox_calibration_workspace = QVBoxLayout()
        self.hBox_top.addLayout(self.vBox_calibration_workspace)

        # Create input field
        self.input_calibration_workspace1 = QLineEdit()
        self.input_calibration_workspace1.setPlaceholderText("coordinate 1")  # Placeholder text
        self.vBox_calibration_workspace.addWidget(self.input_calibration_workspace1)

        # Create input field
        self.input_calibration_workspace2 = QLineEdit()
        self.input_calibration_workspace2.setPlaceholderText("coordinate 2")  # Placeholder text
        self.vBox_calibration_workspace.addWidget(self.input_calibration_workspace2)

        # Create input field
        self.input_calibration_workspace3 = QLineEdit()
        self.input_calibration_workspace3.setPlaceholderText("coordinate 3")  # Placeholder text
        self.vBox_calibration_workspace.addWidget(self.input_calibration_workspace3)

        # Create input field
        self.input_calibration_workspace4 = QLineEdit()
        self.input_calibration_workspace4.setPlaceholderText("coordinate 4")  # Placeholder text
        self.vBox_calibration_workspace.addWidget(self.input_calibration_workspace4)


# ----------------------- layout hBox input coordinate ---------------------------------------------------
        self.vBox_input_coordinate_camera = QVBoxLayout()
        self.hBox_top.addLayout(self.vBox_input_coordinate_camera)

        # Create input field
        self.input_x_coordinate_camera = QLineEdit()
        self.input_x_coordinate_camera.setPlaceholderText("coordinate x")  # Placeholder text
        self.vBox_input_coordinate_camera.addWidget(self.input_x_coordinate_camera)


        # Create input field
        self.input_y_coordinate_camera = QLineEdit()
        self.input_y_coordinate_camera.setPlaceholderText("coordinate y")  # Placeholder text
        self.vBox_input_coordinate_camera.addWidget(self.input_y_coordinate_camera)

        # Create a button
        self.button_find_coordinate = QPushButton("Find")
        self.button_find_coordinate.clicked.connect(self.on_button_find_coordinate)  # Connect button click signal to a slot
        self.vBox_input_coordinate_camera.addWidget(self.button_find_coordinate)

        self.hBox_lable_coordinate = QHBoxLayout()
        self.hBox_lable_coordinate.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.hBox_top.addLayout(self.hBox_lable_coordinate)

        self.label_x_coordinate = QLabel("x : 0")
        self.hBox_lable_coordinate.addWidget(self.label_x_coordinate)

        self.label_y_coordinate = QLabel("y : 0")
        self.hBox_lable_coordinate.addWidget(self.label_y_coordinate)


# ----------------------- Mid components ----------------------------------------------
# ----------------------- Box 1 -------------------------------------------------------
# ----------------------- Slider 1 ----------------------------------------------------
        # Create a slider
        self.slider_servo1 = QSlider(Qt.Orientation.Horizontal)  # Horizontal slider
        self.slider_servo1.setMinimum(0)  # Set minimum value
        self.slider_servo1.setMaximum(270)  # Set maximum value
        self.slider_servo1.setValue(0)  # Set initial value (optional)
        self.slider_servo1.valueChanged.connect(self.slider_value_servo1)  # Connect signal

        # Create a label to display the slider's value
        self.label_servo1 = QLabel("Servo 1 : 0")  # Initial label text
        self.label_servo1.setAlignment(Qt.AlignmentFlag.AlignCenter)  # Center align text

        self.vBox1.addWidget(self.slider_servo1)
        self.vBox1.addWidget(self.label_servo1)

# ----------------------- Slider 2 ----------------------------------------------------
        # Create a slider
        self.slider_servo2 = QSlider(Qt.Orientation.Horizontal)  # Horizontal slider
        self.slider_servo2.setMinimum(0)  # Set minimum value
        self.slider_servo2.setMaximum(270)  # Set maximum value
        self.slider_servo2.setValue(0)  # Set initial value (optional)
        self.slider_servo2.valueChanged.connect(self.slider_value_servo2)  # Connect signal

        # Create a label to display the slider's value
        self.label_servo2 = QLabel("Servo 2 : 0")  # Initial label text
        self.label_servo2.setAlignment(Qt.AlignmentFlag.AlignCenter)  # Center align text

        self.vBox1.addWidget(self.slider_servo2)
        self.vBox1.addWidget(self.label_servo2)

# ----------------------- Slider 3 ----------------------------------------------------
        # Create a slider
        self.slider_servo3 = QSlider(Qt.Orientation.Horizontal)  # Horizontal slider
        self.slider_servo3.setMinimum(0)  # Set minimum value
        self.slider_servo3.setMaximum(270)  # Set maximum value
        self.slider_servo3.setValue(0)  # Set initial value (optional)
        self.slider_servo3.valueChanged.connect(self.slider_value_servo3)  # Connect signal

        # Create a label to display the slider's value
        self.label_servo3 = QLabel("Servo 3 : 0")  # Initial label text
        self.label_servo3.setAlignment(Qt.AlignmentFlag.AlignCenter)  # Center align text

        self.vBox1.addWidget(self.slider_servo3)
        self.vBox1.addWidget(self.label_servo3)

# ----------------------- Slider 4 ----------------------------------------------------
        # Create a slider
        self.slider_servo4 = QSlider(Qt.Orientation.Horizontal)  # Horizontal slider
        self.slider_servo4.setMinimum(0)  # Set minimum value
        self.slider_servo4.setMaximum(270)  # Set maximum value
        self.slider_servo4.setValue(0)  # Set initial value (optional)
        self.slider_servo4.valueChanged.connect(self.slider_value_servo4)  # Connect signal

        # Create a label to display the slider's value
        self.label_servo4 = QLabel("Servo 4 : 0")  # Initial label text
        self.label_servo4.setAlignment(Qt.AlignmentFlag.AlignCenter)  # Center align text

        self.vBox1.addWidget(self.slider_servo4)
        self.vBox1.addWidget(self.label_servo4)

# --------------------------------------- Button servo5 -------------------------------------------------------

        self.hBox_button_servo5 = QHBoxLayout()
        self.vBox1.addLayout(self.hBox_button_servo5)

        # Create a button
        self.button_servo5_up = QPushButton("UP")
        self.button_servo5_up.clicked.connect(self.on_button_up)  # Connect button click signal to a slot
        self.hBox_button_servo5.addWidget(self.button_servo5_up)

        self.button_servo5_down = QPushButton("DOWN")
        self.button_servo5_down.clicked.connect(self.on_button_down)  # Connect button click signal to a slot
        self.hBox_button_servo5.addWidget(self.button_servo5_down)

# ---------------------------------- Button Move ----------------------------------

        self.button_move_control_mode = QPushButton("Start")
        self.button_move_control_mode.clicked.connect(self.on_button_start_control_mode)  # Connect button click signal to a slot
        self.vBox1.addWidget(self.button_move_control_mode)


# -------------------------------------- Box 2 --------------------------------------------------------------------
# -------------------------------------- Input axis ---------------------------------------------------------
        self.hBox_input_axis = QHBoxLayout()
        self.vBox2.addLayout(self.hBox_input_axis)

        # Create input field
        self.input_x_axis_calculate = QLineEdit()
        self.input_x_axis_calculate.setPlaceholderText("X-axis")  # Placeholder text
        self.hBox_input_axis.addWidget(self.input_x_axis_calculate)

        # Create input field
        self.input_y_axis_calculate = QLineEdit()
        self.input_y_axis_calculate.setPlaceholderText("Y-axis")  # Placeholder text
        self.hBox_input_axis.addWidget(self.input_y_axis_calculate)

#  ----------------------------------- Lable Angle -----------------------------------------------------------------

        # Create input field
        self.input_servo4_start_gripper_calulate = QLineEdit()
        self.input_servo4_start_gripper_calulate.setPlaceholderText("Angle Start Griper")  # Placeholder text
        self.vBox2.addWidget(self.input_servo4_start_gripper_calulate)

        self.label_angle_servo1 = QLabel("Servo 1 : 0")
        self.vBox2.addWidget(self.label_angle_servo1)

        self.label_angle_servo2 = QLabel("Servo 2 : 0")
        self.vBox2.addWidget(self.label_angle_servo2)

        self.label_angle_servo3 = QLabel("Servo 3 : 0")
        self.vBox2.addWidget(self.label_angle_servo3)

        # Create input field
        self.input_servo4_end_gripper_calulate = QLineEdit()
        self.input_servo4_end_gripper_calulate.setPlaceholderText("Servo Griper")  # Placeholder text
        self.vBox2.addWidget(self.input_servo4_end_gripper_calulate)

# ---------------------------- Button Calulate ---------------------------------------------------------------

        self.hBox_button_calculate = QHBoxLayout()
        self.vBox2.addLayout(self.hBox_button_calculate)
        # Create a button
        self.button_calculate = QPushButton("Calculate")
        self.button_calculate.clicked.connect(self.on_button_calculate)  # Connect button click signal to a slot
        self.hBox_button_calculate.addWidget(self.button_calculate)

        self.button_confirm_calculate = QPushButton("Confirm")
        self.button_confirm_calculate.clicked.connect(self.on_button_calculate_confirm)  # Connect button click signal to a slot
        self.hBox_button_calculate.addWidget(self.button_confirm_calculate)

#  ---------------------------------- Box 3 -------------------------------------------------------------------
# -------------------------------------- Input axis ---------------------------------------------------------
        self.hBox_input_destination = QHBoxLayout()
        self.vBox3.addLayout(self.hBox_input_destination)


        # Create input field
        self.input_x_axis_endpoint = QLineEdit()
        self.input_x_axis_endpoint.setPlaceholderText("X-axis")  # Placeholder text
        self.hBox_input_destination.addWidget(self.input_x_axis_endpoint)

        # Create input field
        self.input_y_axis_endpoint = QLineEdit()
        self.input_y_axis_endpoint.setPlaceholderText("Y-axis")  # Placeholder text
        self.hBox_input_destination.addWidget(self.input_y_axis_endpoint)

# ---------------------------------- Lable angle Destination ---------------------------------

        # Create input field
        self.input_servo4_start_gripper_endpoint = QLineEdit()
        self.input_servo4_start_gripper_endpoint.setPlaceholderText("Angle Start Griper")  # Placeholder text
        self.vBox3.addWidget(self.input_servo4_start_gripper_endpoint)


        self.label_angle_servo1_endpoint = QLabel("Servo 1 : 0")
        self.vBox3.addWidget(self.label_angle_servo1_endpoint)

        self.label_angle_servo2_endpoint = QLabel("Servo 2 : 0")
        self.vBox3.addWidget(self.label_angle_servo2_endpoint)

        self.label_angle_servo3_endpoint = QLabel("Servo 3 : 0")
        self.vBox3.addWidget(self.label_angle_servo3_endpoint)

        # Create input field
        self.input_servo4_end_gripper_endpoint = QLineEdit()
        self.input_servo4_end_gripper_endpoint.setPlaceholderText("Servo Griper Endpoint")  # Placeholder text
        self.vBox3.addWidget(self.input_servo4_end_gripper_endpoint)

# ---------------------------------- Button Destination ---------------------------------------

        self.hBox_button_destination = QHBoxLayout()
        self.vBox3.addLayout(self.hBox_button_destination)
        # Create a button
        self.button_calculate_destination = QPushButton("Calcutate")
        self.button_calculate_destination.clicked.connect(self.on_button_start_endpoint)  # Connect button click signal to a slot
        self.hBox_button_destination.addWidget(self.button_calculate_destination)

        self.button_comfirm_destination = QPushButton("Confirm")
        self.button_comfirm_destination.clicked.connect(self.on_button_confirm_endpoint)  # Connect button click signal to a slot
        self.hBox_button_destination.addWidget(self.button_comfirm_destination)

# --------------------------- Bottom components ------------------------------------------------------------------------------
# --------------------------- emergency Switch -----------------------------------------------------------------

        self.hBox_emergency_switch = QHBoxLayout()
        self.hBox_bottom.addLayout(self.hBox_emergency_switch)

        # Create a button
        self.button_reset = QPushButton("Reset")
        self.button_reset.clicked.connect(self.on_button_reset)  # Connect button click signal to a slot
        self.hBox_bottom.addWidget(self.button_reset)

        self.button_webcam = QPushButton("OpenWebCam")
        self.button_webcam.clicked.connect(self.on_button_webcam)  # Connect button click signal to a slot
        self.hBox_bottom.addWidget(self.button_webcam)



# -------------------------------------------------------------------------------------------------------------


# -------------------------- Method ----------------------------------------------------------------

    def on_button_find_coordinate(self):
        # when click Find button
        point1 = self.input_calibration_workspace1.text() if self.input_calibration_workspace1.text() else "0,0"
        point2 = self.input_calibration_workspace2.text() if self.input_calibration_workspace2.text() else "0,0"
        point3 = self.input_calibration_workspace3.text() if self.input_calibration_workspace3.text() else "0,0"
        point4 = self.input_calibration_workspace4.text() if self.input_calibration_workspace4.text() else "0,0"

        points = [list(map(int, point1.split(','))), list(map(int, point2.split(','))),list(map(int, point3.split(','))), list(map(int, point4.split(',')))]

        x_camera = int(self.input_x_coordinate_camera.text()) if self.input_x_coordinate_camera.text() else 0
        y_camera = int(self.input_y_coordinate_camera.text()) if self.input_y_coordinate_camera.text() else 0

        x_coordinate , y_coordinate = transform_coordinate(x_camera,y_camera,points)

        self.label_x_coordinate.setText(f"x : {x_coordinate:.2f}")
        self.label_y_coordinate.setText(f"x : {y_coordinate:.2f}")
        
    

    def slider_value_servo1(self, value):
        self.label_servo1.setText(f"Servo 1 : {value}")
        self.theta1 = value

    def slider_value_servo2(self, value):
        self.label_servo2.setText(f"Servo 2 : {value}")
        self.theta2 = value

    def slider_value_servo3(self, value):
        self.label_servo3.setText(f"Servo 3 : {value}")
        self.theta3 = value

    def slider_value_servo4(self, value):
        self.label_servo4.setText(f"Servo 4 : {value}")
        self.theta4 = value

    
#       when click button start
    def on_button_start_control_mode(self):
        # for debug mode hide

        move_servo_smoothly(0,0,self.theta1,self.step,self.delay_servo)
        sleep(1)
        move_servo_smoothly(1,0,self.theta2,self.step,self.delay_servo)
        sleep(1)
        move_servo_smoothly(2,0,self.theta3,self.step,self.delay_servo)
        sleep(1)
        
        while True:
             button_6_state = self.button_6.is_pressed

             backward_servo360(3)
             print("backward")

             if button_6_state:
                  stop_servo360(3)
                  print("Stop")
                  break
             sleep(0.1)

        sleep(2)
        move_servo_smoothly(4,0,self.theta4,self.step,self.delay_servo)
        sleep(2)

        while True:
             button_5_state = self.button_5.is_pressed

             forward_servo360(3)
             print("forward")

             if button_5_state:
                  stop_servo360(3)
                  print("Stop")
                  break
             sleep(0.1)



    def on_button_calculate(self):
#       when click calutation button

        x = float(self.input_x_axis_calculate.text()) if self.input_x_axis_calculate.text() else 0.0
        y = float(self.input_y_axis_calculate.text()) if self.input_y_axis_calculate.text() else 0.0
        print(f" x-axis : {x} and y-axis : {y}")
        
        self.theta1, self.theta2, self.theta3 = inverse_kinematics(x,y+10) # y +10 because center offset 10 cm

        if self.theta1 == None or self.theta2 == None or self.theta3 == None:
                self.theta1 = float(f"{self.theta1}")
                self.theta2 = float(f"{self.theta2}")
                self.theta3 = float(f"{self.theta3}")
        elif self.theta1 > 270 :
                self.theta2 = float(f"{(self.theta1 - 270):.2f}")
                plot_robot_arm(x,y)
        elif self.theta1 < 0 :
                self.theta1 = 270
                self.theta2 = float(f"{(90 + self.theta1)}")
                plot_robot_arm(x,y)
        else:
                self.theta1 = float(f"{self.theta1:.2f}")
                self.theta2 = float(f"{self.theta2:.2f}")
                self.theta3 = float(f"{self.theta3:.2f}")
                plot_robot_arm(x,y)
                

        self.label_angle_servo1.setText(f"Servo 1 : {self.theta1}")
        self.label_angle_servo2.setText(f"Servo 2 : {self.theta2}")
        self.label_angle_servo3.setText(f"Servo 3 : {self.theta3}")
        
        

    def on_button_calculate_confirm(self):
        # when press button confirm mode calculate

        #input_servo4_start_gripper_calulate
        self.theta4_start = float(self.input_servo4_start_gripper_calulate.text()) if self.input_servo4_start_gripper_calulate.text() else 0.0

        self.theta4_end = float(self.input_servo4_end_gripper_calulate.text()) if self.input_servo4_end_gripper_calulate.text() else 0.0

        # for debug mode hide
        move_servo_smoothly(4,0,self.theta4_start,self.step,self.delay_servo)
        sleep(2)
        move_servo_smoothly(0,0,self.theta1,self.step,self.delay_servo)
        sleep(1)
        move_servo_smoothly(1,0,self.theta2,self.step,self.delay_servo)
        sleep(1)
        move_servo_smoothly(2,0,self.theta3,self.step,self.delay_servo)
        sleep(1)

        while True:
             button_6_state = self.button_6.is_pressed

             backward_servo360(3)
             print("backward")

             if button_6_state:
                  stop_servo360(3)
                  print("Stop")
                  break
             sleep(0.1)

        sleep(2)
        move_servo_smoothly(4,self.theta4_start,self.theta4_end,self.step,self.delay_servo)
        sleep(2)

        while True:
             button_5_state = self.button_5.is_pressed

             forward_servo360(3)
             print("forward")

             if button_5_state:
                  stop_servo360(3)
                  print("Stop")
                  break
             sleep(0.1)

        print("confirm calculate")
    
#       when click calulate button endpoint
    def on_button_start_endpoint(self):
        x_endpoint = float(self.input_x_axis_endpoint.text()) if self.input_x_axis_endpoint.text() else 0.0
        y_endpoint = float(self.input_y_axis_endpoint.text()) if self.input_y_axis_endpoint.text() else 0.0

        self.theta1_des, self.theta2_des, self.theta3_des = inverse_kinematics(x_endpoint,y_endpoint)
        self.theta4_des = float(self.input_servo4_end_gripper_endpoint.text()) if self.input_servo4_end_gripper_endpoint.text() else 0.0

        self.theta4 = float(self.input_servo4_end_gripper_calulate.text()) if self.input_servo4_end_gripper_calulate.text() else 0.0

        if self.theta1_des == None or self.theta2_des == None or self.theta3_des == None:
                self.theta1_des = float(f"{self.theta1_des}")
                self.theta2_des = float(f"{self.theta2_des}")
                self.theta3_des = float(f"{self.theta3_des}")
        elif self.theta1_des > 270 :
                self.theta2_des = float(f"{(self.theta1_des - 270):.2f}")
                plot_robot_arm(x_endpoint,y_endpoint)
        elif self.theta1_des < 0 :
                self.theta1_des = 270
                self.theta2_des = float(f"{(90 + self.theta1_des)}")
                plot_robot_arm(x_endpoint,y_endpoint)
        else:
                self.theta1_des = float(f"{self.theta1_des:.2f}")
                self.theta2_des = float(f"{self.theta2_des:.2f}")
                self.theta3_des = float(f"{self.theta3_des:.2f}")
                plot_robot_arm(x_endpoint,y_endpoint)
                

        self.label_angle_servo1.setText(f"Servo 1 : {self.theta1_des}")
        self.label_angle_servo2.setText(f"Servo 2 : {self.theta2_des}")
        self.label_angle_servo3.setText(f"Servo 3 : {self.theta3_des}")   


    def on_button_confirm_endpoint(self):
        
        self.theta4_des_start = float(self.input_servo4_start_gripper_endpoint.text()) if self.input_servo4_start_gripper_endpoint.text() else 0.0
        self.theta4_des_end = float(self.input_servo4_end_gripper_endpoint.text()) if self.input_servo4_end_gripper_endpoint.text() else 0.0

        move_servo_smoothly(0,0,self.theta1_des,self.step,self.delay_servo)
        sleep(1)
        move_servo_smoothly(1,0,self.theta2_des,self.step,self.delay_servo)
        sleep(1)
        move_servo_smoothly(2,0,self.theta3_des,self.step,self.delay_servo)
        sleep(1)
        
        while True:
             button_6_state = self.button_6.is_pressed

             backward_servo360(3)
             print("backward")

             if button_6_state:
                  stop_servo360(3)
                  print("Stop")
                  break
             sleep(0.1)

        sleep(2)
        move_servo_smoothly(4,0,self.theta4_des,self.step,self.delay_servo)
        sleep(2)

        while True:
             button_5_state = self.button_5.is_pressed

             forward_servo360(3)
             print("forward")

             if button_5_state:
                  stop_servo360(3)
                  print("Stop")
                  break
             sleep(0.1)
        

    def off_button_servo_click(self):
        pass

    
    def on_button_reset(self):
        # for debug mode hide
        
        move_servo_smoothly(0,0,self.theta1,self.step,0.1)
        sleep(1)
        move_servo_smoothly(1,0,self.theta2,self.step,0.1)
        sleep(1)
        move_servo_smoothly(2,0,self.theta3,self.step,0.1)
        sleep(1)
        while True:
             button_5_state = self.button_5.is_pressed

             forward_servo360(3)
             print("forward")

             if button_5_state:
                  stop_servo360(3)
                  print("Stop")
                  break
             sleep(0.1)
        sleep(1)
        move_servo_smoothly(4,0,self.theta4,self.step,0.1)
        print("Reset")



    def on_button_up(self):
        while True:
              button_5_state = self.button_5.is_pressed

              forward_servo360(3)
              print("forward")

              if button_5_state:
                   stop_servo360(3)
                   print("Stop")
                   break
              sleep(0.1)

    def on_button_down(self):
        while True:
              button_6_state = self.button_6.is_pressed

              backward_servo360(3)
              print("backward")

              if button_6_state:
                   stop_servo360(3)
                   print("Stop")
                   break
              sleep(0.1)
              
    def on_button_webcam(self):
    # This will start the webcam when the button is clicked
        from vision import main  # Import the main function from vision.py
        main()  # Call the main function to open the webcam


if __name__ == "__main__":
    app = QApplication([])
    window = MainWindow()
    window.show()
    app.exec()
