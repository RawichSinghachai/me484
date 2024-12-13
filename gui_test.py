from PyQt6.QtGui import QPixmap
from PyQt6.QtCore import QCoreApplication, Qt , QSize, pyqtSignal,QDate,QTimer
from PyQt6.QtWidgets import QApplication, QWidget, QLabel, QPushButton, QVBoxLayout, \
    QHBoxLayout, QGridLayout, QLineEdit, QMessageBox, QGroupBox, QSpacerItem, QStackedWidget,QMainWindow ,QSlider
from PyQt6.QtGui import QIcon,QImage
from PyQt6.QtGui import QMouseEvent
from time import sleep

from arm_robot_movement import inverse_kinematics, plot_robot_arm
# debug mode
from servo import (configure_servo, move_servo_smoothly, forward_servo360, backward_servo360, stop_servo360, move_servo_linear,stop_servo_at_angle)

class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()

        self.theta1 = 0
        self.theta2 = 0
        self.theta3 = 0
        self.theta4 = 0

        self.theta1_des = 0 
        self.theta2_des = 0 
        self.theta3_des = 0 
        self.theta4_des = 0 

        self.setWindowTitle("QSlider with Value Display")
        self.setFixedSize(QSize(800, 400))

        # Main Layout
        self.layout = QVBoxLayout()
        # self.layout.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.setLayout(self.layout)

        # Top Hbox
        self.hBox_top = QHBoxLayout()
        self.hBox_top.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.layout.addLayout(self.hBox_top)

        # Top Hbox
        self.hBox_bottom = QHBoxLayout()
        self.hBox_bottom.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.layout.addLayout(self.hBox_bottom)


        # vBox1
        self.vBox1 = QVBoxLayout()
        self.hBox_top.addLayout(self.vBox1)

        # vBox2
        self.vBox2 = QVBoxLayout()
        self.hBox_top.addLayout(self.vBox2)

        # vBox3
        self.vBox3 = QVBoxLayout()
        self.hBox_top.addLayout(self.vBox3)

# ----------------------- Configure -------------------------------------------------
        # debug mode
        # configure_servo(0)
        # configure_servo(1)
        # configure_servo(2)
        # configure_servo(3)


# ----------------------- top components ---------------------------------------------
# ----------------------- Box 1 -------------------------------------------------------
# ----------------------- Slider 1 ----------------------------------------------------
        # Create a slider
        self.slider_servo1 = QSlider(Qt.Orientation.Horizontal)  # Horizontal slider
        self.slider_servo1.setMinimum(0)  # Set minimum value
        self.slider_servo1.setMaximum(270)  # Set maximum value
        self.slider_servo1.setValue(135)  # Set initial value (optional)
        self.slider_servo1.valueChanged.connect(self.slider_value_servo1)  # Connect signal

        # Create a label to display the slider's value
        self.label_servo1 = QLabel("Servo 1 : 135")  # Initial label text
        self.label_servo1.setAlignment(Qt.AlignmentFlag.AlignCenter)  # Center align text

        self.vBox1.addWidget(self.slider_servo1)
        self.vBox1.addWidget(self.label_servo1)

# ----------------------- Slider 2 ----------------------------------------------------
        # Create a slider
        self.slider_servo2 = QSlider(Qt.Orientation.Horizontal)  # Horizontal slider
        self.slider_servo2.setMinimum(0)  # Set minimum value
        self.slider_servo2.setMaximum(270)  # Set maximum value
        self.slider_servo2.setValue(135)  # Set initial value (optional)
        self.slider_servo2.valueChanged.connect(self.slider_value_servo2)  # Connect signal

        # Create a label to display the slider's value
        self.label_servo2 = QLabel("Servo 2 : 135")  # Initial label text
        self.label_servo2.setAlignment(Qt.AlignmentFlag.AlignCenter)  # Center align text

        self.vBox1.addWidget(self.slider_servo2)
        self.vBox1.addWidget(self.label_servo2)

# ----------------------- Slider 3 ----------------------------------------------------
        # Create a slider
        self.slider_servo3 = QSlider(Qt.Orientation.Horizontal)  # Horizontal slider
        self.slider_servo3.setMinimum(0)  # Set minimum value
        self.slider_servo3.setMaximum(270)  # Set maximum value
        self.slider_servo3.setValue(135)  # Set initial value (optional)
        self.slider_servo3.valueChanged.connect(self.slider_value_servo3)  # Connect signal

        # Create a label to display the slider's value
        self.label_servo3 = QLabel("Servo 3 : 135")  # Initial label text
        self.label_servo3.setAlignment(Qt.AlignmentFlag.AlignCenter)  # Center align text

        self.vBox1.addWidget(self.slider_servo3)
        self.vBox1.addWidget(self.label_servo3)

# ----------------------- Slider 4 ----------------------------------------------------
        # Create a slider
        self.slider_servo4 = QSlider(Qt.Orientation.Horizontal)  # Horizontal slider
        self.slider_servo4.setMinimum(0)  # Set minimum value
        self.slider_servo4.setMaximum(270)  # Set maximum value
        self.slider_servo4.setValue(135)  # Set initial value (optional)
        self.slider_servo4.valueChanged.connect(self.slider_value_servo4)  # Connect signal

        # Create a label to display the slider's value
        self.label_servo4 = QLabel("Servo 4 : 135")  # Initial label text
        self.label_servo4.setAlignment(Qt.AlignmentFlag.AlignCenter)  # Center align text

        self.vBox1.addWidget(self.slider_servo4)
        self.vBox1.addWidget(self.label_servo4)

# --------------------------------------- Button servo5 -------------------------------------------------------

        self.hBox_button_servo5 = QHBoxLayout()
        self.vBox1.addLayout(self.hBox_button_servo5)

        # Create a button
        self.button_servo5_up = QPushButton("UP")
        self.button_servo5_up.clicked.connect(self.up_button_servo5_click)  # Connect button click signal to a slot
        self.hBox_button_servo5.addWidget(self.button_servo5_up)

        self.button_servo5_down = QPushButton("DOWN")
        self.button_servo5_down.clicked.connect(self.down_button_servo5_click)  # Connect button click signal to a slot
        self.hBox_button_servo5.addWidget(self.button_servo5_down)

        self.button_servo5_stop = QPushButton("STOP")
        self.button_servo5_stop.clicked.connect(self.stop_button_servo5_click)  # Connect button click signal to a slot
        self.hBox_button_servo5.addWidget(self.button_servo5_stop)

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

        self.label_angle_servo1 = QLabel("Servo 1 : 0")
        self.vBox2.addWidget(self.label_angle_servo1)

        self.label_angle_servo2 = QLabel("Servo 2 : 0")
        self.vBox2.addWidget(self.label_angle_servo2)

        self.label_angle_servo3 = QLabel("Servo 3 : 0")
        self.vBox2.addWidget(self.label_angle_servo3)

        self.label_angle_servo4 = QLabel("Servo 4 : 0")
        self.vBox2.addWidget(self.label_angle_servo4)

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

        self.label_angle_servo1_endpoint = QLabel("Servo 1 : 0")
        self.vBox3.addWidget(self.label_angle_servo1_endpoint)

        self.label_angle_servo2_endpoint = QLabel("Servo 2 : 0")
        self.vBox3.addWidget(self.label_angle_servo2_endpoint)

        self.label_angle_servo3_endpoint = QLabel("Servo 3 : 0")
        self.vBox3.addWidget(self.label_angle_servo3_endpoint)

        self.label_angle_servo4_endpoint = QLabel("Servo 4 : 0")
        self.vBox3.addWidget(self.label_angle_servo4_endpoint)

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

# --------------------------- Bottm components ------------------------------------------------------------------------------
# --------------------------- emergency Switch -----------------------------------------------------------------

        self.hBox_emergency_switch = QHBoxLayout()
        self.hBox_bottom.addLayout(self.hBox_emergency_switch)

        self.button_emergency = QPushButton("Emergency")
        self.button_emergency.clicked.connect(self.on_button_emergency)  # Connect button click signal to a slot
        self.hBox_bottom.addWidget(self.button_emergency)

        # Create a button
        self.button_reset = QPushButton("Reset")
        self.button_reset.clicked.connect(self.on_button_reset)  # Connect button click signal to a slot
        self.hBox_bottom.addWidget(self.button_reset)



# -------------------------------------------------------------------------------------------------------------
        container = QWidget()
        container.setLayout(self.layout)
        self.setCentralWidget(container)

# -------------------------- Method ----------------------------------------------------------------

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

    def up_button_servo5_click(self):
        pass
    
    def down_button_servo5_click(self):
        pass
    
    def stop_button_servo5_click(self):
        pass
    
#       when click button start
    def on_button_start_control_mode(self):
        # for debug mode hide

        move_servo_smoothly(0,0,self.theta1,10,0.1)
        sleep(1)
        move_servo_smoothly(1,0,self.theta2,10,0.1)
        sleep(1)
        move_servo_smoothly(2,0,self.theta3,10,0.1)
        sleep(1)
        move_servo_linear()
        sleep(1)
        move_servo_smoothly(2,0,self.theta4,10,0.1)
      
        


#       when click calutation button
    def on_button_calculate(self):
        x = float(self.input_x_axis_calculate.text()) if self.input_x_axis_calculate.text() else 0.0
        y = float(self.input_y_axis_calculate.text()) if self.input_y_axis_calculate.text() else 0.0
        print(f" x-axis : {x} and y-axis : {y}")

        self.theta1, self.theta2, self.theta3 = inverse_kinematics(x,y)

        if self.theta1 == None or self.theta2 == None or self.theta3 == None:
                self.label_angle_servo1.setText(f"Servo 1 : {self.theta1}")
                self.label_angle_servo2.setText(f"Servo 2 : {self.theta2}")
                self.label_angle_servo3.setText(f"Servo 3 : {self.theta3}")
        else:
                self.label_angle_servo1.setText(f"Servo 1 : {self.theta1:.2f}")
                self.label_angle_servo2.setText(f"Servo 2 : {self.theta2:.2f}")
                self.label_angle_servo3.setText(f"Servo 3 : {self.theta3:.2f}")
                plot_robot_arm(x,y)
        

    def on_button_calculate_confirm(self):
        # when press button confirm mode calculate

        # for debug mode hide
        move_servo_smoothly(0,0,self.theta1,10,0.1)
        sleep(1)
        move_servo_smoothly(1,0,self.theta2,10,0.1)
        sleep(1)
        move_servo_smoothly(2,0,self.theta3,10,0.1)
        sleep(1)
        move_servo_linear()
        sleep(1)
        move_servo_smoothly(2,0,self.theta4,10,0.1)
      
    
#       when click calulate button endpoint
    def on_button_start_endpoint(self):
        x_endpoint = float(self.input_x_axis_endpoint.text()) if self.input_x_axis_endpoint.text() else 0.0
        y_endpoint = float(self.input_y_axis_endpoint.text()) if self.input_y_axis_endpoint.text() else 0.0
        
        self.theta1_des, self.theta2_des, self.theta3_des = inverse_kinematics(x_endpoint,y_endpoint)

        if self.theta1_des == None or self.theta2_des == None or self.theta3_des == None:
                self.label_angle_servo1_endpoint.setText(f"Servo 1 : {self.theta1_des}")
                self.label_angle_servo2_endpoint.setText(f"Servo 2 : {self.theta2_des}")
                self.label_angle_servo3_endpoint.setText(f"Servo 3 : {self.theta3_des}")
        else:
                self.label_angle_servo1_endpoint.setText(f"Servo 1 : {self.theta1_des:.2f}")
                self.label_angle_servo2_endpoint.setText(f"Servo 2 : {self.theta2_des:.2f}")
                self.label_angle_servo3_endpoint.setText(f"Servo 3 : {self.theta3_des:.2f}")
                plot_robot_arm(x_endpoint,y_endpoint)

    def on_button_confirm_endpoint(self):
        pass

    def off_button_servo_click(self):
        pass

    def on_button_emergency(self):
        pass
    def on_button_reset(self):
        pass
    def button_reset(self):
        pass


if __name__ == "__main__":
    app = QApplication([])
    window = MainWindow()
    window.show()
    app.exec()
