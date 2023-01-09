print(" executing speedController.py")
import rospy
from geometry_msgs.msg import Twist
from bts7960 import rpi_JGB37545

# Wheel separation, in meters
WHEEL_SEPARATION = 0.5
# Wheel diameter, in meters
WHEEL_DIAMETER = 0.1
# rpm to power ratio 
MOTOR_RPM2POWER = 1.0 / 320.0
# Conversion factor from meters per second to RPM
RPM_PER_M_PER_S = 60.0 / (3.14159 * WHEEL_DIAMETER)

class speedController():
    def __init__(self):
        self._cmd_velSub = rospy.Subscriber("cmd_vel", Twist, self.twistCb)
        self._initializeMotors()

    def __del__(self):
        print("Called speed controller destructor, stopping motors")
        self._stopMotors()

    def _initializeMotors(self):
        self._leftMPower = 0
        self._rightMPower = 0

        self._leftMObj = rpi_JGB37545.motor(hall_sensor=12, 
            bts_L_EN=27, 
            bts_R_EN=13, 
            bts_L_PWM=6, 
            bts_R_PWM=5, 
            wheel_diameter = WHEEL_DIAMETER)
       
        self._rightMObj = rpi_JGB37545.motor(hall_sensor=20, 
                    bts_L_EN=27, 
                    bts_R_EN=16, 
                    bts_L_PWM=26, 
                    bts_R_PWM=19, 
                    wheel_diameter = WHEEL_DIAMETER)
        self._updateMotorsPower()

    def _stopMotors(self):
        self._leftMObj.stop()
        self._rightMObj.stop()

    def _updateMotorsPower(self):
        self._leftMObj.set_motor_power(self._leftMPower)
        self._rightMObj.set_motor_power(self._rightMPower)

    def setWheelsSpeed(self, left_speed, right_speed):
        rospy.logwarn(f"Motor speed: left: {left_speed:.2f}; right: {right_speed:.2f}")
        # convert from speed to power
        self._leftMPower = min(max(left_speed*MOTOR_RPM2POWER, -1.0), 1.0)
        self._rightMPower = min(max(right_speed*MOTOR_RPM2POWER, -1.0), 1.0)
        #self._updateMotorsPower()
        rospy.logwarn(f"Motor power: left: {self._leftMPower:.2f}; right: {self._rightMPower:.2f}")

    def twistCb(self, msg):
        rospy.logdebug(f"Received new twist message: linear_x: {msg.linear.x:.2f}; angular_z: {msg.angular.z:.2f}")
        # Get the linear and angular velocities from the Twist message
        linear_velocity = msg.linear.x
        angular_velocity = msg.angular.z

        # Calculate the individual wheel velocities
        left_velocity = (linear_velocity - angular_velocity * WHEEL_SEPARATION / 2 )
        right_velocity = (linear_velocity + angular_velocity * WHEEL_SEPARATION / 2 )        
        # Convert the velocities from meters per second to RPM
        left_velocity_rpm = left_velocity * RPM_PER_M_PER_S
        right_velocity_rpm = right_velocity * RPM_PER_M_PER_S

        self.setWheelsSpeed(left_velocity_rpm, right_velocity_rpm)
