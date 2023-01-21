import math
import rospy
import threading
import tf 
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


# Wheel separation, in meters
WHEEL_SEPARATION = 0.5
# Wheel diameter, in meters
WHEEL_DIAMETER = 0.1
# rpm to power ratio 
MOTOR_RPM2POWER = 1.0 / 320.0
# Conversion factor from meters per second to RPM
RPM_PER_M_PER_S = 60.0 / (3.14159 * WHEEL_DIAMETER)

class diffDriveOdometry():
    def __init__(self):
        # speed
        self.linear = 0
        self.angular = 0
        # position
        self.heading = 0
        self.x = 0
        self.y = 0
        # last update stamp
        self._timestamp = rospy.Time.now()

    def update(self, leftSpeed, rightSpeed, time):
        """ based on odometry diff drive from ros_control """
        # Compute linear and angular diff:
        linear  = (rightSpeed + leftSpeed) * 0.5 
        angular = (rightSpeed - leftSpeed) / WHEEL_SEPARATION

        self.linear = linear
        self.angular = angular

        # Integrate odometry
        dt = (time - self._timestamp).to_sec()
        self._integrateExact(linear * dt, angular * dt)
        self._timestamp = time

        return self.getOdometryMsg()


    def _integrateExact(self, linear, angular):
        if abs(angular) < 1e-6:
            # Runge-Kutta 2nd order integration
            direction = self.heading + angular * 0.5
            self.heading += angular
            self.x += linear * math.cos(direction)
            self.y += linear * math.sin(direction)
        else:
            # exact integration
            heading_old = self.heading
            r = linear / angular
            self.heading += angular
            self.x +=  r * (math.sin(self.heading) - math.sin(heading_old))
            self.y += -r * (math.cos(self.heading) - math.cos(heading_old))

    def getOdometryMsg(self):
        orientation = tf.transformations.quaternion_from_euler(0, 0, self.heading)

        odom = Odometry()
        odom.header.stamp = self._timestamp
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.x = orientation[0]
        odom.pose.pose.orientation.y = orientation[1]
        odom.pose.pose.orientation.z = orientation[2]
        odom.pose.pose.orientation.w = orientation[3]
        odom.twist.twist.linear.x = self.linear
        odom.twist.twist.angular.z = self.angular
        
        return odom

class speedController():
    def __init__(self):
        ## Get parameters 
        # motor update frequency
        self._setspeed_frequency = rospy.get_param("~cmd_vel_frequency", 5)
        # time without receiving twist will stop motors
        self._setspeed_timeout = rospy.get_param("~cmd_vel_timeout", 3.0)
        # compute odometry based on open loop
        self._openLoop_odometry = rospy.get_param("~open_loop", False)
        # don't connect to wheels
        self._wheels_driver = rospy.get_param("~wheels_driver", "")

        # debug parameters
        rospy.loginfo(f"Wheels open loop odometry estimation: {self._openLoop_odometry}")
        rospy.loginfo(f"Wheels set speed timeout: {self._setspeed_timeout}")
        rospy.loginfo(f"Wheels set speed frequency: {self._setspeed_frequency}")
        
        if self._wheels_driver == "rpi_JGB37545":
            # todo: move to object handler
            # initialize motor speed to 0 and open io
            self._initializeMotors()
        else:
            rospy.logwarn("No wheels controller was specified, setting openloop odometry estimation")
            self._openLoop_odometry = True
        # initialize variables
        self._lastExternalUpdate = rospy.Time.now()
        self._brakeMotors()
        # initialize wheels odometry estimator
        self.odomHandler = diffDriveOdometry()

        # start the update speed on a timer at an updated fixed frequency
        self._lock = threading.Lock()
        self.updateMotorsTimer = rospy.Timer(rospy.Duration(1.0/self._setspeed_frequency), self._updateMotorsPower)

        # start subscribers and publisher
        self._cmd_velSub = rospy.Subscriber("cmd_vel", Twist, self.twistCb)
        self._odomPub = rospy.Publisher('odom', Odometry, queue_size = 1)


    def __del__(self):
        rospy.loginfo("Called speed controller destructor, stopping motors")
        self.updateMotorsTimer.shutdown()
        # stops and closes io 
        if self._wheels_driver == "rpi_JGB37545":
            self._closeMotors()

    def _initializeMotors(self):
        from bts7960 import rpi_JGB37545
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

    def _closeMotors(self):
        self._leftMObj.stop()
        self._rightMObj.stop()
        del self._leftMObj
        del self._rightMObj

    def _brakeMotors(self):
        self.leftSpeed = 0
        self.rightSpeed = 0

    def _updateMotorsPower(self, event):
        time = rospy.Time.now()
        # make sure no one is changing externaly the speed
        with self._lock:
            if (time - self._lastExternalUpdate) > rospy.Duration(self._setspeed_timeout):
                self._brakeMotors()
            
            leftMPower = min(max(self.leftSpeed*MOTOR_RPM2POWER, -1.0), 1.0)
            rightMPower = min(max(self.rightSpeed*MOTOR_RPM2POWER, -1.0), 1.0)
        
        rospy.logwarn(f"Motor power: left: {leftMPower:.2f}; right: {rightMPower:.2f}")
        
        if self._wheels_driver == "rpi_JGB37545":
            self._leftMObj.set_motor_power(leftMPower)
            self._rightMObj.set_motor_power(rightMPower)
            
            leftSpeed = self._leftMObj.get_wheel_speed()
            rightSpeed = self._rightMObj.get_wheel_speed()

        if self._openLoop_odometry:
            odometry = self.odomHandler.update(self.leftSpeed / RPM_PER_M_PER_S, self.rightSpeed / RPM_PER_M_PER_S, time)
            rospy.loginfo(f"Open Loop Speed: left: {self.leftSpeed:.2f}; right: {self.rightSpeed:.2f}")
        else:
            odometry = self.odomHandler.update(leftSpeed, rightSpeed, time)
            rospy.loginfo(f"Measured Motor speed: left: {leftSpeed:.2f}; right: {rightSpeed:.2f}")
        
        self._odomPub.publish(odometry)
        
    def setWheelsSpeed(self, left_speed, right_speed):
        #rospy.logwarn(f"Motor speed command: left: {left_speed:.2f}; right: {right_speed:.2f}")
        # convert from speed to power
        with self._lock:
            self.leftSpeed = left_speed
            self.rightSpeed = right_speed
            self._lastExternalUpdate = rospy.Time.now()

    def twistCb(self, msg): 
        
        linear_velocity = msg.linear.x
        angular_velocity = msg.angular.z

        # check if commanded frequency it's too high but it is not a stop command
        if ((rospy.Time.now() - self._lastExternalUpdate) < rospy.Duration(1/(self._setspeed_frequency)) and
            not (linear_velocity == 0 and angular_velocity == 0)):
            return
        # Get the linear and angular velocities from the Twist message
        rospy.loginfo(f"Received new twist message: linear_x: {linear_velocity:.2f}; angular_z: {angular_velocity:.2f}")

        # Calculate the individual wheel velocities
        left_velocity = (linear_velocity - angular_velocity * WHEEL_SEPARATION / 2 )
        right_velocity = (linear_velocity + angular_velocity * WHEEL_SEPARATION / 2 )        
        
        # Convert the velocities from meters per second to RPM
        left_velocity_rpm = left_velocity * RPM_PER_M_PER_S
        right_velocity_rpm = right_velocity * RPM_PER_M_PER_S

        self.setWheelsSpeed(left_velocity_rpm, right_velocity_rpm)
