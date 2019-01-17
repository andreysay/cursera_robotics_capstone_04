#!/usr/bin/env python
import rospy
import yaml
import math
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist
from Adafruit_MotorHAT_PWM_ParallaxMotors import Adafruit_MotorHAT, PWM_Motor

class MotorDriver:
    def __init__(self, motor_gain, wheel_sep, wheel_radius):
        self.test_mode = rospy.get_param("~test_mode",False)
        self._wheel_sep = wheel_sep
        self._wheel_rad = wheel_radius
        self._speed_unit = 0.457 # omega_max/35(wheel velocity width)

        self._mh = Adafruit_MotorHAT(addr = 0x40, freq = 44.5)
        self._motor_left_num = 0
        self._motor_right_num = 1
        self._motor_left = self._mh.getMotor(self._motor_left_num)
	self._motor_left.setMotorSide('left')
        self._motor_right = self._mh.getMotor(self._motor_right_num)
	self._motor_right.setMotorSide('right')
        self._motor_left.run(Adafruit_MotorHAT.RELEASE)
        self._motor_right.run(Adafruit_MotorHAT.RELEASE)

        self.last_msg_time = None

        self.motors_on = False
        rospy.on_shutdown(self.turnOffMotors)
        
    # recommended for auto-disabling motors on shutdown!
    def turnOffMotors(self):
	self._mh.getMotor(self._motor_left_num).run(Adafruit_MotorHAT.RELEASE)
	self._mh.getMotor(self._motor_right_num).run(Adafruit_MotorHAT.RELEASE)
        self.motors_on = False
        
    def drive(self,twist):
        x = twist.linear.x
        w = twist.angular.z
        vel_left  = (x - w * self._wheel_sep / 2.) / self._wheel_rad
        vel_right = (x + w * self._wheel_sep/ 2.) / self._wheel_rad

        #speed_left = vel_left/self._speed_unit
	speed_left = 1.5 *  vel_left
        speed_right = 1.5 * vel_right
	speed_left = max(min(speed_left,25),0)
	speed_right = max(min(speed_right,25),0)
	print 'Speed left: %d' % int(speed_left)
	print 'Speed right: %d' % int(speed_right)
	
        if (speed_left < 0):
	    speed_left = -speed_left
            self._motor_left.run(Adafruit_MotorHAT.BACKWARD, int(speed_left))
        elif (speed_left == 0):
            self._motor_left.brake()
        else:
            self._motor_left.run(Adafruit_MotorHAT.FORWARD, int(speed_left))
        #if pwm_left < 45:
        #    pwm_left = 0
        if (speed_right < 0):
	    speed_right = -speed_right
            self._motor_right.run(Adafruit_MotorHAT.BACKWARD, int(speed_right))
        elif (speed_right == 0):
            self._motor_right.brake()
        else:
            self._motor_right.run(Adafruit_MotorHAT.FORWARD, int(speed_right))

        #if pwm_right < 45:
        #    pwm_right = 0
        
        #print "Final pwm is:"
        #print int(pwm_left), int(pwm_right)
        
        #self._motor_left.setSpeed(int(pwm_left))
        #self._motor_right.setSpeed(int(pwm_right))
	
        self.motors_on = True
        
        #rospy.sleep(1)
        
        self.last_msg_time = rospy.get_rostime()
        

if __name__ == '__main__':
    node = rospy.init_node('motor_driver')

    param_path = rospy.get_param("~param_path")
    f = open(param_path, 'r')
    params_raw = f.read()
    f.close()
    params = yaml.load(params_raw)
    motor_gain = params['motor_gain']
    wheel_sep = params['wheel_sep']
    wheel_radius = params['wheel_radius']

    driver = MotorDriver(motor_gain, wheel_sep, wheel_radius)
    
    rospy.Subscriber('cmd_vel', Twist, driver.drive)

    rate = rospy.Rate(10)

    if driver.test_mode:
        timeout = 1
        rospy.loginfo("[motor_driver]: Test mode on")
    else:
        timeout = 0.1
    
    while not rospy.is_shutdown():
        if driver.last_msg_time is not None and (((rospy.get_rostime() - driver.last_msg_time).to_sec()) > timeout) and driver.motors_on:
            driver.turnOffMotors()
        rate.sleep()
