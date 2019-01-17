#!/usr/bin/env python
import time

from Adafruit_PWM_Servo_Driver import PWM

class PWM_Motor:
    def __init__(self, controller, num):
        # Adafruit HAT object
        self.MC = controller
        if (num > 15):
            raise NameError('MotorHAT Motor must be between 0 and 15 inclusive')        
        self.motornum = num
        self.PWMpin = num
        self.motorSide = {'left':0, 'right':0}
        self.servoTicksMax = {'f_left':320, 'f_right':250, 'b_left':250, 'b_right':320, 'release':285}
        self.motorSpeed = self.servoTicksMax['release']

    def setMotorSide(self, motorSide):
        if motorSide == 'left':
            self.motorSide['left'] = 1
        elif motorSide == 'right':
            self.motorSide['right'] = 1
        else:
            raise NameError('Motor side must be right or left!')
    def brake(self):
        count = 4
        if (self.motorSide['left']):
            while (self.motorSpeed > self.servoTicksMax['release']):
                self.motorSpeed = self.motorSpeed - count
                self.motorSpeed = max(self.motorSpeed, self.servoTicksMax['release'])
                self.MC._pwm.setPWM(self.PWMpin, 0, self.motorSpeed)
            while (self.motorSpeed < self.servoTicksMax['release']):
                self.motorSpeed = self.motorSpeed + count
                self.motorSpeed = min(self.motorSpeed, self.servoTicksMax['release'])
                self.MC._pwm.setPWM(self.PWMpin, 0, self.motorSpeed)
        if (self.motorSide['right']):
            while (self.motorSpeed < self.servoTicksMax['release']):
                self.motorSpeed = self.motorSpeed - count
                self.motorSpeed = max(self.motorSpeed, self.servoTicksMax['release'])
                self.MC._pwm.setPWM(self.PWMpin, 0, self.motorSpeed)
            while (self.motorSpeed > self.servoTicksMax['release']):
                self.motorSpeed = self.motorSpeed + count
                self.motorSpeed = min(self.motorSpeed, self.servoTicksMax['release'])
                self.MC._pwm.setPWM(self.PWMpin, 0, self.motorSpeed)
    def run(self, command, speed=0):
        m_speed = 285
        if not self.MC:
            return
        if not self.motorSide['left'] and not self.motorSide['right']:
            raise ValueError('Motor side should be defined!')
        if (command == Adafruit_MotorHAT.FORWARD and self.motorSide['left']):
            m_speed = 285 + speed
            if m_speed > self.servoTicksMax['f_left']:
                m_speed = 320
            elif m_speed < self.servoTicksMax['release']:
                m_speed = 285
        if (command == Adafruit_MotorHAT.FORWARD and self.motorSide['right']):
            m_speed = 285 - speed
            if m_speed < self.servoTicksMax['f_right']:
                m_speed = 250
            elif m_speed > self.servoTicksMax['release']:
                m_speed = 285
        if (command == Adafruit_MotorHAT.BACKWARD and self.motorSide['left']):
            m_speed = 285 - speed
            if m_speed < self.servoTicksMax['b_left']:
                m_speed = 250
            elif m_speed > self.servoTicksMax['release']:
                m_speed = 285
        if (command == Adafruit_MotorHAT.BACKWARD and self.motorSide['right']):
            m_speed = 285 + speed
            if m_speed > self.servoTicksMax['b_right']:
                speed = 320
            elif m_speed < self.servoTicksMax['release']:
                m_speed = 285                
        if (command == Adafruit_MotorHAT.RELEASE):
            m_speed = 285
            self.MC._pwm.setPWM(self.PWMpin, 0, m_speed)
        self.motorSpeed = m_speed    
        self.MC._pwm.setPWM(self.PWMpin, 0, m_speed)


class Adafruit_MotorHAT:
    FORWARD = 1
    BACKWARD = 2
    RELEASE = 4

    def __init__(self, addr = 0x40, freq = 44.5):
        self._frequency = freq
        self.motors = [ PWM_Motor(self, m) for m in range(16) ]
        self._pwm = PWM(addr, debug=False)
        self._pwm.setPWMFreq(self._frequency)

    def getMotor(self, num):
        if (num < 0) or (num > 15):
            raise NameError('MotorHAT Motor must be between 0 and 15 inclusive')
        return self.motors[num]
