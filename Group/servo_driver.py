from machine import Pin, PWM
from time import sleep

SERVO_FREQ = 50
MIN_PULSE = 500
MAX_PULSE = 2500

class ServoDriver:
    def __init__(self, shoulder_pin=0, elbow_pin=1, pen_pin=2):
        self.shoulder = PWM(Pin(shoulder_pin))
        self.elbow = PWM(Pin(elbow_pin))
        self.pen = PWM(Pin(pen_pin))
        
        for s in [self.shoulder, self.elbow, self.pen]:
            s.freq(SERVO_FREQ)

    def _angle_to_duty(self, angle):
        angle = max(0, min(180, angle))
        
        pulse = MIN_PULSE + (angle / 180) * (MAX_PULSE - MIN_PULSE)
        duty = int(pulse / 20000 * 65535)
        
        return duty

    def set_angle(self, servo, angle):
        servo.duty_u16(self._angle_to_duty(angle))

    def move_arm(self, shoulder_angle, elbow_angle):
        self.set_angle(self.shoulder, shoulder_angle)
        self.set_angle(self.elbow, elbow_angle)

    def set_pen(self, down):
        angle = 30 if down else 0
        self.set_angle(self.pen, angle)
