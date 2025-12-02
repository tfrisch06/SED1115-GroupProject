from machine import Pin, PWM

SERVO_FREQ = 50 # PWM frequency
MIN_PULSE = 500
MAX_PULSE = 2500

class ServoDriver:
    """
    Controls the 3 servos of the brachiograph project. 2 for the arms and 1 for the pen. Can move the arm servos
    and contol the pen up/down position
    """
    
    def __init__(self, shoulder_pin=0, elbow_pin=1, pen_pin=2):
        """
        Initialize PWM channels for shoulder, elbow and pen servos

        Args:
            shoulder_pin (int, optional): GPIO pin connected to the shoulder servo. Defaults to 0.
            elbow_pin (int, optional): GPIO pin connected to the elbow servo. Defaults to 1.
            pen_pin (int, optional): GPIO pin connected to the pin servo. Defaults to 2.
        """
        
        self.shoulder = PWM(Pin(shoulder_pin))
        self.elbow = PWM(Pin(elbow_pin))
        self.pen = PWM(Pin(pen_pin))
        
        self.shoulder_angle = 90
        self.elbow_angle = 90
        
        # Set all servos to the frequency
        for s in [self.shoulder, self.elbow, self.pen]:
            s.freq(SERVO_FREQ)

    def angle_to_duty(self, angle: float) -> int:
        """
        Converts an angle in degrees to a 16 bit PWM cycle

        Args:
            angle (float): 

        Returns:
            int: _description_
        """
        
        angle = max(0, min(180, angle))
        
        pulse = MIN_PULSE + (angle / 180) * (MAX_PULSE - MIN_PULSE)
        
        duty = int(pulse / 20000 * 65535)
        
        return duty

    def set_angle(self, servo: PWM, angle: float):
        servo.duty_u16(self.angle_to_duty(angle))

    def move_arm(self, shoulder_angle: float, elbow_angle: float):
        self.set_angle(self.shoulder, shoulder_angle)
        self.set_angle(self.elbow, elbow_angle)
        
    def set_pen(self, down: bool):
        angle = 30 if down else 0
        self.set_angle(self.pen, angle)

