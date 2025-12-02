from servo_driver import ServoDriver

class PenControl:
    """
    Controls the pen up/down state. Class keeps track of current pen positions.
    """
    
    def __init__(self, servo_driver: ServoDriver):
        """
        Initialize the pen control with a servo

        Args:
            servo_driver (ServoDriver): Instance of a servo driver
        """
        
        self.servo_driver = servo_driver
        self.pen_down = False
    
    def update(self, down_state: bool):
        """
        Updates the pen state to the desired position. Only move if the pen position is different

        Args:
            down_state (bool): State the pen is current in. True means down, False is up
        """
        
        if down_state != self.pen_down:
            self.servo_driver.set_pen(down_state)
            self.pen_down = down_state
