class PenControl:
    def __init__(self, servo_driver):
        self.servo_driver = servo_driver
        self.pen_down = False

    def update(self, down_state):
        if down_state != self.pen_down:
            self.servo_driver.set_pen(down_state)
            self.pen_down = down_state
