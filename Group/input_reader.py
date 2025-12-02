from ads1x15 import ADS1015
from machine import ADC, Pin, I2C
import config as c
import time

class InputReader:
    """
    Reads analog and digital inputs for controlling the robot arms.
    
    Handles potentiometer input for (x, y)
    Button for pen up/down
    Feedback from the servo motors
    """
    
    def __init__(self, x_pin=26, y_pin=27, switch_pin=22):
        """
        Initialize the input reader.

        Args:
            x_pin (int, optional): GPIO pin representing the x-axis potentiometer. Defaults to 26.
            y_pin (int, optional): GPIO pin representing the y-axis potentiometer. Defaults to 27.
            switch_pin (int, optional): GPIO pin representing the pen up/down button. Defaults to 22.
        """
        
        self.adc_x = ADC(x_pin)
        self.adc_y = ADC(y_pin)
        self.button = Pin(switch_pin, Pin.IN, Pin.PULL_DOWN)
        
        self.last_button_state = 0
        self.pen_down = False
        self.debounce_time = 0.05
        
        i2c = I2C(1, sda=Pin(14), scl=Pin(15))
        self.adc = ADS1015(i2c, address=0x48, gain=1)

    def read_pots(self) -> tuple[float, float]:
        """
        Reads the potentiometer input and normalizes it

        Returns:
            tuple[float, float]: Normalized x and y values in 0-1
        """
        x_val = self.adc_x.read_u16() / 65535
        y_val = self.adc_y.read_u16() / 65535
        
        return x_val, y_val
                                      
    def read_button(self) -> bool:
        """
        Read the pen control button and update pen state with a debounce time

        Returns:
            bool: Pen state. True if pen is down, False if up
        """
        
        current_state = self.button.value()
        
        # Detect if button released and toggle pen state
        if current_state == 0 and self.last_button_state == 1:
            # Debounce delay
            time.sleep(self.debounce_time)
            if self.button.value() == 0:
                self.pen_down = not self.pen_down
                
        self.last_button_state = current_state
        return self.pen_down

    def voltage_to_angle(self, v: float, vmin: float, vmax: float, amin: float, amax: float) -> float:
        """
        Converts a given voltage to a corresponding angle. Values outside the voltage range are clamped to either
        minimum or maximum. Max and min voltage and angles must be found through calibration.

        Args:
            v (float): Measured voltage
            vmin (float): Minimum Expected voltage
            vmax (float): Maximum expected voltage
            amin (float): Angle corresponding to vmin
            amax (float): Angle corresponding to vmax

        Returns:
            float: Calculated angle in degrees
        """
        
        # Clamp to valid voltage range
        if v < vmin: 
            v = vmin
        if v > vmax:
            v = vmax

        # Normalizes voltage from 0 - 1
        # Scale to angle range: 0 - (amax-amin)
        # Shift by minimum angle
        return amin + ((v - vmin) / (vmax - vmin)) * (amax - amin)
    
    def read_voltage(self) -> tuple[float, float]:
        """
        Reads the voltage from the feedback channels.

        Returns:
            tuple[float, float]: Shoulder and elbow servo voltage.
        """
        channel_0 = self.adc.read(rate=4, channel1=0)
        channel_1 = self.adc.read(rate=4, channel1=1)
        
        shoulder_v = self.adc.raw_to_v(channel_0)
        elbow_v = self.adc.raw_to_v(channel_1)
        
        return shoulder_v, elbow_v
    
    def read_feedback(self) -> tuple[float, float]:
        """
        Reader servo feedback voltages and convert them to angles
        
        Steps:
        1. Reads raw ADC from the feedback channels
        2. Converts raw to voltage
        3. Maps voltage to angles

        Returns:
            tuple[float, float]: Calculated shoulder and elbow angles in degrees
        """
        
        shoulder_v, elbow_v = self.read_voltage()

        shoulder_angle = self.voltage_to_angle(
            c.MAX_ANGLE_SHOULDER - shoulder_v,
            c.MIN_VOLTAGE_SHOULDER,
            c.MAX_VOLTAGE_SHOULDER,
            c.MIN_ANGLE_SHOULDER,
            c.MAX_ANGLE_SHOULDER
        )

        elbow_angle = self.voltage_to_angle(
            c.MAX_ANGLE_ELBOW - elbow_v,
            c.MIN_VOLTAGE_ELBOW,
            c.MAX_VOLTAGE_ELBOW,
            c.MIN_ANGLE_ELBOW,
            c.MAX_ANGLE_ELBOW
        )

        return shoulder_angle, elbow_angle
