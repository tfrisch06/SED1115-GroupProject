from ads1x15 import ADS1015
from machine import ADC, Pin, I2C
import time

MIN_VOLTAGE_ELBOW = 0.2
MAX_VOLTAGE_ELBOW = 3.156
MIN_ANGLE_ELBOW = 9.054546
MAX_ANGLE_ELBOW = 173.3455
MIN_VOLTAGE_SHOULDER = 0.162
MAX_VOLTAGE_SHOULDER = 3.176
MIN_ANGLE_SHOULDER = 10.69091
MAX_ANGLE_SHOULDER = 172.1455

class InputReader:
    def __init__(self, x_pin=26, y_pin=27, switch_pin=22):
        self.adc_x = ADC(x_pin)
        self.adc_y = ADC(y_pin)
        self.button = Pin(switch_pin, Pin.IN, Pin.PULL_DOWN)
        
        self.last_button_state = 0
        self.pen_down = False
        self.debounce_time = 0.05
        
        i2c = I2C(1, sda=Pin(14), scl=Pin(15))
        self.adc = ADS1015(i2c, address=0x48, gain=1)

    def read_pots(self):
        x_val = self.adc_x.read_u16() / 65535
        y_val = self.adc_y.read_u16() / 65535
        
        return round(x_val, 2), round(y_val, 2)
                                      
    def read_button(self):
        current_state = self.button.value()
        
        if current_state == 0 and self.last_button_state == 1:
            time.sleep(self.debounce_time)
            if self.button.value() == 0:
                self.pen_down = not self.pen_down
                
        self.last_button_state = current_state
        return self.pen_down

    def read_feedback(self):
        channel_0 = self.adc.read(rate=4, channel1=0)
        channel_1 = self.adc.read(rate=4, channel1=1)
        
        shoulder_v = self.adc.raw_to_v(channel_0)
        elbow_v = self.adc.raw_to_v(channel_1)
        
        shoulder = (shoulder_v / 3.3) * 180
        elbow = (elbow_v / 3.3) * 180
        
        return shoulder, elbow
    
    def voltage_to_angle(self, v, vmin, vmax, amin, amax):
        if v < vmin: 
            v = vmin
        if v > vmax:
            v = vmax

        return amin + ( (v - vmin) / (vmax - vmin) ) * (amax - amin)
    
    def read_feedback_new(self):
        channel_0 = self.adc.read(rate=4, channel1=0)
        channel_1 = self.adc.read(rate=4, channel1=1)
        
        shoulder_v = self.adc.raw_to_v(channel_0)
        elbow_v = self.adc.raw_to_v(channel_1)

        shoulder_angle = self.voltage_to_angle(
            shoulder_v,
            MIN_VOLTAGE_SHOULDER,
            MAX_VOLTAGE_SHOULDER,
            MIN_ANGLE_SHOULDER,
            MAX_ANGLE_SHOULDER
        )

        elbow_angle = self.voltage_to_angle(
            elbow_v,
            MIN_VOLTAGE_ELBOW,
            MAX_VOLTAGE_ELBOW,
            MIN_ANGLE_ELBOW,
            MAX_ANGLE_ELBOW
        )

        return shoulder_angle, elbow_angle

