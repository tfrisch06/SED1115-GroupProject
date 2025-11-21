from ads1x15 import ADS1015
from machine import ADC, Pin, I2C
import time

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