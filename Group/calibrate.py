import math
import time
from machine import Pin
from input_reader import InputReader
from servo_driver import ServoDriver
from pen_control import PenControl

# --- IMPORT CONSTANTS FROM YOUR CODE ---
# We need these to calculate the "Pure Math" angles
PAPER_WIDTH = 215   
PAPER_HEIGHT = 279   
SHOULDER_X = -50
SHOULDER_Y = PAPER_HEIGHT / 2
L1 = 155
L2 = 157

button = Pin(10, Pin.IN, Pin.PULL_DOWN)

servos = ServoDriver()
pen = PenControl(servos)

def get_geometric_angles(x_norm, y_norm):
    """
    Calculates the PURE geometric angles (no calibration) for a target.
    This is the 'x' in our regression (Theoretical Angle).
    """
    # 1. Coordinate Scaling (Matches your inverse_kinematics logic)
    cx = (1 - x_norm) * PAPER_WIDTH
    cy = (1 - y_norm) * PAPER_HEIGHT
    
    # 2. Shift to Shoulder Frame
    dx = cx - SHOULDER_X
    dy = cy - SHOULDER_Y
    
    # 3. Law of Cosines
    cos_t2 = (dx**2 + dy**2 - L1**2 - L2**2) / (2 * L1 * L2)
    cos_t2 = min(1, max(-1, cos_t2))
    
    # Note: Using POSITIVE sqrt for Elbow Down as discussed
    sin_t2 = math.sqrt(max(0.0, 1.0 - cos_t2**2))
    theta2 = math.atan2(sin_t2, cos_t2)
    
    k1 = L1 + L2 * cos_t2
    k2 = L2 * sin_t2
    theta1 = math.atan2(dy, dx) - math.atan2(k2, k1)
    
    deg_theta1 = math.degrees(theta1)
    deg_theta2 = math.degrees(theta2)
    
    # Force angles to be positive (0 to 180)  
    while deg_theta1 > 180:
        deg_theta1 -= 360
    while deg_theta1 < -180:
        deg_theta1 += 360
    return deg_theta1, deg_theta2

def linear_regression(x_values, y_values):
    """
    Solves y = Ax + B for calibration constants.
    """
    n = len(x_values)
    sum_x = sum(x_values)
    sum_y = sum(y_values)
    sum_xy = sum(x * y for x, y in zip(x_values, y_values))
    sum_xx = sum(x * x for x in x_values)
    
    # Slope (A)
    numerator = (n * sum_xy) - (sum_x * sum_y)
    denominator = (n * sum_xx) - (sum_x ** 2)
    A = numerator / denominator
    
    # Intercept (B)
    B = (sum_y - A * sum_x) / n
    
    return A, B

def main():
    reader = InputReader()
    servos = ServoDriver()
    
    # The 4 corners we want to calibrate against (Normalized X, Y)
    # Order: Bottom-Left, Bottom-Right, Top-Right, Top-Left
    targets = [
        (0, 0, "Top Right"),
        (1, 0, "Top Left"),
        (1, 1, "Bottom Left"),
        (0, 1, "Bottom Right")
    ]
    
    measured_shoulder = []
    measured_elbow = []
    
    theo_shoulder = []
    theo_elbow = []
    
    print("=== BRACHIOGRAPH CALIBRATION MODE ===")
    print("Use the potentiometers to manually move the pen to the target corner.")
    print("Press the button when the pen is perfectly on the corner.")
    print("-------------------------------------")

    for x_target, y_target, name in targets:
        print(f"\n>>> TARGET: {name}")
        
        # 1. Calculate what the angle SHOULD be (Theoretical)
        s_math, e_math = get_geometric_angles(x_target, y_target)
        theo_shoulder.append(s_math)
        theo_elbow.append(e_math)
        
        # 2. Let user drive the arm manually to find that spot physically
        while True:
            # Direct mapping: Potentiometer 0-1 -> Servo 0-180
            x_pot, y_pot = reader.read_pots()
            pen_state = reader.read_button()
            
            # Map pot (0-1) directly to servo angle (0-180) for manual control
            # Adjust range if your servo can't do full 180 physically
            s_cmd = x_pot * 180
            e_cmd = y_pot * 180
            
            servos.move_arm(s_cmd, e_cmd)
            
            # Check button with debounce (using your class method)
            if button.value():
                # Button pressed! Capture this position.
                print(f"Captured! | Math: ({s_math:.1f}, {e_math:.1f}) | Servo: ({s_cmd:.1f}, {e_cmd:.1f})")
                measured_shoulder.append(s_cmd)
                measured_elbow.append(e_cmd)
                
                # Wait for button release/debounce
                time.sleep(1) 
                break
            
            pen.update(pen_state)
            time.sleep(0.05)
    pen.update(False)    
    

    print("\n\n=== CALCULATION RESULTS ===")
    
    # Calculate Shoulder Constants
    s_A, s_B = linear_regression(theo_shoulder, measured_shoulder)
    
    # Calculate Elbow Constants
    e_A, e_B = linear_regression(theo_elbow, measured_elbow)
    
    print("-" * 30)
    print("Copy these values into inverse_kinematics.py:")
    print("-" * 30)
    print(f"SHOULDER_A = {s_A:.5f}")
    print(f"SHOULDER_B = {s_B:.5f}")
    print(f"ELBOW_A = {e_A:.5f}")
    print(f"ELBOW_B = {e_B:.5f}")
    print("-" * 30)

if __name__ == "__main__":
    main()
