from time import sleep
from input_reader import InputReader
from servo_driver import ServoDriver
from inverse_kinematics import inverse_kinematics
from pen_control import PenControl

HOME_X = 0.5
HOME_Y = 0.2

def move_to_home(servos: ServoDriver):
    """
    Upon end of the program move the arms to the middle of the page. Calculates angles required to 
    reach the position at (HOME_X, HOME_Y) and move accordingly.

    Args:
        servos (ServoDriver): Class representing the servo motors
    """
    
    shoulder_angle, elbow_angle = inverse_kinematics(HOME_X, HOME_Y)
    servos.move_arm(shoulder_angle, elbow_angle)
    sleep(1) 
    
def manually_measure():
    """
    Helper function to find max/min voltage for shoulder and elbow servo
    """
    reader = InputReader()
    
    while True:
        s, e = reader.read_voltage()
        print(f"S: {s:.4f}, E: {e:.4f}")

def main():
    """
    Main control loop for the brachiograph project
    
    Steps:
    1. Readers user input from potentiometers and a button
    2. Compute angles corresponding to the potentiometer inputs
    3. Read feedback postions
    4. Move to position requested
    5. Update pen stat if a button has been pressed
    6. Repeat from step 1 
    """
    
    reader = InputReader()
    servos = ServoDriver()
    pen = PenControl(servos)

    print("Starting Brachiograph control...")

    try:
        while True:
            # Read inputs
            x_val, y_val = reader.read_pots()
            pen_state = reader.read_button()
            
            # Compute servo angles from inverse kinematics
            shoulder_angle, elbow_angle = inverse_kinematics(x_val, y_val)

            # Read feedback and report on the current angle
            f1, f2 = reader.read_feedback()
            print(f"Current angles: ({f1:.2f}, {f2:.2f})")
            
            # Move arm
            servos.move_arm(shoulder_angle, elbow_angle)

            # Update pen
            pen.update(pen_state)

            sleep(0.01)
    except KeyboardInterrupt:
        print("Return to home")
        move_to_home(servos)

if __name__ == "__main__":
    main()
    manually_measure()
