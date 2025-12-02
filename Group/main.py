from time import sleep
from input_reader import InputReader
from servo_driver import ServoDriver
from inverse_kinematics import inverse_kinematics, forward_kinematics
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
        sa = reader.voltage_to_angle(
            3.3 - s,
            0,
            3.3,
            0,
            180
        )

        ea = reader.voltage_to_angle(
            3.3 - e,
            0,
            3.3,
            0,
            180
        )
        
        fs, fe = forward_kinematics(sa, ea)
        ks, ke = inverse_kinematics(fs, fe)
        
        print(f"SV: {s:.4f}, EV: {e:.4f} | AngleS: {sa:.4f}, AngleE: {ea:.4f} | SK: {ks}, EK: {ke} | FS: {fs}, FE: {fe}")
        
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
            # servos.move_arm_new(shoulder_angle, elbow_angle)

            # Update pen
            pen.update(pen_state)

            sleep(0.01)
    except KeyboardInterrupt:
        print("Return to home")
        move_to_home(servos)

def test():
    servos = ServoDriver()
    reader = InputReader()
    
    for (i, j) in [(0,0), (0, 1), (1, 1), (1, 0)]:
        s, e = inverse_kinematics(i, j)
        servos.move_arm(s, e)
        
        sleep(1.5)
        print(reader.read_feedback())
        sleep(0.5)
        
if __name__ == "__main__":
    main()
    # manually_measure()
    # test()
