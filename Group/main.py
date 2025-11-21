from time import sleep
from input_reader import InputReader
from servo_driver import ServoDriver
from inverse_kinematics import inverse_kinematics
from pen_control import PenControl

HOME_X = 0.5
HOME_Y = 0.2

def move_to_home(servos):
    shoulder_angle, elbow_angle = inverse_kinematics(HOME_X, HOME_Y)
    servos.move_arm(shoulder_angle, elbow_angle)
    sleep(1) 

def main():
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
            
            # f1, f2 = reader.read_feedback()
            # print(shoulder_angle, elbow_angle, " | ", round(f1, 2), round(f2, 2))
            
            # Move arm
            servos.move_arm(shoulder_angle, elbow_angle)

            # Update pen
            pen.update(pen_state)

            sleep(0.05)
            
        # Test Corners
        # for (i, j) in [(0, 0), (1, 0), (0, 1), (1, 1)]:
        #     shoulder_angle, elbow_angle = inverse_kinematics(i, j)
            
        #     f1, f2 = reader.read_feedback()
        #     servos.move_arm(shoulder_angle, elbow_angle)
        #     print(i, j, " | ", round(shoulder_angle, 2), round(elbow_angle, 2),  " | ", round(f1, 2), round(f2, 2))
            
        #     sleep(2)
    except KeyboardInterrupt:
        print("Return to home")
        move_to_home(servos)

if __name__ == "__main__":
    main()
