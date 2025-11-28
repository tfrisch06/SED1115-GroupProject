from time import sleep
from input_reader import InputReader
from servo_driver import ServoDriver
from inverse_kinematics import inverse_kinematics
from pen_control import PenControl

HOME_X = 0.5
HOME_Y = 0.2

PAPER_WIDTH = 215  # 215 mm
PAPER_HEIGHT = 295   # 295 mm

# SHOULDER_OFFSET = 35.0
SHOULDER_OFFSET = 128
SHOULDER_DIRECTION = -1

# ELBOW_OFFSET = 170.0
ELBOW_OFFSET = 128
ELBOW_DIRECTION = 1

def move_to_home(servos):
    shoulder_angle, elbow_angle = inverse_kinematics(HOME_X, HOME_Y)
    servos.move_arm(shoulder_angle, elbow_angle)
    sleep(1) 

def test():
    servos = ServoDriver()

    servos.move_arm(180-52, 180-52)
    sleep(0.5)

def main():
    reader = InputReader()
    servos = ServoDriver()
    pen = PenControl(servos)

    # print("Testing servo corners...")
    # test_corners(servos)
    # sleep(1)

    # print("Testing (0,0)")
    # servos.move_arm(0,0)
    # sleep(1)
    
    print("Starting Brachiograph control...")

    try:
        while True:
            # Read inputs
            x_val, y_val = reader.read_pots()

            x_pos = max(0, min(PAPER_WIDTH, x_val * PAPER_WIDTH))
            y_pos = max(0, min(PAPER_HEIGHT, y_val * PAPER_HEIGHT))
            # print(f"x: {x_pos}, y: {y_pos}")

            pen_state = reader.read_button()
            
            # Compute servo angles from inverse kinematics
            shoulder_angle, elbow_angle = inverse_kinematics(x_pos, y_pos)
            # print(f"Shoulder angle: {shoulder_angle}, Elbow_angle: {elbow_angle}")

            if shoulder_angle is None or elbow_angle is None:
                continue

            shoulder_angle = (shoulder_angle * SHOULDER_DIRECTION) + SHOULDER_OFFSET
            elbow_angle = (elbow_angle * ELBOW_DIRECTION) + ELBOW_OFFSET

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
    # test()
