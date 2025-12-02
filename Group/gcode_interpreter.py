from machine import Pin, PWM
from servo_driver import ServoDriver
from pen_control import PenControl
from time import sleep

GOTO_TAG = "G1"
SERVO_CONTROL_TAG = "M"

SHOUDER_TAG = "S"
ELBOW_TAG = "E"

servos = ServoDriver()
pen = PenControl(servos)

def read_gcode(file_name, raw_gcode_commands):
    with open(file_name, "r") as f:
        for line in f:
            line = str(line)
            raw_gcode_commands.append(line.strip())

def interpret_gcode(raw_gcode_commands):
    for command in raw_gcode_commands:
        command = str(command)
        if command.startswith(GOTO_TAG):
            # Isolate angles (past tag)
            angles = command[2:].split()

            # Seperate shoulder and elbow angles
            shoulder_angle = None
            elbow_angle = None
            for angle in angles:
                angle = str(angle).strip()
                if angle.startswith(SHOUDER_TAG):
                    # Isolate angle from tag
                    shoulder_angle = float(angle[1:])
                elif angle.startswith(ELBOW_TAG):
                    # Isolate angle from tag
                     elbow_angle = float(angle[1:])
            # catch and display invalid arm movement
            try:
                execute_move_arm(shoulder_angle, elbow_angle)
            except:
                print(f"Invalid command: {command}")
        elif command.startswith(SERVO_CONTROL_TAG):
            # Isolate specific number code from tag
            code = command[1:]

            if code == "3":
                # Places the wrist in the "down" position
                pen.update(1)
            elif code == "5":
                # Places the wrist in the "up" position
                pen.update(0)
            elif code == "18":
                print("disabling servos")
                break
            else:
                print("Unexpected error. You shouldn't be here.")

        sleep(0.5)

def execute_move_arm(shoulder_angle, elbow_angle):
    if ((shoulder_angle is None) or (elbow_angle is None)):
        raise ValueError

    servos.move_arm(shoulder_angle, elbow_angle)
                

input_loop = True

# Allows user to enter a file name
# Checks file is valid before continuing
while input_loop:
    raw_gcode_commands = []
    file_name = input("Enter the file name")

    try:
        read_gcode(file_name, raw_gcode_commands)
    except:
        print(f"Error reading file \"{file_name}\", please try again")
        continue
    input_loop = False

interpret_gcode(raw_gcode_commands)