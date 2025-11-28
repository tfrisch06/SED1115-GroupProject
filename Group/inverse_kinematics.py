from math import sqrt, acos, atan2, degrees

# (0, 0) = (153.27, 132.22)
# (1, 0) = (77.56, 49.2)
# (1, 1) = (29.78, 49.09)
# (0, 1) = (33.49, 131.67)

La = 155
Lb = 155

PAPER_WIDTH = 215  # 215 mm
PAPER_HEIGHT = 295   # 295 mm

def inverse_kinematics(x, y):
    # New sol
    A_x = -50
    A_y = PAPER_HEIGHT/2

    dx = x - A_x
    dy = y - A_y

    AC = sqrt(dx**2 + dy**2)

    if AC > (La + Lb) or AC < abs(La - Lb):
        print("unreachable")
        return None, None
    
    try:
        cos_BAC = ((La**2 + AC**2 - Lb**2)/(2*La*AC))
        cos_BAC = max(-1.0, min(1.0, cos_BAC))
        angle_BAC = acos(cos_BAC)

        cos_ACB = ((Lb**2 + AC**2 - La**2)/(2*Lb*AC))
        cos_ACB = max(-1.0, min(1.0, cos_ACB))
        angle_ACB = acos(cos_ACB)
    except ValueError:
        print("Math Error: Domain error in acos")
        return None, None

    angle_YAC = atan2(dy, dx)

    alpha = degrees(angle_YAC - angle_BAC)
    beta = degrees(-(angle_BAC + angle_ACB))
    
    return alpha, beta