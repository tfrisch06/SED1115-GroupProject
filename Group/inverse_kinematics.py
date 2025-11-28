from math import sqrt, acos, atan2, degrees

# (0, 0) = (153.27, 132.22)
# (1, 0) = (77.56, 49.2)
# (1, 1) = (29.78, 49.09)
# (0, 1) = (33.49, 131.67)

La = 155
Lb = 155

PAPER_WIDTH = 215  # 215 mm
PAPER_HEIGHT = 279   # 295 mm

SHOULDER_X = 0
SHOULDER_Y = 0

SHOULDER_A = 1
SHOULDER_B = 146
ELBOW_A = -1.1
ELBOW_B = 171.11

theta_shoulder_offest = 50
theta_elbow_offest = 160

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
    beta = degrees(angle_BAC + angle_ACB)

    return alpha, beta

    # Attempt at mirroring 
    '''
    A_x = SHOULDER_X
    A_y = SHOULDER_Y
    AC = sqrt( ((A_x - x)**2) + ((A_y - y)**2) )

    if AC == 0 or AC > (La + Lb):
        raise ValueError("Out of bounds")
    
    A_baseC = sqrt( (A_x - x)**2 + y**2 )

    print((La**2 + AC**2 + Lb**2)/(2 * La * AC))
    angle_BAC = acos((La**2 + AC**2 + Lb**2)/(2 * La * AC))
    angle_ACB = asin((La * sin(angle_BAC))/(Lb))
    angle_YAC = acos((A_y**2 + AC**2 - A_baseC**2)/(2 * A_y * AC))

    alpha = degrees(angle_BAC + angle_YAC)
    beta = degrees(angle_BAC + angle_ACB)

    return alpha, beta
    '''

    # Old sol
    '''
    cx = x * PAPER_WIDTH
    cy = y * PAPER_HEIGHT
    
    dx = cx - SHOULDER_X
    dy = cy - SHOULDER_Y
    lac = math.sqrt(dx**2 + dy**2)
    
    max_reach = 155 + 155
    if lac == 0 or lac > max_reach:
        raise ValueError("Outside bounds")
        
    cos_t2 = (dx**2 + dy**2 - L1**2 - L2**2) / (2 * L1 * L2)
    cos_t2 = min(1, max(-1, cos_t2))
    
    sin_t2 = math.sqrt(max(0.0, 1.0 - cos_t2**2))
    theta2 = math.atan2(sin_t2, cos_t2)
    
    k1 = L1 + L2 * cos_t2
    k2 = L2 * sin_t2
    theta1 = math.atan2(dy, dx) - math.atan2(k2, k1)
    
    shoulder_deg = math.degrees(theta1)
    elbow_deg = math.degrees(theta2)
    
    shoulder_servo = SHOULDER_A * shoulder_deg + SHOULDER_B
    elbow_servo = ELBOW_A * elbow_deg + ELBOW_B
    
    shoulder_servo = max(13, min(162, shoulder_servo))
    elbow_servo = max(1, min(140, elbow_servo))
    
    return shoulder_servo, elbow_servo
    '''