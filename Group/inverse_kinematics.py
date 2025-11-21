import math

# (0, 0) = (153.27, 132.22)
# (1, 0) = (77.56, 49.2)
# (1, 1) = (29.78, 49.09)
# (0, 1) = (33.49, 131.67)

L1 = 155
L2 = 155

PAPER_WIDTH = 215  # 215 mm
PAPER_HEIGHT = 279   # 295 mm

SHOULDER_X = -50
SHOULDER_Y = PAPER_HEIGHT / 2

SHOULDER_A = 1
SHOULDER_B = 146
ELBOW_A = -1.1
ELBOW_B = 171.11

def inverse_kinematics(x, y):
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