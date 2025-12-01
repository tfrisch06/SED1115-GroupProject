import math

L1 = 155
L2 = 157

PAPER_WIDTH = 215  # 215 mm
PAPER_HEIGHT = 279   # 279 mm

SHOULDER_X = -50
SHOULDER_Y = PAPER_HEIGHT / 2

SHOULDER_A = 1
SHOULDER_B = 146
ELBOW_A = -1.1
ELBOW_B = 171.11

def inverse_kinematics(x, y):
    cx = (1 - x) * PAPER_WIDTH
    cy = (1 - y) * PAPER_HEIGHT
    
    dx = cx - SHOULDER_X
    dy = cy - SHOULDER_Y
    lac = math.sqrt(dx**2 + dy**2)
    
    max_reach = L1 + L2
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
    
    # shoulder_servo = max(13, min(162, shoulder_servo))
    # elbow_servo = max(1, min(140, elbow_servo))
    
    return shoulder_servo, elbow_servo

def forward_kinematics(shoulder_servo, elbow_servo):
    shoulder_deg = (shoulder_servo - SHOULDER_B) / SHOULDER_A
    elbow_deg = (elbow_servo - ELBOW_B) / ELBOW_A

    theta1 = math.radians(shoulder_deg)
    theta2 = math.radians(elbow_deg)

    x = L1 * math.cos(theta1) + L2 * math.cos(theta1 + theta2)
    y = L1 * math.sin(theta1) + L2 * math.sin(theta1 + theta2)
    
    x_norm = 1 - ((x + SHOULDER_X) / PAPER_WIDTH)
    y_norm = 1 - ((y + SHOULDER_Y) / PAPER_HEIGHT)

    return x_norm, y_norm

