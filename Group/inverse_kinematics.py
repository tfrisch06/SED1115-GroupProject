import math

# Arm lengths. L2 is slightly longer to adjust for the pen offset
L1 = 155
L2 = 157

# 8.5 x 11 inch paper
PAPER_WIDTH = 215  # 215 mm
PAPER_HEIGHT = 279   # 279 mm

# Location of the shoulder servo
SHOULDER_X = -50
SHOULDER_Y = PAPER_HEIGHT / 2

# Servo calibrations
SHOULDER_A = 1
SHOULDER_B = 146
ELBOW_A = -1.1
ELBOW_B = 171.11

def inverse_kinematics(x: float, y: float) -> tuple[float, float]:
    """
    Calculates the angles (in degrees) required to move to a target position. Target position is represented by 2
    normalized potentiometer inputs (0 to 1), scaled to the paper dimensions. 

    Args:
        x (float): Normalized horizontal postion. (0=left, 1=right)
        y (float): Normalized vertical position. (0=down, 1=up)

    Raises:
        ValueError: If the target position is outside of reach (Should never happen)

    Returns:
        tuple[float, float]: The shoulder and elbow angles (in degrees) calculated
    """

    # Scale normalized input to coordinates    
    cx = (1 - x) * PAPER_WIDTH
    cy = (1 - y) * PAPER_HEIGHT
    
    # Compute position relative to the shoulder
    dx = cx - SHOULDER_X
    dy = cy - SHOULDER_Y
    lac = math.sqrt(dx**2 + dy**2)
    
    # Check if target is reachable
    max_reach = L1 + L2
    if lac == 0 or lac > max_reach:
        raise ValueError("Outside bounds")
        
    # Law of cosines to compute elbow angle 
    cos_t2 = (dx**2 + dy**2 - L1**2 - L2**2) / (2 * L1 * L2)
    
    # Clamp to avoid math domain errors
    cos_t2 = min(1, max(-1, cos_t2))
    sin_t2 = math.sqrt(max(0.0, 1.0 - cos_t2**2))
    theta2 = math.atan2(sin_t2, cos_t2)
    
    # Compute shoulder angle
    k1 = L1 + L2 * cos_t2
    k2 = L2 * sin_t2
    theta1 = math.atan2(dy, dx) - math.atan2(k2, k1)
    
    # Convert from radians to degrees
    shoulder_deg = math.degrees(theta1)
    elbow_deg = math.degrees(theta2)
    
    # Apply servo calibration
    shoulder_servo = SHOULDER_A * shoulder_deg + SHOULDER_B
    elbow_servo = ELBOW_A * elbow_deg + ELBOW_B
    
    return shoulder_servo, elbow_servo

def forward_kinematics(shoulder_servo: float, elbow_servo: float) -> tuple[float, float]:
    """
    Calculates the normalized position on the page given 2 servo angles.

    Args:
        shoulder_servo (float): Angle of the shoulder servo
        elbow_servo (float): Angle of the elbow servo

    Returns:
        tuple[float, float]: Normalized x and y positions
    """
    
    # Convert servo angles to positions
    shoulder_deg = (shoulder_servo - SHOULDER_B) / SHOULDER_A
    elbow_deg = (elbow_servo - ELBOW_B) / ELBOW_A
    
    theta1 = math.radians(shoulder_deg)
    theta2 = math.radians(elbow_deg)

    # Computer pen position
    x = L1 * math.cos(theta1) + L2 * math.cos(theta1 + theta2)
    y = L1 * math.sin(theta1) + L2 * math.sin(theta1 + theta2)
    
    # Normalize to a 0-1 range
    x_norm = 1 - ((x + SHOULDER_X) / PAPER_WIDTH)
    y_norm = 1 - ((y + SHOULDER_Y) / PAPER_HEIGHT)

    return x_norm, y_norm

