import math

L1 = 155
L2 = 155

PAPER_WIDTH = 215  # 215 mm
PAPER_HEIGHT = 295   # 295 mm

calibration = {
    (0, 0): (84.3, 60),
    (1, 0): (34.8, 53.4),
    (0, 1): (162.1, 138.3),
    (1, 1): (42, 138),
}

def inverse_kinematics(x, y):
    d = math.sqrt(x**2 + y**2)
    
    cos_theta2 = (d**2 - L1**2 - L2**2)/ (2 * L1 * L2)
    cos_theta2 = max(-1, min(1, cos_theta2))
    
    theta2 = math.acos(cos_theta2)
    theta1 = math.atan2(y, x) - math.atan2(L2 * math.sin(theta2), L1 + L2 * math.cos(theta2))
    
    shoulder = math.degrees(theta1)
    elbow = math.degrees(theta2)
    
    return shoulder, elbow

def map_xy_to_angles(x, y):
    x_mm = x * PAPER_WIDTH
    y_mm = y * PAPER_HEIGHT
    
    shoulder_ik, elbow_ik = inverse_kinematics(x_mm, y_mm)
    
    S00, E00 = calibration[(0,0)]
    S10, E10 = calibration[(1,0)]
    S01, E01 = calibration[(0,1)]
    S11, E11 = calibration[(1,1)]
    
    def bilinear(x, y, v00, v10, v01, v11):
        return ((1-x)*(1-y)*v00 + x*(1-y)*v10 + (1-x)*y*v01 + x*y*v11)
    
    shoulder_c = bilinear(x, y, S00, S10, S01, S11)
    elbow_c = bilinear(x, y, E00, E10, E01, E11)
    
    # print(shoulder_ik, elbow_ik, " | ", shoulder_c, elbow_c, " | ", x, y)
    
    return shoulder_ik, elbow_ik 

def lab8_ik(x, y):
    ax = -50 
    ay = 139.5
    cx = x * PAPER_WIDTH
    cy = y * PAPER_HEIGHT
    
    ac = math.sqrt((ax - cx)**2  + (ay - cy)**2)
    
    a_base_c = math.sqrt((ax - cx)**2 + cy**2)
    
    bac = math.acos((L1**2 + ac**2 - L2**2) / (2 * L1 * ac))
    
    acb = math.asin(L1 * math.sin(bac) / L2)
    
    yac = math.acos((ay**2 + ac**2 - a_base_c**2) / (2 * ay * ac))
    
    alpha = bac + yac
    beta = bac + acb
    
    return math.degrees(alpha - math.radians(75)), math.degrees(math.radians(150) - beta)