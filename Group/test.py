import inverse_kinematics as ik


def voltage_to_angle(v, vmin, vmax, amin, amax):
    if v < vmin:
        v = vmin
    if v > vmax:
        v = vmax

    return amin + ((v - vmin) / (vmax - vmin)) * (amax - amin)

def angle_to_voltage(angle, vmin, vmax, amin, amax):
    angle = max(amin, min(amax, angle))
    voltage = vmin + ( (angle - amin) / (amax - amin) ) * (vmax - vmin)
    voltage = max(vmin, min(vmax, voltage))
    
    return voltage


MIN_VOLTAGE_ELBOW = 0.2
MAX_VOLTAGE_ELBOW = 3.156
MIN_ANGLE_ELBOW = 9.054546
MAX_ANGLE_ELBOW = 173.3455
MIN_VOLTAGE_SHOULDER = 0.162
MAX_VOLTAGE_SHOULDER = 3.176
MIN_ANGLE_SHOULDER = 10.69091
MAX_ANGLE_SHOULDER = 172.1455

for (i, j) in [(0,0), (0, 1), (1, 1), (1, 0)]:
    s, e = ik.inverse_kinematics(i, j)
    fs, fe = ik.forward_kinematics(s, e)
    vs = angle_to_voltage(s, MIN_VOLTAGE_SHOULDER, MAX_VOLTAGE_SHOULDER, MIN_ANGLE_SHOULDER, MAX_ANGLE_SHOULDER)
    ve = angle_to_voltage(e, MIN_VOLTAGE_ELBOW, MAX_VOLTAGE_ELBOW, MIN_ANGLE_ELBOW, MAX_ANGLE_ELBOW)
    
    avs = voltage_to_angle(vs, MIN_VOLTAGE_SHOULDER, MAX_VOLTAGE_SHOULDER, MIN_ANGLE_SHOULDER, MAX_ANGLE_SHOULDER)
    ave = voltage_to_angle(ve, MIN_VOLTAGE_ELBOW, MAX_VOLTAGE_ELBOW, MIN_ANGLE_ELBOW, MAX_ANGLE_ELBOW)

    print(f"Position: {i}, {j}")
    print(f"Inverse: {s:.2f}, {e:.2f}")
    print(f"Forward: {fs:.2f}, {fe:.2f}")
    print(f"Voltage: {vs:.2f}, {ve:.2f}")
    print(f"Angle: {avs:.2f}, {ave:.2f}\n")
