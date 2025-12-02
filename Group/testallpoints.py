import inverse_kinematics as ik

STEPS = 101

for i_step in range(STEPS):
    x = i_step / (STEPS - 1)

    for j_step in range(STEPS):
        y = j_step / (STEPS - 1)

        try:
            s, e = ik.inverse_kinematics(x, y)
        except Exception as err:
            print(f"[{x:.2f}, {y:.2f}]  IK failed: {err}")
            continue

        try:
            fx, fy = ik.forward_kinematics(s, e)
        except Exception as err:
            print(f"[{x:.2f}, {y:.2f}]  FK failed: {err}")
            continue

        # error between original point and forward-kinematics result
        error = ((fx - x)**2 + (fy - y)**2) ** 0.5

        if error >= 0.0001:
            print(f"XY=({x:.2f}, {y:.2f})  IK=({s:.2f}, {e:.2f}) FK=({fx:.2f}, {fy:.2f}")
