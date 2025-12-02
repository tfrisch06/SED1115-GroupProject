def compute_calibration_constants(angles, servos):
    """
    Computes A and B for linear model:
        servo = A * angle + B

    Works on MicroPython (no numpy).
    Uses least-squares linear regression.
    """

    n = len(angles)
    if n < 2:
        raise ValueError("Need at least 2 calibration points")

    sum_x = sum(angles)
    sum_y = sum(servos)
    sum_xx = sum(a * a for a in angles)
    sum_xy = sum(a * s for a, s in zip(angles, servos))

    # Compute slope A
    denominator = (n * sum_xx - sum_x * sum_x)
    if denominator == 0:
        raise ValueError("Invalid calibration points (vertical line)")

    A = (n * sum_xy - sum_x * sum_y) / denominator

    # Compute intercept B
    B = (sum_y - A * sum_x) / n

    return A, B


# ----------------------------------------------------------------------
# EXAMPLE USAGE (replace with your own measured angles + servo values)
# ----------------------------------------------------------------------

shoulder_angles = [0,  45,  90]       # degrees
shoulder_servos = [146, 191, 236]     # your readings

elbow_angles = [0,  45,  90]
elbow_servos = [171.1, 121.6, 72.1]

SHOULDER_A, SHOULDER_B = compute_calibration_constants(shoulder_angles, shoulder_servos)
ELBOW_A, ELBOW_B = compute_calibration_constants(elbow_angles, elbow_servos)

print("Computed Calibration Constants:")
print("SHOULDER_A =", SHOULDER_A)
print("SHOULDER_B =", SHOULDER_B)
print("ELBOW_A    =", ELBOW_A)
print("ELBOW_B    =", ELBOW_B)
