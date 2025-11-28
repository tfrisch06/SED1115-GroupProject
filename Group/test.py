import inverse_kinematics as ik

for (i, j) in [(0,0), (0, 1), (1, 1), (1, 0)]:
    s, e = ik.inverse_kinematics(i, j)
    fs, fe = ik.forward_kinematics(s, e)

    print(f"Position: {i}, {j}")
    print(f"Inverse: {s:.2f}, {e:.2f}")
    print(f"Forward: {fs:.2f}, {fe:.2f}\n")
    
print("=================================")
for (i, j) in [(0, 0), (0, 0.5), (0.5, 0.5), (0.5, 0)]:
    s, e = ik.inverse_kinematics(i, j)
    fs, fe = ik.forward_kinematics(s, e)

    print(f"Position: {i}, {j}")
    print(f"Inverse: {s:.2f}, {e:.2f}")
    print(f"Forward: {fs:.2f}, {fe:.2f}\n")
