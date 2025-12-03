import math

# Robot geometry constants
R1 = 60.0
R2 = 40.0
L = 100.0
base_angles = [0, 120, 240]


# Forward kinematics (approximation)
def forwardKinematics(theta1, theta2, theta3):
    t1 = math.radians(theta1)
    t2 = math.radians(theta2)
    t3 = math.radians(theta3)

    nx = (math.sin(t1) + math.sin(t2) + math.sin(t3)) / 3
    ny = (math.cos(t1) + math.cos(t2) + math.cos(t3)) / 3
    nz = math.sqrt(max(0.0, 1 - nx * nx - ny * ny))
    return nx, ny, nz


# Inverse kinematics
def inverseKinematics(nx, ny, nz):
    angles = []
    for deg in base_angles:
        theta = math.radians(deg)
        bx = math.cos(theta)
        by = math.sin(theta)

        c = (R1 - R2 * (bx * nx + by * ny)) / (L * nz)
        c = max(min(c, 1.0), -1.0)
        angle = math.degrees(math.acos(c))
        angles.append(angle)

    return angles[0], angles[1], angles[2]


# Helper for MPU6050 tilt
def tilt_to_servos(tilt_x_deg, tilt_y_deg):
    tx = math.radians(tilt_x_deg)
    ty = math.radians(tilt_y_deg)

    nx = math.sin(tx)
    ny = math.sin(ty)
    nz_sq = max(0.0, 1 - nx * nx - ny * ny)
    nz = math.sqrt(nz_sq)
    if nz < 1e-3:
        # Prevent numerical blow-ups when the requested tilt exceeds the kinematic limit
        nz = 1e-3

    return inverseKinematics(nx, ny, nz)
