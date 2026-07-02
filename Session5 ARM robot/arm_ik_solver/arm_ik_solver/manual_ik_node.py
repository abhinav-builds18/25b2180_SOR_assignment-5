import math
import sys
import time

import rclpy
from rclpy.node import Node
from builtin_interfaces.msg import Duration
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

BASE_TO_PAN = 0.05        
PAN_TO_LIFT = 0.025       
H = BASE_TO_PAN + PAN_TO_LIFT  
L1 = 0.20  
L2 = 0.25   
L3 = 0.175  

JOINT_NAMES = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_joint"]

JOINT_LIMITS = {
    "shoulder_pan_joint": (-3.14, 3.14),
    "shoulder_lift_joint": (-1.5708, 1.5708),
    "elbow_joint": (-2.3562, 2.3562),
    "wrist_joint": (-2.3562, 2.3562),
}



def solve_ik(x, y, z, phi_deg=180.0, elbow_up=True):

    if abs(x) < 1e-9 and abs(y) < 1e-9:
        raise ValueError(
            "target sits exactly on the base's vertical axis (x=0, y=0) - "
            "base rotation is undefined here, nudge x or y slightly"
        )

    
    theta1 = math.atan2(y, x)

    r = math.hypot(x, y)
    z2 = z - H

    phi = math.radians(phi_deg)
    rw = r - L3 * math.sin(phi)
    zw = z2 - L3 * math.cos(phi)

    D = (rw**2 + zw**2 - L1**2 - L2**2) / (2 * L1 * L2)

    if D > 1 or D < -1:
        raise ValueError(
            f"point ({x}, {y}, {z}) is not reachable, D = {D:.3f} (needs to be between -1 and 1). "
            f"max reach of the arm is about {L1+L2:.3f} m from the shoulder"
        )

    elbow_angle = math.acos(D)
    if not elbow_up:
        elbow_angle = -elbow_angle

    shoulder_angle = math.atan2(rw, zw) - math.atan2(
        L2 * math.sin(elbow_angle), L1 + L2 * math.cos(elbow_angle)
    )

    wrist_angle = phi - shoulder_angle - elbow_angle

    return theta1, shoulder_angle, elbow_angle, wrist_angle



def rot_z(theta):
    c, s = math.cos(theta), math.sin(theta)
    return [[c, -s, 0, 0],
            [s,  c, 0, 0],
            [0,  0, 1, 0],
            [0,  0, 0, 1]]


def rot_y(theta):
    c, s = math.cos(theta), math.sin(theta)
    return [[c, 0, s, 0],
            [0, 1, 0, 0],
            [-s, 0, c, 0],
            [0,  0, 0, 1]]


def trans_z(d):
    return [[1, 0, 0, 0],
            [0, 1, 0, 0],
            [0, 0, 1, d],
            [0, 0, 0, 1]]


def mat_mult(A, B):
    result = [[0.0] * 4 for _ in range(4)]
    for i in range(4):
        for j in range(4):
            result[i][j] = sum(A[i][k] * B[k][j] for k in range(4))
    return result


def forward_kinematics(theta1, theta2, theta3, theta4):
    T = trans_z(BASE_TO_PAN)
    T = mat_mult(T, rot_z(theta1))
    T = mat_mult(T, trans_z(PAN_TO_LIFT))
    T = mat_mult(T, rot_y(theta2))
    T = mat_mult(T, trans_z(L1))
    T = mat_mult(T, rot_y(theta3))
    T = mat_mult(T, trans_z(L2))
    T = mat_mult(T, rot_y(theta4))
    T = mat_mult(T, trans_z(L3))
    return T[0][3], T[1][3], T[2][3]


class ManualIKNode(Node):
    def __init__(self):
        super().__init__("manual_ik_node")
        self.pub = self.create_publisher(JointTrajectory, "/arm_controller/joint_trajectory", 10)

    def move_to(self, x, y, z, phi_deg=180.0, elbow_up=True):

        try:
            angles = solve_ik(x, y, z, phi_deg, elbow_up)
        except ValueError as e:
            self.get_logger().error(str(e))
            return False

        for name, ang in zip(JOINT_NAMES, angles):
            lo, hi = JOINT_LIMITS[name]
            if ang < lo or ang > hi:
                self.get_logger().warn(f"{name} = {ang:.3f} rad is outside its limit ({lo}, {hi})")

        fx, fy, fz = forward_kinematics(*angles)
        err = math.dist((x, y, z), (fx, fy, fz))

        print(f"target (relative to base_link center): x={x} y={y} z={z}")
        print("joint angles (rad):", [round(a, 4) for a in angles])
        print("joint angles (deg):", [round(math.degrees(a), 2) for a in angles])
        print(f"FK check - end effector actually lands at: ({fx:.4f}, {fy:.4f}, {fz:.4f})")
        print(f"position error: {err*1000:.3f} mm")
        if err > 1e-6:
            self.get_logger().warn("FK check did not land exactly on target, double check the geometry")

        traj = JointTrajectory()
        traj.joint_names = JOINT_NAMES
        pt = JointTrajectoryPoint()
        pt.positions = list(angles)
        pt.velocities = [0.0, 0.0, 0.0, 0.0]
        pt.time_from_start = Duration(sec=3, nanosec=0)
        traj.points.append(pt)

        self.pub.publish(traj)
        self.get_logger().info("trajectory published")
        return True


def parse_line(line):

    parts = line.split()
    if len(parts) < 3:
        return None
    try:
        x, y, z = float(parts[0]), float(parts[1]), float(parts[2])
    except ValueError:
        return None
    phi = 180.0
    elbow_up = True
    if len(parts) >= 4:
        try:
            phi = float(parts[3])
        except ValueError:
            return None
    if len(parts) >= 5:
        elbow_up = parts[4].lower() != "down"
    return x, y, z, phi, elbow_up


def main(args=None):

    cli_args = [
        a for a in sys.argv[1:]
        if not (a.startswith("--") or a.startswith("__") or ":=" in a)
    ]

    rclpy.init(args=args)
    node = ManualIKNode()

    rclpy.spin_once(node, timeout_sec=1.0)
    time.sleep(0.5)

    if len(cli_args) >= 3:
        parsed = parse_line(" ".join(cli_args))
        if parsed:
            node.move_to(*parsed)
        else:
            node.get_logger().warn("couldn't parse the command line coordinates, skipping")

    print()
    print("node is running - type new coordinates to move the arm again.")
    print("format: x y z [phi_deg] [up/down]   e.g.  0.3 0.1 0.15  or  0.3 0.1 0.15 180 up")
    print("type 'q' to quit")
    print()

    try:
        while rclpy.ok():
            line = input("target > ").strip()
            if line.lower() in ("q", "quit", "exit"):
                break
            if not line:
                continue
            parsed = parse_line(line)
            if parsed is None:
                print("couldn't parse that - need at least: x y z")
                continue
            node.move_to(*parsed)
    except (KeyboardInterrupt, EOFError):
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()