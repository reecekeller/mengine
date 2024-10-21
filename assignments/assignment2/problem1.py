import numpy as np
import os
import math
import mengine as m

"""
16-741 Assignment 2 Problem 1.

Attention: quaternions are represented as [x, y, z, w], same as in pybullet.

"""

np.set_printoptions(precision=4, suppress=True)


def rotation_matrix_to_quaternion(R: np.ndarray) -> np.ndarray:
    """Convert rotation matrix R (3x3) to quaternion q (1x4)."""
    # input: R: rotation matrix
    # output: q: quaternion    
    q = np.zeros(4)
    if R[2, 2] < 0:
        if (R[0, 0] > R[1, 1]):
            q[0] = 1 + R[0, 0] - R[1, 1] - R[2, 2]
            q[1] = R[0, 1] + R[1, 0]
            q[2] = R[2, 0] + R[0, 2]
            q[3] = R[1, 2] - R[2, 1]
            q = 0.5*q/np.sqrt(q[0])
            q[3]=-q[3]
        else:
            q[1] = 1 - R[0, 0] + R[1, 1] - R[2, 2]
            q[0] = R[0, 1] + R[1, 0]
            q[2] = R[1, 2] + R[2, 1]
            q[3] = R[2, 0] - R[0, 2]
            q = 0.5*q/np.sqrt(q[1])
            q[3]=-q[3]
    else:
        if R[0, 0] < -R[1, 1]:
            q[2] = 1 - R[0, 0] - R[1, 1] + R[2, 2]
            q[3] = R[0, 1] - R[1, 0]
            q[0] = R[2, 0] + R[0, 2]
            q[1] = R[1, 2] + R[2, 1]
            q = 0.5*q/np.sqrt(q[2])
            q[3]=-q[3]
        else:
            q[3] = 1 + R[0, 0] + R[1, 1] + R[2, 2]
            q[2] = R[0, 1] - R[1, 0]
            q[1] = R[2, 0] - R[0, 2]
            q[0] = R[1, 2] - R[2, 1]
            q = 0.5*q/np.sqrt(q[3])
            q[3]=-q[3]
    return q
    # ------ Student answer above -------


def rodrigues_formula(n, x, theta):
    # Rodrigues' formula for axis-angle: rotate a point x around an axis n by angle theta
    # input: n, x, theta: axis, point, angle
    # output: x_new: new point after rotation
    # ------ TODO Student answer below -------
    return n * np.dot(n, x) + np.sin(theta) * np.cross(n, x) - np.cos(theta) * np.cross(n, np.cross(n, x))
    # ------ Student answer above -------


def axis_angle_to_quaternion(axis: np.ndarray, angle: float) -> np.ndarray:
    """Convert axis-angle representation to quaternion."""
    # input: axis: axis of rotation
    #        angle: angle of rotation (radians)
    # output: q: quaternion
    # ------ TODO: Student answer below -------
    return np.array([axis[0]*np.sin(angle/2), axis[1]*np.sin(angle/2), axis[2]*np.sin(angle/2), np.cos(angle/2)])
    # ------ Student answer above -------


def hamilton_product(p: np.ndarray, q: np.ndarray) -> np.ndarray:
    # ------ TODO: Student answer below -------
    prod = np.zeros(4)
    b, c, d, a = p[0], p[1], p[2], p[3]
    f, g, h, e= q[0], q[1], q[2], q[3]
    prod[3] = a*e-b*f-c*g-d*h
    prod[0] = a*f+b*e+c*h-d*g
    prod[1] = a*g-b*h+c*e+d*f
    prod[2] = a*h+b*g-c*f+d*e
    return prod
    # ------ Student answer above -------


def unit_tests():
    """Simple unit tests.
    Passing these test cases does NOT ensure your implementation is fully correct.
    """
    # test rotation_matrix_to_quaternion
    q = rotation_matrix_to_quaternion(np.diag([1, -1, -1]))
    try:
        assert np.allclose(q, [1, 0, 0, 0]) or np.allclose(q, [-1, 0, 0, 0])
        print("rotation_matrix_to_quaternion passed test case 1")
    except AssertionError:
        print("rotation_matrix_to_quaternion failed test case 1")

    R = np.array([[-0.545, 0.797, 0.260],
                  [0.733, 0.603, -0.313],
                  [-0.407, 0.021, -0.913]])
    q = rotation_matrix_to_quaternion(R)
    print(q)
    try:
        assert np.allclose(q, [0.437, 0.875, -0.0836, 0.191], atol=1e-3)
        print("rotation_matrix_to_quaternion passed test case 2")
    except AssertionError:
        print("rotation_matrix_to_quaternion failed test case 2")

    # test axis_angle_to_quaternion
    q = axis_angle_to_quaternion(np.array([1, 0, 0]), 0.123)
    try:
        assert np.allclose(q, [0.06146124, 0, 0, 0.99810947])
        print("axis_angle_to_quaternion passed test case")
    except AssertionError:
        print("axis_angle_to_quaternion failed test case")

    # test hamilton_product
    p = np.array([0.437, 0.875, -0.0836, 0.191])
    q = np.array([0.06146124, 0, 0, 0.99810947])
    try:
        assert np.allclose(hamilton_product(p, q),
                           [0.4479,  0.8682, -0.1372,  0.1638], atol=1e-3)
        print("hamilton_product passed test case")
    except AssertionError:
        print("hamilton_product failed test case")


if __name__ == '__main__':
    # Create environment and ground plane
    env = m.Env()
    ground = m.Ground([0, 0, -0.5])
    env.set_gui_camera(look_at_pos=[0, 0, 0])

    # Axis-angle definition
    n = np.array([0, 0, 1])
    x = np.array([0.2, 0, 0])

    # Create axis
    axis = m.Shape(m.Cylinder(radius=0.02, length=0.5), static=True, position=[0, 0, 0], orientation=n,
                   rgba=[0.8, 0.8, 0.8, 1])
    # Create point to rotate around axis
    point = m.Shape(m.Sphere(radius=0.02), static=True,
                    position=x, rgba=[0, 0, 1, 0.5])
    point_q = m.Shape(m.Sphere(radius=0.02), static=True,
                      position=x, rgba=[0, 1, 0, 0.5])

    # First we want to implement some converstions and the Hamilton product for quaternions.
    print("Running unit tests...")
    unit_tests()

    x_new_report = []
    x_new_q_report = []

    for i in range(10000):
        theta = np.radians(i)
        # Rodrigues' formula for axis-angle rotation
        x_new = rodrigues_formula(n, x, theta)

        # Axis-angle to quaternion
        theta = np.radians(i-10)  # Offset theta so we can see the two points
        q = axis_angle_to_quaternion(n, theta)

        # rotate using quaternion and the hamilton product
        # ------ TODO Student answer below -------
        q_star = np.array([-q[0], -q[1], -q[2], q[3]])
        x_q = np.array([x[0], x[1], x[2], 0])
        prod = hamilton_product(x_q, q_star)
        x_new_q = hamilton_product(q, prod)
        # ------ Student answer above -------

        point.set_base_pos_orient(x_new)
        point_q.set_base_pos_orient(x_new_q[:3])

        m.step_simulation(realtime=True)

        if i % 50 == 0 and i < 501:
            x_new_report.append(x_new.tolist())
            x_new_q_report.append(x_new_q[:3].tolist())
        if i == 500:
            print("Point rotated using rodrigues formula: ")
            for row in x_new_report:
                formatted_row = [f"{elem:.4f}" for elem in row]
                print(formatted_row)
            print("Point rotated using hamilton product: ")
            for row in x_new_q_report:
                formatted_row = [f"{elem:.4f}" for elem in row]
                print(formatted_row)
