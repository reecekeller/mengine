import os
import numpy as np
import mengine as m
np.set_printoptions(precision=3, suppress=True)

# NOTE: This problem asks you to convert between the different rotation representations.

# Create environment and ground plane
env = m.Env()
ground = m.Ground([0, 0, -0.5])
env.set_gui_camera(look_at_pos=[0, 0, 0])

# position definition
x = np.array([0.2, 0, 0])

# Create points to rotate
# point rotated using euler angles
point_e = m.Shape(m.Sphere(radius=0.03), static=True,
                  position=x, rgba=[0, 1, 0, 0.2])
# point rotated using axis-angle
point_aa = m.Shape(m.Sphere(radius=0.025), static=True,
                   position=x, rgba=[1, 0, 0, 0.2])
# point rotated using rotation matrix
point_r = m.Shape(m.Sphere(radius=0.02), static=True,
                  position=x, rgba=[0, 0, 1, 0.2])


def rodrigues_formula(n, x, theta):
    # Rodrigues' formula for axis-angle: rotate a point x around an axis n by angle theta
    # input: n, x, theta: axis, point, angle
    # output: x_new: new point after rotation
    #n = n/np.linalg.norm(n)
    return n * np.dot(n, x) + np.sin(theta) * np.cross(n, x) - np.cos(theta) * np.cross(n, np.cross(n, x))
    # ------ Student answer above -------


def rotate_euler(alpha, beta, gamma, x):
    # Rotate a point x using euler angles (alpha, beta, gamma)
    # input: alpha, beta, gamma: euler angles
    # output: x_new: new point after rotation

    # ------ TODO Student answer below -------
    R_23 = np.eye(3)
    R_23[0, 0] = np.cos(gamma); R_23[0, 1] = -np.sin(gamma)
    R_23[1, 0] = np.sin(gamma); R_23[1, 1] = np.cos(gamma)
    
    R_12 = np.eye(3)
    R_12[0, 0] = np.cos(beta); R_12[0, 2] = np.sin(beta)
    R_12[2, 0] = -np.sin(beta); R_12[2, 2] = np.cos(beta)

    R_01 = np.eye(3)
    R_01[0, 0] = np.cos(alpha); R_01[0, 1] = -np.sin(alpha)
    R_01[1, 0] = np.sin(alpha); R_01[1, 1] = np.cos(alpha)
    
    R_03 = R_01 @ R_12 @ R_23
    return np.dot(R_03, x)
    # ------ Student answer above -------


def euler_to_rotation_matrix(alpha, beta, gamma):
    # Convert euler angles (alpha, beta, gamma) to rotation matrix
    # input: alpha, beta, gamma: euler angles
    # output: R: rotation matrix

    # ------ TODO Student answer below -------
    R_23 = np.eye(3)
    R_23[0, 0] = np.cos(gamma); R_23[0, 1] = -np.sin(gamma)
    R_23[1, 0] = np.sin(gamma); R_23[1, 1] = np.cos(gamma)
    
    R_12 = np.eye(3)
    R_12[0, 0] = np.cos(beta); R_12[0, 2] = np.sin(beta)
    R_12[2, 0] = -np.sin(beta); R_12[2, 2] = np.cos(beta)

    R_01 = np.eye(3)
    R_01[0, 0] = np.cos(alpha); R_01[0, 1] = -np.sin(alpha)
    R_01[1, 0] = np.sin(alpha); R_01[1, 1] = np.cos(alpha)
    
    R_03 = R_01 @ R_12 @ R_23
    return R_03
    # ------ Student answer above -------


def euler_to_axis_angle(alpha, beta, gamma):
    # Convert euler angles (alpha, beta, gamma) to axis-angle representation (n, theta)
    # input: alpha, beta, gamma: euler angles
    # output: n, theta
    # ------ TODO Student answer below -------
    R = euler_to_rotation_matrix(alpha, beta, gamma)
    theta = np.arccos((np.trace(R)-1)/2)
    n_x = (R[2, 1] - R[1, 2])/(2*np.sin(theta))
    n_y = (R[0, 2] - R[2, 0])/(2*np.sin(theta))
    n_z = (R[1, 0] - R[0, 1])/(2*np.sin(theta))
    n = np.array([n_x, n_y, n_z])
    #n = n/np.linalg.norm(n)
    return n, theta
    # else:
    #     print("R is the identity matrix")
    #     return np.zeros(3), 0
    # ------ Student answer above -------


x_new_e = np.array([0.2, 0, 0])
x_new_r = np.array([0.2, 0, 0])
x_new_aa = np.array([0.2, 0, 0])

for alpha, beta, gamma in zip([20, -25, 0], [45, 5, 135], [10, 90, -72]):
    alpha = np.radians(alpha)
    beta = np.radians(beta)
    gamma = np.radians(gamma)

    (n, theta) = euler_to_axis_angle(alpha, beta, gamma)
    R = euler_to_rotation_matrix(alpha, beta, gamma)

    # positions of rotated points for each representation
    x_new_e = rotate_euler(alpha, beta, gamma, x)
    x_new_r = R.dot(x)
    x_new_aa = rodrigues_formula(n, x, theta)

    print('-'*20)
    print('Euler angles:', np.degrees(alpha), np.degrees(beta), np.degrees(gamma))
    print('Axis angle:', n, np.degrees(theta))
    print('Rotation matrix:', R)
    print('x_new_e:', x_new_e)
    print('x_new_r:', x_new_r)
    print('x_new_aa:', x_new_aa)
    print('-'*20)

    point_e.set_base_pos_orient(x_new_e)
    point_r.set_base_pos_orient(x_new_r)
    point_aa.set_base_pos_orient(x_new_aa)

    # NOTE: Press enter to continue to next angles
    print('Press enter in the simulator to continue to the next angle set')
    keys = m.get_keys()
    while True:
        keys = m.get_keys()
        if 'return' in keys:
            break
        m.step_simulation(realtime=True)
    m.step_simulation(steps=50, realtime=True)
