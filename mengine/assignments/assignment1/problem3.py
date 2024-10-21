from scipy.interpolate import CubicSpline
import os
import numpy as np
import mengine as m
np.set_printoptions(precision=3, suppress=True)

# NOTE: This assignment asks you to Implement FK, plot the robot workspace, and check for collisions
# Create environment and ground plane
env = m.Env()


def reset_sim():
    env.reset()
    ground = m.Ground([0, 0, -0.5])
    env.set_gui_camera(look_at_pos=[0, 0, 0.5], distance=1.5)
    # Create example robot
    robot = m.URDF(filename=os.path.join(
        m.directory, 'assignments', 'example_arm.urdf'), static=True, position=[0, 0, 0])
    robot.controllable_joints = [0, 1, 2]
    robot.end_effector = 3
    robot.update_joint_limits()
    return robot


def sample_configuration():
    # Sample a random configuration for the robot.
    # NOTE: Be conscious of joint angle limits
    # output: q: joint angles of the robot
    # ------ TODO Student answer below -------
    #print('TODO sample_configuration')
    q1 = np.random.uniform(-np.pi, np.pi/2)
    q2 = np.random.uniform(-np.pi/2, np.pi/2)
    q3 = np.random.uniform(-np.pi/2, np.pi/2)

    return np.array([q1, q2, q3])
    # ------ Student answer above -------


def calculate_FK(q, joint=3):
    # Calculate the forward kinematics of the robot
    # NOTE: We encourage doing this with transformation matrices, as shown in class
    # input: q: joint angles of the robot
    #        joint: index of the joint to calculate the FK for
    # output: ee_position: position of the end effector
    #         ee_orientation: orientation of the end effector
    # ------ TODO Student answer below -------
    #print('TODO calculate_FK')
    L1 = 0.5
    L2 = 0.4
    L3 = 0.3

    T_01 = np.eye(4)
    T_01[0, 0] = np.cos(q[0]); T_01[0, 1] = -np.sin(q[0])
    T_01[1, 0] = np.sin(q[0]); T_01[1, 1] = np.cos(q[0])

    T_12 = np.eye(4)
    T_12[1, 1] = np.cos(q[1]); T_12[1, 2] = -np.sin(q[1]); T_12[2, 3] = L1
    T_12[2, 1] = np.sin(q[1]); T_12[2, 2] = np.cos(q[1])

    T_23 = np.eye(4)
    T_23[1, 1] = np.cos(q[2]); T_23[1, 2] = -np.sin(q[2]); T_23[2, 3] = L2
    T_23[2, 1] = np.sin(q[2]); T_23[2, 2] = np.cos(q[2])

    T_34 = np.eye(4); T_34[2, 3] = L3
    if joint==3:
        T = T_01 @ T_12 @ T_23 @ T_34
    elif joint==2:
        T = T_01 @ T_12 @ T_23
    elif joint==1:
        T = T_01 @ T_12

    #x = L1*np.cos(q[0]) + L2*np.cos(q[0] + q[1]) + L3*np.cos(q[0]+q[1]+q[2])
    position = T[:3, 3]
    orientation = T[:3, :3]
    orientation = m.get_quaternion(orientation) # NOTE: If you used transformation matrices, call this function to get a quaternion
    # ------ Student answer above -------
    return position, orientation


def compare_FK(ee_positions, ee_positions_pb, ee_orientations, ee_orientations_pb):
    # Compare the FK implementation to the built-in one
    # input: ee_positions: list of positions of the end effector
    #        ee_positions_pb: list of positions of the end effector from pybullet
    #        ee_orientations: list of orientations of the end effector
    #        ee_orientations_pb: list of orientations of the end effector from pybullet
    distance_error_sum = 0
    orientation_error_sum = 0
    for p1, p2 in zip(ee_positions, ee_positions_pb):
        distance_error_sum += np.linalg.norm(p1 - p2)
    for q1, q2 in zip(ee_orientations, ee_orientations_pb):
        error = np.arccos(2*np.square(q1.dot(q2)) - 1)
        orientation_error_sum += 0 if np.isnan(error) else error
    print('Average FK distance error:', distance_error_sum / len(ee_positions))
    print('Average FK orientation error:', orientation_error_sum / len(ee_orientations))


def plot_point(position):
    # input: position: list of [x,y,z] position of the end effector
    m.Shape(m.Sphere(radius=0.01), static=True, position=position, collision=False, rgba=[1, 0, 0, 1])


def wait_for_enter():
    # NOTE: Press enter to continue to next problem
    print('Press enter in the simulator to continue to the next problem')
    keys = m.get_keys()
    while True:
        keys = m.get_keys()
        if 'return' in keys:
            break
        m.step_simulation(realtime=True)
    m.step_simulation(steps=50, realtime=True)


def check_collision(q, box_position, box_half_extents):
    # Check if the robot is in collision region
    # input: q: joint angles of the robot
    #        box_position: position of the collision region
    #        box_half_extents: half extents of the collision region
    # output: in_collision: True if the robot is in collision region, False otherwise
    # ------ TODO Student answer below -------
    #print('TODO check_collision')
    e1, _ = calculate_FK(q, joint=1)
    e2, _ = calculate_FK(q, joint=2)
    e3, _ = calculate_FK(q, joint=3)

    def condition(effector, box_position, box_half_extents):
        condition_x = box_position[0]-box_half_extents[0] < effector[0] < box_position[0]+box_half_extents[0]
        condition_y = box_position[1]-box_half_extents[1] < effector[1] < box_position[1]+box_half_extents[1]
        condition_z = box_position[2]-box_half_extents[2] < effector[2] < box_position[2]+box_half_extents[2]
        if condition_x and condition_y and condition_z:
            return True
    
    if condition(e1, box_position, box_half_extents) or condition(e2, box_position, box_half_extents) or condition(e3, box_position, box_half_extents):
        return True
    else:
        return False
    # ------ Student answer above -------


# ##########################################
# Problem 3.1:
# Implement FK for a 3-link manipulator and compare your implementation to the built-in function in pybullet
# ##########################################
robot = reset_sim()
ee_positions = []
ee_orientations = []
ee_positions_pb = []
ee_orientations_pb = []

for i in range(100):
    # sample a random configuration q
    q = sample_configuration()
    # move robot into configuration q
    robot.control(q, set_instantly=True)
    m.step_simulation(realtime=True)
    # calculate ee_position, ee_orientation using calculate_FK
    ee_position, ee_orientation = calculate_FK(q, joint=3)
    ee_positions.append(ee_position)
    ee_orientations.append(ee_orientation)
    # calculate ee position, orientation using pybullet's FK
    ee_position_pb, ee_orientation_pb = robot.get_link_pos_orient(robot.end_effector)
    ee_positions_pb.append(ee_position_pb)
    ee_orientations_pb.append(ee_orientation_pb)
# compare your implementation and pybullet's FK
compare_FK(ee_positions, ee_positions_pb, ee_orientations, ee_orientations_pb)

# NOTE: Press enter to continue to problem 3.2
wait_for_enter()


# ##########################################
# Problem 3.2:
# Plot the workspace of the robot using a sampling-based approach
# ##########################################

# ------ TODO Student answer below -------
for i in range(1001):
    # sample a random configuration q
    # TODO
    q = sample_configuration()
    # move robot into configuration q
    robot.control(q, set_instantly=True)
    m.step_simulation(realtime=True)

    # calculate ee_position, ee_orientation using calculate_FK
    # TODO
    ee_position, _ = calculate_FK(q, joint=3)
    # plot workspace as points of the end effector
    plot_point(ee_position)
# ------ Student answer above -------

# NOTE: Press enter to continue to problem 3.3
wait_for_enter()


# ##########################################
# Problem 3.3:
# Expand FK to be FK to all joints and then check for collisions with a box region
# ##########################################

reset_sim()
# create a collision region
box_position = np.array([-0.3, 0, 1.0])
box_half_extents = np.array([0.15, 0.35, 0.25])

# Create a visual box to check collisions with
box = m.Shape(m.Box(box_half_extents), static=True, position=box_position, collision=False, rgba=[0, 1, 0, 0.5])

# Define a joint space trajectory for the robot to follow
q0 = np.array([-np.pi/4, -np.pi/2, -3*np.pi/4])
q1 = np.array([np.pi/2, np.pi/4, np.pi/2])
q2 = np.array([np.pi / 2, np.pi / 4, np.pi / 2])
t = np.array([1, 2, 3])
# Create a cubic spline interpolation function
q0_spline = CubicSpline(t, q0)
q1_spline = CubicSpline(t, q1)
q2_spline = CubicSpline(t, q2)
t_interp = np.linspace(t[0], t[-1], num=200)
q0_traj = q0_spline(t_interp)
q1_traj = q1_spline(t_interp)
q2_traj = q2_spline(t_interp)
traj = np.array([q0_traj, q1_traj, q2_traj])

t_collision = []
for i in range(200):
    # move robot to configuration
    target_joint_angles = traj[:, i]
    robot.control(target_joint_angles, set_instantly=True)
    m.step_simulation(realtime=True)

    # check if robot is in collision region
    # TODO implement check_collision
    in_collision = check_collision(target_joint_angles, box_position, box_half_extents)
    if in_collision:
        box.change_visual(rgba=[1, 0, 0, 0.5])
        print('Robot is in collision region!')
        t_collision.append(i)
    else:
        box.change_visual(rgba=[0, 1, 0, 0.5])
print(t_collision)
m.step_simulation(steps=50, realtime=True)
