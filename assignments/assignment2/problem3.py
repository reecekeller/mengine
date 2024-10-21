import time
import os
import mengine as m
import numpy as np


def q_to_R(q: np.ndarray):
    q0 = q[3]; q1 = q[0]; q2 =q[1]; q3 = q[2]
    R = np.array([[q0**2 + q1**2 - q2**2 - q3**2, 2*(q1*q2-q0*q3), 2*(q1*q3+q0*q2)], 
                  [2*(q1*q2+q0*q3), q0**2-q1**2+q2**2-q3**2, 2*(q2*q3-q0*q1)],
                  [2*(q1*q3-q0*q2), 2*(q2*q3-q0*q1), q0**2-q1**2-q2**2+q3**2]])
    return R

def invertQ(q):
    """
    Invert a quaternion, this function is optional and you could use it in line_intersection if you want
    """
    # ------ TODO Student answer below -------
<<<<<<< HEAD
    # NOTE: Optional, you do not need to use this function
    return np.array([0, 0, 0, 1])
=======
    q_star = np.array([-q[0], -q[1], -q[2], -q[3]]) 
    return q_star / np.linalg.norm(q_star)**2
>>>>>>> 81de652ecef227d66a0351ef09dffbbd6d6bce0d
    # ------ Student answer above -------


def line_intersection(p1, p2, q1, q2):
    """
    Find the intersection of two 3D line segments p1-p2 and q1-q2.
    If there is an intersection, returns the point. Otherwise, returns None.
    """
    # ------ TODO Student answer below -------
    a = p2-p1
    b = q2-q1
    A = np.array([a, -b]) 
    x = q1 - p1  
    (t, s), _, _, _ = np.linalg.lstsq(A.T, x)
    if 0 <= t <= 1 and 0 <= s <= 1:
        p_int = p1+t*a
        return p_int
    else:
        return None
    # ------ Student answer above -------


# Create environment and ground plane
env = m.Env()
# ground = m.Ground()
env.set_gui_camera(look_at_pos=[0, 0.4, 0.25])

fbl = m.URDF(filename=os.path.join(m.directory, 'fourbarlinkage.urdf'),
             static=True, position=[0, 0, 0.3], orientation=[0, 0, 0, 1])
fbl.controllable_joints = [0, 1, 2]
# Create a constraint for the 4th joint to create a closed loop
fbl.create_constraint(parent_link=1, child=fbl, child_link=4, joint_type=m.p.JOINT_POINT2POINT, joint_axis=[
                      0, 0, 0], parent_pos=[0, 0, 0], child_pos=[0, 0, 0])
m.step_simulation(steps=20, realtime=False)

coupler_links = [1, 3, 5]

links = [1, 3]
global_points = []
previous_global_points = []
lines = [None, None]
lines_start_end = [[[0, 0, 0], [0, 0, 0]], [[0, 0, 0], [0, 0, 0]]]

for link in links:
    global_points.append(fbl.get_link_pos_orient(link)[0])
    previous_global_points.append(global_points[-1])
    point = m.Shape(m.Sphere(radius=0.02), static=True,
                    position=global_points[-1], rgba=[0, 0, 1, 1])

intersect_points_local = []
intersect_points_local_bodies = []

for i in range(10000):
    fbl.control([np.radians(i)]*3)

    if i > 3:
        for j, (link, global_position, previous_global_position) in enumerate(zip(links, global_points, previous_global_points)):
            p_new = fbl.get_link_pos_orient(link)[0]
            ic_vector_of_motion = p_new - previous_global_position
            ic_bisector = np.cross(ic_vector_of_motion, [0, 1, 0])
            ic_bisector = ic_bisector / np.linalg.norm(ic_bisector)
            previous_global_points[j] = p_new

            lines[j] = m.Line(p_new-ic_bisector, p_new+ic_bisector,
                              radius=0.005, rgba=[0, 0, 1, 0.5], replace_line=lines[j])
            lines_start_end[j] = (p_new-ic_bisector, p_new+ic_bisector)

        if len(intersect_points_local) < 400:
            # stop drawing if we have drawn 500 points
            intersect_point = line_intersection(
                lines_start_end[0][0], lines_start_end[0][1], lines_start_end[1][0], lines_start_end[1][1])

            if intersect_point is not None:
                m.Shape(m.Sphere(radius=0.005), static=True,
                        position=intersect_point, collision=False, rgba=[1, 0, 0, 1])
                # draw moving centrode
                # get intersection point in local frame w.r.t. link 4
                p, _ = fbl.global_to_local_coordinate_frame(intersect_point, link=3)
                local_intersect_point = np.array(p)

                intersect_points_local.append(local_intersect_point)
                # get global coordinates of intersection point
                intersect_point_local_body = m.Shape(m.Sphere(radius=0.005), static=True,
                                                     position=intersect_point, collision=False, rgba=[0, 1, 0, 1])
                intersect_points_local_bodies.append(
                    intersect_point_local_body)

        # redraw intersection points of moving centrode
        for body, point_local in zip(intersect_points_local_bodies, intersect_points_local):
            p, _ = fbl.local_to_global_coordinate_frame(point_local, link=3)
            body.set_base_pos_orient(p)
<<<<<<< HEAD
=======

            ## old code before assignment update
        #         q = fbl.get_link_pos_orient(4)[1]
        #         xyz = fbl.get_link_pos_orient(4)[0]
        #         #q_inv = invertQ(q)
        #         R_inv = q_to_R(q)
        #         local_intersect_point = R_inv @ (intersect_point-xyz)
        #         # ------ Student answer above -------

        #         intersect_points_local.append(local_intersect_point)
        #         # get global coordinates of intersection point
        #         intersect_point_local_body = m.Shape(m.Sphere(radius=0.005), static=True,
        #                                              position=intersect_point, collision=False, rgba=[0, 1, 0, 1])                
        #         intersect_points_local_bodies.append(
        #             intersect_point_local_body)

        # # redraw intersection points of moving centrode
        # # ------ TODO Student answer below -------
        # # Hint: You can use Body.set_base_pos_orient(xyz) to update a body's position
        # for body, point_local in zip(intersect_points_local_bodies, intersect_points_local):
        #     q = fbl.get_link_pos_orient(4)[1]
        #     xyz = fbl.get_link_pos_orient(4)[0]
        #     R = q_to_R(invertQ(q)) 
        #     global_point = R @ point_local  + xyz  
        #     body.set_base_pos_orient(global_point)
                            # ------ Student answer above -------
>>>>>>> 81de652ecef227d66a0351ef09dffbbd6d6bce0d

    m.step_simulation(realtime=True)

    if i == 500 or i == 600 or i == 700:
        print('--------------------------------------------------------------')
        print(f'Frame {i}: Please save screenshot and include in writeup')
        input("Press Enter to continue...")
