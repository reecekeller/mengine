import matplotlib.pyplot as plt
import os
import numpy as np
import mengine as m
import pybullet as p
np.set_printoptions(precision=3, suppress=True)

# NOTE: This assignment asks you to apply Reuleaux's method.


env = m.Env(gravity=[0, 0, 0])
camera = m.Camera()
camera.set_camera_rpy(look_at_pos=[0, 0, 0], distance=0.7, rpy=[0, -89.99, 0])
obj_pos = np.array([0, 0, 0.5])


def add_contact_point(position):
    # input: position: position of the contact point relativ to object position
    return m.Shape(m.Sphere(radius=0.02), static=True,
                   position=obj_pos + position, rgba=[1, 0, 0, 1])

def wait_for_key(message):
    # NOTE: Press enter to continue
    print(message)
    keys = m.get_keys()
    while True:
        keys = m.get_keys()
        if 'return' in keys:
            break
        m.step_simulation(realtime=True)


def reset():
    # Create environment and ground plane
    env.reset()
    ground = m.Ground([0, 0, -0.06])
    env.set_gui_camera(look_at_pos=obj_pos, distance=0.5, pitch=-15, yaw=15)


    m.box = m.Shape(m.Box(half_extents=[0.1, 0.08, 0.05]), static=False, position=obj_pos, rgba=[0, 1, 0, 0.5])

    radius_contact_spheres = 0.02

    # CHANGE NOTE: Previously, this question required a student answer:
    #              These points are now provided! You do not need to implement anything extra for this function!
    s1 = add_contact_point([-0.1 - radius_contact_spheres, 0, 0])
    s2 = add_contact_point([0.06, 0.08 + radius_contact_spheres, 0])
    s3 = add_contact_point([0.1 + radius_contact_spheres, 0.05, 0.0])
    s4 = add_contact_point([0.0, -0.08 - radius_contact_spheres, 0.0])
    s5 = add_contact_point([0.0, 0.0, -0.05 - radius_contact_spheres])
    s6 = add_contact_point([-0.06, 0.05, 0.05 + radius_contact_spheres])
    s7 = add_contact_point([0.06, -0.05, 0.05 + radius_contact_spheres])


def get_contact_screw(contact_location, contact_normal):
    """Returns the contact screw in screw coordinates given a contact location and contact_normal
    """
    # ------ TODO Student answer below -------

    # Compute the contact screw using the contact location and contact normal
    contact_screw = np.zeros(6)
    am = np.cross(contact_location, contact_normal)
    contact_screw = np.concatenate((contact_normal, am))
    
    return contact_screw
    # ------ Student answer above -------



reset()
wait_for_key('Press enter to drop the block...')
p.setGravity(0,0,-10)
count = 0

# Drop the box and visualize the contact normals
while True:
    count += 1
    m.clear_all_visual_items()
    contact_points = m.box.get_contact_points()
    if contact_points is not None:
        for i, contact_point in enumerate(contact_points):
            # Draw contact normals
            m.Line(contact_point['posB'], np.array(
                contact_point['posB']) + np.array(contact_point['contact_normal'])*0.3, radius=0.002, rgb=[0, 0, 1])

            # compute contact screw
            contact_screw = get_contact_screw(contact_point['posB'], contact_point['contact_normal'])
            if count == 10:
                print(f'Contact Screw {i}: {contact_screw}')

    m.step_simulation(realtime=True)
