import os
import numpy as np
import mengine as m
np.set_printoptions(precision=2, suppress=True)

mu = 0.5 # Coefficient of friction

# Create environment and ground plane
env = m.Env()
ground = m.Ground()

# Create table and cube
table = m.URDF(filename=os.path.join(m.directory, 'table', 'table.urdf'), static=True, position=[0, 0, 0], orientation=[0, 0, 0, 1])
cube = m.Shape(m.Box(half_extents=[0.1]*3), static=False, mass=1.0, position=[0, 0, 1], orientation=[0, 0, 0, 1], rgba=[0, 1, 0, 0.5])
cube.set_whole_body_frictions(lateral_friction=mu, spinning_friction=0, rolling_friction=0)

# Let the cube drop onto the table
m.step_simulation(steps=50)

# Create a collision-free sphere to visualize where we are applying force
sphere = m.Shape(m.Sphere(radius=0.01), static=True, mass=0, position=cube.local_to_global_coordinate_frame([0.1, 0, 0]), collision=False, rgba=[1, 0, 0, 1])
line = None

for i in range(300):
    # Apply a force to the cube
    applied_force = [-i / 25.0, 0, 0]
    applied_pos = [0.1, 0.1, 0]
    cube.apply_external_force(link=cube.base, force=applied_force, pos=applied_pos, local_coordinate_frame=True)

    m.step_simulation()

    # Show position of applied force and a line indicating the applied velocity
    pos_global = cube.local_to_global_coordinate_frame(applied_pos)[0]
    force_global = cube.local_to_global_coordinate_frame(applied_force)[0]
    sphere.set_base_pos_orient(pos_global)
    m.clear_visual_item(line)
    line = m.Line(pos_global, pos_global + (force_global-pos_global)/10.0, rgb=[1, 0, 0])

    normal, friction_1, friction_2 = cube.get_resultant_contact_forces(bodyB=table)

    cp = cube.get_contact_points(bodyB=table)
    r_IC = np.array([0, 0, -0.1])
    k = np.array([0, 0, 1])
    theta_dot = np.dot(k, cube.get_base_angular_velocity())
    integral = np.zeros(3)
    if cp is not None:
        for c in cp:
            r = cube.global_to_local_coordinate_frame(c['posA'])[0]
            force = c['normal_force']
            integral += (r - r_IC) / np.linalg.norm(r - r_IC) * force
    print(-mu * np.sign(theta_dot) * np.cross(k, integral), friction_1, friction_2)

    # velocity = cube.get_base_linear_velocity()
    # if np.linalg.norm(velocity) > 0.005:
    #     # Record the resultant normal and frictional forces between the cube and table
    #     normal, friction_1, friction_2 = cube.get_resultant_contact_forces(bodyB=table)
    #     print('Cube has begun to move with linear velocity:', velocity)
    #     print('Estimate mu: %.3f' % (np.linalg.norm(applied_force) / np.linalg.norm(normal)), '| True mu:', mu)
    #     break

