import os
import numpy as np
import mengine as m
np.set_printoptions(precision=2, suppress=True)

applied_pos = [0.3, 0, 0.05]

# Create environment and ground plane
env = m.Env()

def friction_test(mu=0.5):
    # Reset environment and ground plane
    env.reset()
    ground = m.Ground()

    # Create table and cube
    table = m.URDF(filename=os.path.join(m.directory, 'table', 'table.urdf'), static=True, position=[0, 0, 0], orientation=[0, 0, 0, 1])
    table.set_whole_body_frictions(lateral_friction=mu, spinning_friction=0, rolling_friction=0)
    cube = m.Shape(m.Box(half_extents=[0.1]*3), static=False, mass=1.0, position=[0, 0, 1], orientation=[0, 0, 0, 1], rgba=[0, 1, 0, 0.5])

    # Let the cube drop onto the table
    m.step_simulation(steps=30, realtime=False)

    # Create a sphere (finger) to collide with the cube
    finger = m.Shape(m.Sphere(radius=0.02), static=False, mass=1.0, position=cube.local_to_global_coordinate_frame(applied_pos)[0], rgba=[1, 0, 0, 1])
    finger.set_whole_body_frictions(lateral_friction=100, spinning_friction=100, rolling_friction=100)

    for i in range(100):
        # Apply a force to the finger
        force = -finger.get_link_mass(finger.base)*env.gravity # Gravity compensation force
        force += np.array([-3, 0, 0])
        finger.apply_external_force(link=finger.base, force=force, pos=finger.get_base_pos_orient()[0], local_coordinate_frame=False)

        m.step_simulation(realtime=True)

friction_test(mu=0.5)
friction_test(mu=2.0)
