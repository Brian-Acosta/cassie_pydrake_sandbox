"""
 This file contains a few examples of using Drake and the included Cassie utilities
 to perform useful multibody kinematics and dynamics computations
"""

import numpy as np
import matplotlib.pyplot as plt

from cassie_utils import (
    make_cassie_model,
    fourbar_linkage_constraints,
    contact_constraints
)

from pydrake.multibody.all import MultibodyForces


def load_data(filepath):
    raw_data = np.load(filepath, allow_pickle=True)
    robot_output = raw_data['robot_output'].item()
    position_names = raw_data['position_names']
    velocity_names = raw_data['velocity_names']
    actuator_names = raw_data['actuator_names']

    return robot_output, position_names, velocity_names, actuator_names


def dynamics_example():
    """
    Example which estimates contact and constraint forces based on the rest of the
     dynamics quantities. Note that we don't impose friction cone constraints, since this is just
     a simple example
    :return: None
    """
    datafile = 'data/mar_12_2024_log1.npz'
    robot_output, position_names, velocity_names, actuator_names = load_data(datafile)
    plant, context = make_cassie_model('urdf/cassie_v2.urdf')
    loop = fourbar_linkage_constraints(plant)
    left_contact = contact_constraints(plant, 'left')
    right_contact = contact_constraints(plant, 'right')

    # print out mardown tables listing the joint info
    print('Positions:\n| Index | Name |')
    for i, n in enumerate(position_names):
        print(f'| {i} | {n} |')

    print('Velocities:\n| Index | Name |')
    for i, n in enumerate(velocity_names):
        print(f'| {i} | {n} |')

    print('Inputs:\n| Index | Name |')
    for i, n in enumerate(actuator_names):
        print(f'| {i} | {n} |')

    # At each time step in our data, calculate the terms in the manipulator equation
    #
    # M \dot{v} + C(q, v) = Bu + g(q) + Kq +
    # J_{left}' \lambda_{left} + J_{right}' \lambda_{right} + J_{loop} \lambda_{loop}
    #
    # Where C(q, v) are the coriolis forces, B is the actuation matrix, u is the input,
    # g(q) are the gravitational forces, Kq are the applied spring forces,
    # J_{left} is the stacked Jacobian for the left foot contact points,
    # J_{right} is the stacked Jacobian for the right foot contact points,
    # J_{loop} is the stacked Jacobian for both the left and right leg loop closure constraint
    # \lambda_{left}, \lambda_{right}, and \lambda_{loop} are the unknown constraint forces
    # corresponding to these Jacobians

    n = 10000  # analyze the first ~10 seconds of the data file
    contact_forces = np.zeros((n, 2))
    for i in range(n):
        t = robot_output['t_x'][i]
        q = robot_output['q'][i]
        v = robot_output['v'][i]
        u = robot_output['u'][i]

        # approximate acceleration via finite difference
        vdot = (robot_output['v'][i+1] - robot_output['v'][i]) / (robot_output['t_x'][i+1] - t)

        plant.SetPositions(context, q)
        plant.SetVelocities(context, v)

        M = plant.CalcMassMatrix(context)
        C = plant.CalcBiasTerm(context)
        B = plant.MakeActuationMatrix()

        # Unfortunately, Drake doesn't have a one-liner for the spring forces yet
        force_result = MultibodyForces(plant)
        plant.CalcForceElementsContribution(context, force_result)
        Kq = force_result.generalized_forces()

        # Remember that in Drake's formulation, g is on the right hand side
        g = plant.CalcGravityGeneralizedForces(context)

        # We don't estimate the velocity of the ankle spring, so Jv might have a small
        # nonzero magnitude even when these constraints are respected
        J_left = left_contact.jacobian(context)
        J_right = right_contact.jacobian(context)
        J_loop = loop.jacobian(context)

        J_stacked = np.vstack([J_left, J_right, J_loop])
        lambda_stacked = np.linalg.lstsq(J_stacked.T, M @ vdot + C - B @ u - g - Kq, rcond=None)[0]

        contact_forces[i, 0] = np.linalg.norm(lambda_stacked[:3] + lambda_stacked[3:6])
        contact_forces[i, 1] = np.linalg.norm(lambda_stacked[6:9] + lambda_stacked[9:12])

    plt.plot(robot_output['t_x'][:n], contact_forces)
    plt.title('Estimated Contact Forces Using Least Squares')
    plt.xlabel('Time (s)')
    plt.ylabel('Total Contact Force (N)')
    plt.legend(['left', 'right'])


def plot_measured_foot_speed():
    '''
    An example of using the foot contact constraint Jacobian.
    :return: None
    '''
    datafile = 'data/mar_12_2024_log1.npz'
    robot_output, _, _, _ = load_data(datafile)
    plant, context = make_cassie_model('urdf/cassie_v2.urdf')
    left_contact = contact_constraints(plant, 'left')

    avg_speed_of_left_contact_points = []
    for i in range(len(robot_output['t_x'])):
        t = robot_output['t_x'][i]
        q = robot_output['q'][i]
        v = robot_output['v'][i]
        u = robot_output['u'][i]

        plant.SetPositions(context, q)
        plant.SetVelocities(context, v)

        front_contact_vel = left_contact.jacobian(context)[:3] @ v
        rear_contact_vel = left_contact.jacobian(context)[3:] @ v
        speed = np.linalg.norm(0.5 * (front_contact_vel + rear_contact_vel))
        avg_speed_of_left_contact_points.append(speed)

    plt.plot(robot_output['t_x'][:10000], avg_speed_of_left_contact_points[:10000])
    plt.title('Velocity magnitude of the left foot')
    plt.xlabel('Time(s)')
    plt.ylabel('Speed (m/s)')


if __name__ == '__main__':
    dynamics_example()
    plt.figure()
    plot_measured_foot_speed()
    plt.show()


