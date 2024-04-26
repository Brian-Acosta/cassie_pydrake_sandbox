import numpy as np
from typing import Tuple

from pydrake.multibody.all import (
    Parser,
    MultibodyPlant,
    RevoluteSpring,
    JointActuatorIndex
)

from pydrake.systems.framework import Context

from constraint_utils import (
    PointOnFrame,
    StackedConstraint,
    ContactConstraint,
    DistanceConstraint,
)


def make_cassie_model(urdf: str) -> Tuple[MultibodyPlant, Context]:
    """
    Builds a drake MultibodyPlant model of Cassie to use as a kinematics
    calculator. Also provides a Context, the object used to store the state of the robot.

    WARNING: This Cassie model is not suitable for simulation, as we do not add a distance constraint
     to represent the four-bar linkage, and we initialize the plant as a continuous model, which would simulate
     quite slowly

    :param urdf: file path to the Cassie urdf
    :return: A tuple containing the plant and the plant's state (context)
    """
    plant = MultibodyPlant(0.0)
    parser = Parser(plant)
    model_instance = parser.AddModels(urdf)

    # Add Cassie's leaf springs
    joint_stiffness_map = {
        "knee_joint_left": 1500, "knee_joint_right": 1500,
        "ankle_spring_joint_left": 1250, "ankle_spring_joint_right": 1250,
    }

    for joint_name, stiffness in joint_stiffness_map.items():
        plant.AddForceElement(
            RevoluteSpring(
                plant.GetJointByName(joint_name), 0, stiffness
            )
        )

    # Add reflected inertia
    rotor_inertias = 1e-6 * np.array([61, 61, 61, 61, 365, 365, 365, 365, 4.9, 4.9])
    gear_ratios = np.array([25, 25, 25, 25, 16, 16, 16, 16, 50, 50])
    motor_joint_names = [
        "hip_roll_left_motor", "hip_roll_right_motor", "hip_yaw_left_motor",
        "hip_yaw_right_motor", "hip_pitch_left_motor", "hip_pitch_right_motor",
        "knee_left_motor", "knee_right_motor", "toe_left_motor",
        "toe_right_motor"]

    for i in range(10):
        joint_actuator = plant.get_mutable_joint_actuator(JointActuatorIndex(i))
        joint_actuator.set_default_rotor_inertia(rotor_inertias[i])
        joint_actuator.set_default_gear_ratio(gear_ratios[i])
        assert(motor_joint_names[i] == joint_actuator.name())

    plant.Finalize()
    return plant, plant.CreateDefaultContext()


def fourbar_linkage_constraints(plant: MultibodyPlant) -> StackedConstraint:
    left_rod_thigh_connection = PointOnFrame(
       np.array([0, 0, 0.045]), plant.GetFrameByName("thigh_left")
    )
    left_rod_heel_connection = PointOnFrame(
       np.array([.11877, -.01, 0.0]), plant.GetFrameByName("heel_spring_left")
    )
    right_rod_thigh_connection = PointOnFrame(
       np.array([0, 0, 0.045]), plant.GetFrameByName("thigh_right")
    )
    right_rod_heel_connection = PointOnFrame(
       np.array([.11877, -.01, 0.0]), plant.GetFrameByName("heel_spring_right")
    )

    return StackedConstraint([
       DistanceConstraint(plant, left_rod_thigh_connection, left_rod_heel_connection,  0.5012),
       DistanceConstraint(plant, right_rod_thigh_connection, right_rod_heel_connection, 0.5012)
    ])


def contact_constraints(plant: MultibodyPlant, toe_name: str):

    assert toe_name == "left" or toe_name == "right"

    front = PointOnFrame(
        np.array([-0.0457, 0.112, 0]), plant.GetFrameByName(f"toe_{toe_name}")
    )
    rear = PointOnFrame(
        np.array([0.088, 0, 0]), plant.GetFrameByName(f"toe_{toe_name}")
    )
    return StackedConstraint([
        ContactConstraint(plant, front, frame_to_express_in=plant.world_frame()),
        ContactConstraint(plant, rear, frame_to_express_in=plant.world_frame()),
    ])