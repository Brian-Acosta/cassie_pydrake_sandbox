"""
    Multibody Kinematic constraint implementations translated to python from dairlib:
    https://github.com/DAIRLab/dairlib/tree/main/multibody/kinematic
"""

import numpy as np
from typing import Tuple, List
from dataclasses import dataclass

from pydrake.multibody.all import (
    Frame,
    MultibodyPlant,
    JacobianWrtVariable
)

from pydrake.systems.framework import Context


@dataclass
class PointOnFrame:
    point: np.ndarray
    frame: Frame


class DistanceConstraint:
    """
        Represents the constraint phi(x) = 0, where phi(x) is the distance
        between two points, minus the target distance.
    """

    def __init__(self, plant: MultibodyPlant, point_A: PointOnFrame, point_B: PointOnFrame, distance: float):
        self.plant = plant
        self.point_a = point_A
        self.point_b = point_B
        self.distance = distance
        self.wrt = JacobianWrtVariable.kV

        assert self.distance > 0

    def _calc_rel_pos(self, context: Context):
        world = self.plant.world_frame()
        pt_a_W = self.plant.CalcPointsPositions(
            context, self.point_a.frame, self.point_a.point, world).ravel()
        pt_b_W = self.plant.CalcPointsPositions(
            context, self.point_b.frame, self.point_b.point, world).ravel()

        return pt_a_W - pt_b_W

    def _calc_rel_jac(self, context: Context):
        world = self.plant.world_frame()
        J_a = self.plant.CalcJacobianTranslationalVelocity(
            context, self.wrt, self.point_a.frame, self.point_a.point, world, world)
        J_b = self.plant.CalcJacobianTranslationalVelocity(
            context, self.wrt, self.point_b.frame, self.point_b.point, world, world)

        return J_a - J_b

    def _calc_rel_bias(self, context: Context):
        world = self.plant.world_frame()
        J_a_dot_v = self.plant.CalcBiasTranslationalAcceleration(
            context, self.wrt, self.point_a.frame, self.point_a.point, world, world)
        J_b_dot_v = self.plant.CalcBiasTranslationalAcceleration(
            context, self.wrt, self.point_b.frame, self.point_b.point, world, world)
        return J_a_dot_v - J_b_dot_v

    def evaluate(self, context: Context):
        """ Evaluate the constraint """
        rel = self._calc_rel_pos(context)
        return np.array([np.linalg.norm(rel) - self.distance])

    def jacobian(self, context: Context):
        """
            Jacobian of ||pt_A - pt_B||, evaluated all in world frame, is
              (pt_A - pt_B)^T * (J_A - J_B) / ||pt_A - pt_B||

            The Jacobian is such that phi_dot = J * v

            :param context: state of the robot
            :return: Jacobian of the constraint
        """

        rel_pos = self._calc_rel_pos(context)
        rel_jac = self._calc_rel_jac(context)

        return rel_pos.T @ rel_jac / np.linalg.norm(rel_pos)

    def jacobian_dot_times_v(self, context: Context):
        """
        From applying the chain rule to Jacobian, Jdot * v is
         ||(J_A - J_B) * v||^2/phi ...
           + (pt_A - pt_B)^T * (J_A_dot * v  -J_B_dot * v) / phi ...
           - phidot * (pt_A - pt_B)^T (J_A - J_B) *v / phi^2

        :param context: state of the robot
        :return: Jdot * v, where phi_ddot = Jdot * v + J * vdot
        """
        rel_pos = self._calc_rel_pos(context)
        rel_jac = self._calc_rel_jac(context)
        rel_bias = self._calc_rel_bias(context)

        phi = np.linalg.norm(rel_pos)
        phi_dot = self.jacobian(context) @ self.plant.GetVelocities(context).ravel()
        rel_vel = rel_jac @ self.plant.GetVelocities(context).ravel()
        return (rel_vel.T @ rel_vel / phi) + \
               (rel_pos.T @ rel_bias / phi) - \
               (phi_dot * rel_pos.T @ rel_vel / (phi * phi))


class ContactConstraint:
    """
        Constraint that a point remains fixed in the world.
        Everything is expressed in the world frame unless a different frame is provided
    """
    def __init__(self, plant: MultibodyPlant, contact_point: PointOnFrame, frame_to_express_in: Frame=None):
        self.plant = plant
        self.contact = contact_point
        self.frame_E = frame_to_express_in if frame_to_express_in is not None else plant.world_frame()
        self.wrt = JacobianWrtVariable.kV

    def evaluate(self, context: Context):
        """
        :param context: state of the robot
        :return: the point of the point in the world
        """
        return self.plant.CalcPointsPositions(
            context, self.contact.frame, self.contact.point, self.frame_E
        ).ravel()

    def jacobian(self, context: Context):
        return self.plant.CalcJacobianTranslationalVelocity(
            context, self.wrt, self.contact.frame, self.contact.point,
            self.plant.world_frame(), self.frame_E)

    def jacobian_dot_times_v(self, context: Context):
        self.plant.CalcBiasTranslationalAcceleration(
            context, self.wrt, self.contact.frame, self.contact.point,
            self.plant.world_frame(), self.frame_E).ravel()


class StackedConstraint:
    def __init__(self, constraints: List[DistanceConstraint | ContactConstraint]):
        self.constraints = constraints

    def evaluate(self, context):
        return np.concatenate(
            [c.evaluate(context) for c in self.constraints]
        )

    def jacobian(self, context):
        return np.vstack(
            [c.jacobian(context).reshape(-1, c.plant.num_velocities()) for c in self.constraints]
        )

    def jacobian_dot_times_v(self, context):
        return np.concatenate(
            [c.jacobian_dot_times_v(context) for c in self.constraints]
        )