"""
Defines the vehicle and vehicle state classes.

Vehicle class stores the physical, aerodynamic, and propulsion properties
of the vehicle. VehicleState class stores the vehicle's current position,
motion, and mass throughout the simulation.
"""

import math
import numpy as np


class Vehicle:
    def __init__(
        self,
        name: str,
        length: float,
        diameter: float,
        dry_mass: float,
        propellant_mass: float,
        drag_coefficient: float,
        lift_coefficient: float,
        max_thrust: float,
        specific_impulse: float,
        minimum_throttle: float,  # ADD
        maximum_gimbal_angle: float,  # ADD
    ):
        self.name = name
        self.length = length
        self.diameter = diameter
        self.dry_mass = dry_mass
        self.propellant_mass = propellant_mass
        self.drag_coefficient = drag_coefficient
        self.lift_coefficient = lift_coefficient
        self.max_thrust = max_thrust
        self.specific_impulse = specific_impulse
        self.min_throttle = minimum_throttle
        self.max_gimbal_angle = maximum_gimbal_angle

    ### Maybe make an input?
    def reference_area(self):
        """
        Calculates vehicle's reference area.

        Parameters:
            Self: The vehicle object.

        Returns:
            Reference Area (m^2): float
        """
        return math.pi * (self.diameter / 2) ** 2

    def initial_mass(self) -> float:
        """
        Calculates vehicle's initial total mass.

        Parameters:
            Self: The vehicle object.

        Returns:
            Initial Mass (kg): float
        """
        return self.dry_mass + self.propellant_mass

    def ballistic_coefficient(self, mass: float, coefficient_drag: float) -> float:
        """
        Calculates vehicle's ballistic coefficient based on its current mass
        and reference area.

        Parameters:
            Self: The vehicle object.
            mass (float): The current vehicle mass (kg).
            coefficient_drag (float): The drag coefficient

        Returns:
            Ballistic Coefficient (kg/m^2): float
        """
        return mass / (coefficient_drag * self.reference_area())


class VehicleState:
    def __init__(
        self,
        position_vector: np.ndarray,
        velocity_vector: np.ndarray,
        flight_path_angle: float,
        heading: float,
        mass: float,
    ):
        self.position_vector = position_vector
        self.velocity_vector = velocity_vector
        self.flight_path_angle = flight_path_angle
        self.heading = heading
        self.mass = mass
