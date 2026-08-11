"""
Defines the vehicle and vehicle state classes.

Vehicle class stores the physical and aerodynamic properties of the vehicle.
VehicleState class stores the vehicle's current position and motion.
"""

import math


class Vehicle:
    def __init__(
        self,
        name: str,
        mass: float,
        nose_radius: float,
        drag_coefficient: float,
        lift_coefficient: float,
    ):
        self.name = name
        self.mass = mass
        self.nose_radius = nose_radius
        self.drag_coefficient = drag_coefficient
        self.lift_coefficient = lift_coefficient

    def reference_area(self) -> float:
        """
        Calculates vehicle's reference area based on its nose radius.

        Parameters:
            Self: The vehicle object.

        Returns:
            float: The reference area of the vehicle.
        """
        return math.pi * self.nose_radius**2

    def ballistic_coefficient(self) -> float:
        """
        Calculates vehicle's ballistic coefficient based on its mass and reference area.

        Parameters:
            Self: The vehicle object.

        Returns:
            float: The ballistic coefficient of the vehicle.
        """
        return self.mass / (self.drag_coefficient * self.reference_area())

    def lift_to_drag_ratio(self) -> float:
        """
        Calculates vehicle's lift-to-drag ratio based on its lift and drag coefficients.

        Parameters:
            Self: The vehicle object.

        Returns:
            float: The lift-to-drag ratio of the vehicle.
        """
        return self.lift_coefficient / self.drag_coefficient


class VehicleState:
    def __init__(
        self,
        altitude: float,
        latitude: float,
        longitude: float,
        velocity: float,
        flight_path_angle: float,
        heading: float,
        bank_angle: float,
    ):
        self.altitude = altitude
        self.latitude = latitude
        self.longitude = longitude
        self.velocity = velocity
        self.flight_path_angle = flight_path_angle
        self.heading = heading
        self.bank_angle = bank_angle
