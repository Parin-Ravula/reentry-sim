"""
Defines the equations of motion and aerodynamic calculations used in the
reentry simulation.

Dynamics class calculates the forces acting on the vehicle and
determines how the vehicle's state changes throughout atmospheric reentry.
"""

from vehicle import Vehicle, VehicleState
from environment import Environment

import math


class Dynamics:
    def __init__(self, vehicle: Vehicle, environment: Environment):
        self.vehicle = vehicle
        self.environment = environment

    def gravity(self, state: VehicleState) -> float:
        """
        Calculates acceleration due to gravity at a given altitude.

        Parameters:
            Self: The dynamics object.
            state: VehicleState object.
        Returns:
            Acceleration due to gravity - g (m/s^2): float.
        """
        return (
            self.environment.gravitational_parameter
            / (self.environment.earth_radius + state.altitude) ** 2
        )

    def dynamic_pressure(self, state: VehicleState) -> float:
        """
        Calculates dynamic pressure at a given altitude.

        Parameters:
            self: The dynamics object.
            state: VehicleState object.
        Returns:
            Dynamic Pressure (Pa): float.
        """
        density = self.environment.get_atmospheric_value(state.altitude)["density"]
        return 0.5 * density * state.velocity**2

    def mach_number(self, state: VehicleState) -> float:
        """
        Calculates mach number at a given altitude.

        Parameters:
            Self: The dynamics object.
            state: VehicleState object.
        Returns:
            Mach Number (n/a): float.
        """
        speed_of_sound = self.environment.get_atmospheric_value(state.altitude)[
            "speed_of_sound"
        ]
        return state.velocity / speed_of_sound

    def drag_force(self, state: VehicleState) -> float:
        """
        Calculates drag force at a given altitude.

        Parameters:
            Self: The dynamics object.
            state: VehicleState object.
        Returns:
            Drag Force (N): float.
        """
        return (
            self.dynamic_pressure(state)
            * self.vehicle.drag_coefficient
            * self.vehicle.reference_area()
        )

    def lift_force(self, state: VehicleState) -> float:
        """
        Calculates lift force at a given altitude.

        Parameters:
            Self: The dynamics object.
            state: VehicleState object
        Returns:
            Lift Force (N): float
        """
        return (
            self.dynamic_pressure(state)
            * self.vehicle.lift_coefficient
            * self.vehicle.reference_area()
        )

    def altitude_rate(self, state: VehicleState) -> float:
        """
        Calculates vehicle's rate of change in altitude

        Parameters:
            Self: The dynamics object.
            state: VehicleState object
        Returns:
            Altitude Rate (m/s): float
        """
        return state.velocity * math.sin(math.radians(state.flight_path_angle))

    def velocity_rate(self, state: VehicleState) -> float:
        """
        Calculates the vehicle's rate of change in velocity.

        Parameters:
            Self: The dynamics object.
            state: VehicleState object

        Returns:
            Velocity Rate (m/s^2): float
        """
        return -(self.drag_force(state) / self.vehicle.mass) - self.gravity(
            state
        ) * math.sin(math.radians(state.flight_path_angle))

    def flight_path_angle_rate(self, state: VehicleState) -> float:
        """
        Calculates the vehicle's rate of change in flight path angle.

        Parameters:
            Self: The dynamics object.
            state: VehicleState object

        Returns:
            Flight Path Angle Rate (deg/s): float
        """
        radius = self.environment.earth_radius + state.altitude

        return math.degrees(
            (self.lift_force(state) * math.cos(math.radians(state.bank_angle)))
            / (self.vehicle.mass * state.velocity)
            + ((state.velocity / radius) - (self.gravity(state) / state.velocity))
            * math.cos(math.radians(state.flight_path_angle))
        )

    def latitude_rate(self, state: VehicleState) -> float:
        """
        Calculates the vehicle's latitude rate.

        Parameters:
            Self: The dynamics object.
            state: VehicleState object

        Returns:
            Latitude Rate (deg/s): float
        """
        return math.degrees(
            (
                state.velocity
                * math.cos(math.radians(state.flight_path_angle))
                * math.cos(math.radians(state.heading))
            )
            / (self.environment.earth_radius + state.altitude)
        )

    def longitude_rate(self, state: VehicleState) -> float:
        """
        Calculates the vehicle's longitude rate.

        Parameters:
            Self: The dynamics object.
            state: VehicleState object

        Returns:
            Longitude Rate (deg/s): float
        """
        return math.degrees(
            (
                state.velocity
                * math.cos(math.radians(state.flight_path_angle))
                * math.sin(math.radians(state.heading))
            )
            / (
                (self.environment.earth_radius + state.altitude)
                * math.cos(math.radians(state.latitude))
            )
        )

    def heading_rate(self, state: VehicleState) -> float:
        """
        Calculates the vehicle's heading rate.

        Parameters:
            Self: The dynamics object.
            state: VehicleState object

        Returns:
            Heading Rate (deg/s): float
        """
        return math.degrees(
            (self.lift_force(state) * math.sin(math.radians(state.bank_angle)))
            / (
                self.vehicle.mass
                * state.velocity
                * math.cos(math.radians(state.flight_path_angle))
            )
            + (
                state.velocity
                * math.cos(math.radians(state.flight_path_angle))
                * math.sin(math.radians(state.heading))
                * math.tan(math.radians(state.latitude))
            )
            / (self.environment.earth_radius + state.altitude)
        )

    def state_derivatives(self, state: VehicleState) -> dict:
        """
        Calculates all vehicle state derivatives.

        Parameters:
            Self: The dynamics object.
            state: VehicleState object

        Returns:
            dict: The current derivatives of the vehicle state.
        """
        return {
            "altitude": self.altitude_rate(state),
            "latitude": self.latitude_rate(state),
            "longitude": self.longitude_rate(state),
            "velocity": self.velocity_rate(state),
            "flight_path_angle": self.flight_path_angle_rate(state),
            "heading": self.heading_rate(state),
        }
