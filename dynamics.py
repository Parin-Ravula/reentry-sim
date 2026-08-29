"""
Defines the equations of motion, aerodynamic calculations, and propulsion
calculations used in the reusable booster simulation.

Dynamics class calculates the forces acting on the vehicle and determines
how the vehicle's state changes throughout atmospheric descent and landing.
"""

from vehicle import Vehicle, VehicleState
from environment import Environment

import math
import numpy as np


class Dynamics:
    def __init__(self, vehicle: Vehicle, environment: Environment):
        self.vehicle = vehicle
        self.environment = environment

    ### Helper Functions
    def calculate_vector_magnitude(self, vector: np.ndarray):
        """
        Calculates the magnitude (length) of a 3D vector.

        Parameters:
            Self: The dynamics object.
            vector: A 3D vector (np.ndarray).

        Returns:
            Magnitude of the vector: float.
        """
        return math.sqrt((vector[0]) ** 2 + (vector[1]) ** 2 + (vector[2]) ** 2)

    def calculate_unit_vector(self, vector: np.ndarray):
        """
        Calculates the unit vector (direction, magnitude 1) of a 3D vector.

        Parameters:
            Self: The dynamics object.
            vector: A 3D vector (np.ndarray).

        Returns:
            Unit vector in the same direction as the input vector: np.ndarray.
        """
        ### Fix for when vehicle at rest
        magnitude = self.calculate_vector_magnitude(vector)
        return np.array(
            [vector[0] / magnitude, vector[1] / magnitude, vector[2] / magnitude]
        )

    def gravity(self, state: VehicleState) -> np.ndarray:
        """
        Calculates acceleration due to gravity at a given altitude.

        Parameters:
            Self: The dynamics object.
            state: VehicleState object.

        Returns:
            Acceleration due to gravity vector (m/s^2): np.ndarray.
        """
        return (
            self.environment.gravitational_parameter
            / (self.environment.earth_radius + state.position_vector[2]) ** 2
        ) * np.array([0, 0, -1])

    def dynamic_pressure(self, state: VehicleState) -> float:
        """
        Calculates dynamic pressure at a given altitude.

        Parameters:
            Self: The dynamics object.
            state: VehicleState object.

        Returns:
            Dynamic Pressure (Pa): float.
        """
        density = self.environment.get_atmospheric_value(state.position_vector[2])[
            "density"
        ]

        return (
            0.5 * density * self.calculate_vector_magnitude(state.velocity_vector) ** 2
        )

    def mach_number(self, state: VehicleState) -> float:
        """
        Calculates mach number at a given altitude.

        Parameters:
            Self: The dynamics object.
            state: VehicleState object.

        Returns:
            Mach Number (dimensionless): float.
        """
        speed_of_sound = self.environment.get_atmospheric_value(
            state.position_vector[2]
        )["speed_of_sound"]

        return self.calculate_vector_magnitude(state.velocity_vector) / speed_of_sound

    def calculate_drag_force(self, state: VehicleState) -> np.ndarray:
        """
        Calculates drag force acting on the vehicle.

        Parameters:
            Self: The dynamics object.
            state: VehicleState object.

        Returns:
            Drag Force vector (N): np.ndarray.
        """
        drag_force_scalar = (
            self.dynamic_pressure(state)
            * self.vehicle.drag_coefficient
            * self.vehicle.reference_area()
        )

        return (
            -1 * drag_force_scalar * self.calculate_unit_vector(state.velocity_vector)
        )

    def thrust_force(self, throttle: float, thrust_direction: np.ndarray) -> np.ndarray:
        """
        Calculates thrust force acting on the vehicle.

        Parameters:
            Self: The dynamics object.
            throttle: Throttle setting (0 to 1).
            thrust_direction: Direction vector for thrust (normalized internally).

        Returns:
            Thrust Force vector (N): np.ndarray.
        """
        thrust_direction = self.calculate_unit_vector(thrust_direction)
        return self.vehicle.max_thrust * throttle * thrust_direction

    def acceleration(self, state: VehicleState, net_force: np.ndarray) -> np.ndarray:
        """
        Calculates acceleration of the vehicle from a net force.

        Parameters:
            Self: The dynamics object.
            state: VehicleState object.
            net_force: Net force vector (N) acting on the vehicle.

        Returns:
            Acceleration vector (m/s^2): np.ndarray.
        """
        return net_force / state.mass

    def net_force(
        self, state: VehicleState, throttle: float, thrust_direction: np.ndarray
    ) -> np.ndarray:
        """
        Calculates the net force acting on the vehicle from gravity, drag,
        and thrust.

        Parameters:
            Self: The dynamics object.
            state: VehicleState object.
            throttle: Throttle setting (0 to 1).
            thrust_direction: Direction vector for thrust.

        Returns:
            Net force vector (N): np.ndarray.
        """
        gravity = self.gravity(state)
        drag = self.calculate_drag_force(state)
        thrust = self.thrust_force(throttle, thrust_direction)
        return thrust + drag + gravity * state.mass

    def position_rate(self, state: VehicleState) -> np.ndarray:
        """
        Calculates rate of change of position (i.e., velocity).

        Parameters:
            Self: The dynamics object.
            state: VehicleState object.

        Returns:
            Velocity vector (m/s): np.ndarray.
        """
        return state.velocity_vector

    def velocity_rate(
        self, state: VehicleState, throttle: float, thrust_direction: np.ndarray
    ) -> np.ndarray:
        """
        Calculates rate of change of velocity (i.e., acceleration).

        Parameters:
            Self: The dynamics object.
            state: VehicleState object.
            throttle: Throttle setting (0 to 1).
            thrust_direction: Unit vector for thrust direction.

        Returns:
            Acceleration vector (m/s^2): np.ndarray.
        """
        net_force = self.net_force(state, throttle, thrust_direction)
        return self.acceleration(state, net_force)

    def mass_flow_rate(
        self, thrust_force: np.ndarray, specific_impulse: float, g0: float
    ) -> float:
        """
        Calculates rate of change of mass due to propellant consumption.

        Parameters:
            Self: The dynamics object.
            thrust_force: Thrust force vector (N).
            specific_impulse: Specific impulse of the engine (s).
            g0: Standard gravity at Earth's surface (m/s^2).

        Returns:
            Mass flow rate (kg/s): float. Negative, since mass decreases
            as propellant is consumed.
        """
        return -self.calculate_vector_magnitude(thrust_force) / (specific_impulse * g0)

    def flight_path_angle(self, state: VehicleState) -> float:
        """
        Calculates the flight path angle (angle between the velocity vector
        and the local horizontal plane), derived from the velocity vector.

        Parameters:
            Self: The dynamics object.
            state: VehicleState object.

        Returns:
            Flight path angle (degrees): float.
        """
        horizontal_speed = math.sqrt(
            state.velocity_vector[0] ** 2 + state.velocity_vector[1] ** 2
        )
        return math.degrees(math.atan2(state.velocity_vector[2], horizontal_speed))

    def heading(self, state: VehicleState) -> float:
        """
        Calculates the heading (direction of horizontal velocity), derived
        from the velocity vector.

        Parameters:
            Self: The dynamics object.
            state: VehicleState object.

        Returns:
            Heading (degrees): float.
        """
        return math.degrees(
            math.atan2(state.velocity_vector[1], state.velocity_vector[0])
        )

    def state_derivatives(
        self, state: VehicleState, throttle: float, thrust_direction: np.ndarray
    ) -> dict:
        """
        Calculates all vehicle state derivatives.

        Parameters:
            Self: The dynamics object.
            state: VehicleState object.
            throttle: Throttle setting (0 to 1).
            thrust_direction: Unit vector for thrust direction.

        Returns:
            dict: The current derivatives of the vehicle state
                (position, velocity, and mass rates).
        """
        thrust = self.thrust_force(throttle, thrust_direction)
        mass_rate = self.mass_flow_rate(
            thrust, self.vehicle.specific_impulse, g0=9.80665
        )

        return {
            "position_vector": self.position_rate(state),
            "velocity_vector": self.velocity_rate(state, throttle, thrust_direction),
            "mass": mass_rate,
        }
