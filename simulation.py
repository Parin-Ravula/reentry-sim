"""
Runs the reusable booster simulation and manages the vehicle's state over time.

Simulation class uses the vehicle dynamics to update the vehicle's state
at each time step and stores the resulting trajectory throughout descent.
"""

from vehicle import VehicleState
from dynamics import Dynamics
from environment import Environment
from guidance import Guidance
import numpy as np


class Simulation:
    def __init__(
        self,
        dynamics: Dynamics,
        state: VehicleState,
        environment: Environment,
        guidance: Guidance,
        time_step: float,
        max_time: float,
    ):
        self.dynamics = dynamics
        self.state = state
        self.environment = environment
        self.guidance = guidance
        self.time_step = time_step
        self.max_time = max_time

        self.current_time = 0.0
        self.history = []

    def record_state(self, throttle: float, thrust_direction: np.ndarray) -> None:
        """
        Records the vehicle's current state, plus the control inputs about
        to be applied this step, in the simulation history.

        Parameters:
            Self: The simulation object.
            throttle: Throttle setting (0 to 1) commanded this step.
            thrust_direction: Thrust direction vector commanded this step.

        Returns:
            None
        """
        self.history.append(
            {
                "time": self.current_time,
                "position": self.state.position_vector.copy(),
                "velocity": self.state.velocity_vector.copy(),
                "mass": self.state.mass,
                "throttle": throttle,
                "thrust_direction": thrust_direction.copy(),
                "environment": self.environment.get_atmospheric_values(
                    self.state.position_vector[2]
                ),
            }
        )

    def step(self, throttle: float, thrust_direction: np.ndarray) -> None:
        """
        Advances the vehicle's state by one simulation time step, given the
        control inputs decided for this step.

        Parameters:
            Self: The simulation object.
            throttle: Throttle setting (0 to 1) commanded this step.
            thrust_direction: Thrust direction vector commanded this step.

        Returns:
            None
        """
        derivatives = self.dynamics.state_derivatives(
            self.state, throttle, thrust_direction
        )

        self.state.position_vector = (
            self.state.position_vector + derivatives["position_vector"] * self.time_step
        )
        self.state.velocity_vector = (
            self.state.velocity_vector + derivatives["velocity_vector"] * self.time_step
        )
        self.state.mass += derivatives["mass"] * self.time_step

        self.current_time += self.time_step

    def run(self) -> None:
        while self.state.position_vector[2] > 0 and self.current_time < self.max_time:
            throttle, thrust_direction = self.guidance.command(self.state)
            self.record_state(throttle, thrust_direction)
            self.step(throttle, thrust_direction)

        throttle, thrust_direction = self.guidance.command(self.state)
        self.record_state(throttle, thrust_direction)
        print(self.history[-1])
