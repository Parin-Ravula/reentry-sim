"""
Runs the reentry simulation and manages the vehicle's state over time.

Simulation class uses the vehicle dynamics to update the vehicle's state
at each time step and stores the resulting trajectory throughout reentry.
"""

from vehicle import VehicleState
from dynamics import Dynamics


class Simulation:
    def __init__(
        self,
        dynamics: Dynamics,
        state: VehicleState,
        time_step: float,
        max_time: float,
    ):
        self.dynamics = dynamics
        self.state = state
        self.time_step = time_step
        self.max_time = max_time

        self.current_time = 0.0
        self.history = []

    def record_state(self) -> None:
        self.history.append(
            {
                "altitude": self.state.altitude,
                "latitude": self.state.latitude,
                "longitude": self.state.longitude,
                "velocity": self.state.velocity,
                "flight_path_angle": self.state.flight_path_angle,
                "heading": self.state.heading,
                "bank_angle": self.state.bank_angle,
            }
        )

    def step(self) -> None:
        derivatives = self.dynamics.state_derivatives(self.state)

        self.state.altitude += derivatives["altitude"] * self.time_step
        self.state.latitude += derivatives["latitude"] * self.time_step
        self.state.longitude += derivatives["longitude"] * self.time_step
        self.state.velocity += derivatives["velocity"] * self.time_step
        self.state.flight_path_angle += (
            derivatives["flight_path_angle"] * self.time_step
        )
        self.state.heading += derivatives["heading"] * self.time_step

        self.current_time += self.time_step

    def run(self) -> None:
        while self.state.altitude > 0 and self.current_time < self.max_time:
            self.record_state()
            self.step()

        self.record_state()
        print(self.history[-1])
