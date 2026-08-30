"""
Creates and runs a reusable booster landing simulation.

Initializes the vehicle, environment, vehicle state, dynamics, and simulation
objects before running and plotting the resulting trajectory.
"""

from pathlib import Path
import numpy as np

from vehicle import Vehicle, VehicleState
from environment import Environment
from dynamics import Dynamics
from simulation import Simulation
from guidance import Guidance
from plot import plot_simulation


def main():
    my_vehicle = Vehicle(
        name="Test Vehicle",
        length=40,
        diameter=3.7,
        dry_mass=25000,
        propellant_mass=3000,
        drag_coefficient=0.8,
        lift_coefficient=0.0,
        max_thrust=800000,
        specific_impulse=285,
    )
    my_vehicle_state = VehicleState(
        position_vector=np.array([500.0, 200.0, 3000.0]),
        velocity_vector=np.array([-40.0, -15.0, -600.0]),
        mass=my_vehicle.initial_mass(),
    )
    # Earth = Environment(Path("./us_standard_atmopshere_1976_model_v1.csv"))
    earth = Environment(
        Path(
            "/Users/parin/Projects/reentry-sim/reentry-sim/us_standard_atmosphere_1976_model_v1.csv"
        )
    )
    my_dynamics = Dynamics(my_vehicle, my_vehicle_state, earth)
    my_guidance = Guidance(throttle=0, thrust_direction=np.array([0, 0, 0]))
    my_simulation = Simulation(
        dynamics=my_dynamics,
        state=my_vehicle_state,
        environment=earth,
        guidance=my_guidance,
        time_step=0.05,
        max_time=10000,
    )

    my_simulation.run()
    plot_simulation(my_simulation.history, my_dynamics)


if __name__ == "__main__":
    main()
