"""
Plots trajectory data produced by the booster landing simulation.

Provides functions for visualizing the vehicle's altitude, velocity,
mass, ground track, and instantaneous forces/acceleration throughout
descent and landing.
"""

import math
import numpy as np
import matplotlib.pyplot as plt

from vehicle import VehicleState
from dynamics import Dynamics


def plot_simulation(
    history: list[dict], dynamics: Dynamics, output_path: str = "simulation_results.png"
) -> None:
    """
    Plots the vehicle's state and instantaneous forces throughout the
    simulation, as a single combined figure.

    Parameters:
        history: List containing the recorded vehicle states and controls.
        dynamics: Dynamics object, used to recompute drag/net force/
            acceleration at each logged state using the same physics the
            simulation actually used (rather than duplicating the formulas
            here).
        output_path: File path to save the combined figure as a PNG.

    Returns:
        None
    """
    time = [state["time"] for state in history]

    altitude = [state["position"][2] for state in history]
    x_position = [state["position"][0] for state in history]
    y_position = [state["position"][1] for state in history]

    speed = [np.linalg.norm(state["velocity"]) for state in history]
    vertical_velocity = [state["velocity"][2] for state in history]

    mass = [state["mass"] for state in history]

    flight_path_angle = [
        math.degrees(
            math.atan2(
                state["velocity"][2],
                math.sqrt(state["velocity"][0] ** 2 + state["velocity"][1] ** 2),
            )
        )
        for state in history
    ]

    # Reconstruct a VehicleState at each logged instant and reuse Dynamics'
    # own force/acceleration methods, rather than re-deriving the formulas
    # here — keeps a single source of truth for the physics.
    drag_magnitude = []
    net_force_magnitude = []
    acceleration_magnitude = []
    for entry in history:
        reconstructed_state = VehicleState(
            position_vector=entry["position"],
            velocity_vector=entry["velocity"],
            mass=entry["mass"],
        )
        drag = dynamics.calculate_drag_force(reconstructed_state)
        net = dynamics.net_force(
            reconstructed_state, entry["throttle"], entry["thrust_direction"]
        )
        accel = dynamics.acceleration_vector(reconstructed_state, net)

        drag_magnitude.append(dynamics.calculate_vector_magnitude(drag))
        net_force_magnitude.append(dynamics.calculate_vector_magnitude(net))
        acceleration_magnitude.append(dynamics.calculate_vector_magnitude(accel))

    fig, axes = plt.subplots(4, 2, figsize=(14, 18), constrained_layout=True)

    axes[0, 0].plot(time, altitude)
    axes[0, 0].set_xlabel("Time (s)")
    axes[0, 0].set_ylabel("Altitude (m)")
    axes[0, 0].set_title("Altitude vs Time")
    axes[0, 0].grid()

    axes[0, 1].plot(time, speed, label="Speed")
    axes[0, 1].plot(time, vertical_velocity, label="Vertical Velocity")
    axes[0, 1].set_xlabel("Time (s)")
    axes[0, 1].set_ylabel("Velocity (m/s)")
    axes[0, 1].set_title("Velocity vs Time")
    axes[0, 1].legend()
    axes[0, 1].grid()

    axes[1, 0].plot(time, flight_path_angle)
    axes[1, 0].set_xlabel("Time (s)")
    axes[1, 0].set_ylabel("Flight Path Angle (deg)")
    axes[1, 0].set_title("Flight Path Angle vs Time")
    axes[1, 0].grid()

    axes[1, 1].plot(time, mass)
    axes[1, 1].set_xlabel("Time (s)")
    axes[1, 1].set_ylabel("Mass (kg)")
    axes[1, 1].set_title("Mass vs Time")
    axes[1, 1].grid()

    axes[2, 0].plot(time, drag_magnitude)
    axes[2, 0].set_xlabel("Time (s)")
    axes[2, 0].set_ylabel("Drag Force (N)")
    axes[2, 0].set_title("Drag Magnitude vs Time")
    axes[2, 0].grid()

    axes[2, 1].plot(time, net_force_magnitude)
    axes[2, 1].set_xlabel("Time (s)")
    axes[2, 1].set_ylabel("Net Force (N)")
    axes[2, 1].set_title("Net Force Magnitude vs Time")
    axes[2, 1].grid()

    axes[3, 0].plot(time, acceleration_magnitude)
    axes[3, 0].set_xlabel("Time (s)")
    axes[3, 0].set_ylabel("Acceleration (m/s^2)")
    axes[3, 0].set_title("Acceleration Magnitude vs Time")
    axes[3, 0].grid()

    axes[3, 1].plot(x_position, y_position)
    axes[3, 1].scatter([0], [0], color="red", marker="x", label="Pad")
    axes[3, 1].set_xlabel("X Position (m)")
    axes[3, 1].set_ylabel("Y Position (m)")
    axes[3, 1].set_title("Ground Track")
    axes[3, 1].legend()
    axes[3, 1].grid()

    fig.savefig(output_path, dpi=150)
    plt.show()
