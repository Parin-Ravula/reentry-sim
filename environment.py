"""
Defines the atmospheric environment model used in the reentry simulation.

Environment class loads atmospheric data and provides atmospheric
properties at a given altitude using linear interpolation.
"""

import csv
from pathlib import Path


class Environment:
    def __init__(self, input_file_path: Path):
        self.earth_radius = 6371000  # meters
        self.gravitational_parameter = 3.986e14  # m^3/s^2
        self.input_file_path = input_file_path
        self.atmosphere_data = self.load_atmosphere_data()

    def load_atmosphere_data(self) -> list:
        """
        Loads atmospheric data from a CSV file.

        Parameters:
            Self: The environment object.
        Returns:
            list: A list of dictionaries containing atmospheric data.
        """
        atmosphere_data = []

        with open(self.input_file_path, mode="r") as file:
            reader = csv.DictReader(file)

            for row in reader:
                atmosphere_data.append(
                    {
                        "altitude": float(row["Altitude_m"]),
                        "temperature": float(row["Temperature_K"]),
                        "pressure": float(row["Pressure_Pa"]),
                        "density": float(row["Density_kg_m3"]),
                        "speed_of_sound": float(row["SpeedOfSound_m_s"]),
                        "dynamic_viscosity": float(row["DynamicViscosity_Pa_s"]),
                    }
                )

        return atmosphere_data

    def get_atmospheric_values(self, altitude: float) -> dict:
        """
        Retrieves atmospheric values at a given altitude.

        Parameters:
            Self: The environment object.
            altitude (float): The altitude (m) in meters.

        Returns:
            dict: Atmospheric values at the specified altitude.
            - float: Altitude (m)
            - float: Temperature (K)
            - float: Pressure (Pa)
            - float: Density (kg/m3)
            - float: Speed of Sound (m/s)
            - float: Dynamic Viscosity (Pa/s)
        """

        def interpolate(lower_value, upper_value, lower_altitude, upper_altitude):
            return ((upper_value - lower_value) / (upper_altitude - lower_altitude)) * (
                altitude - lower_altitude
            ) + lower_value

        if altitude <= self.atmosphere_data[0]["altitude"]:
            print(f"NOTE: Atmospheric values at {altitude} may not be accurate")
            data = self.atmosphere_data[0]

            return {
                "altitude": altitude,
                "temperature": data["temperature"],
                "pressure": data["pressure"],
                "density": data["density"],
                "speed_of_sound": data["speed_of_sound"],
                "dynamic_viscosity": data["dynamic_viscosity"],
            }

        if altitude >= self.atmosphere_data[-1]["altitude"]:
            print(f"NOTE: Atmospheric values at {altitude} may not be accurate")
            data = self.atmosphere_data[-1]
            return {
                "altitude": altitude,
                "temperature": data["temperature"],
                "pressure": data["pressure"],
                "density": data["density"],
                "speed_of_sound": data["speed_of_sound"],
                "dynamic_viscosity": data["dynamic_viscosity"],
            }

        upper_index = 0

        while altitude >= self.atmosphere_data[upper_index]["altitude"]:
            if altitude == self.atmosphere_data[upper_index]["altitude"]:
                data = self.atmosphere_data[upper_index]
                return {
                    "altitude": altitude,
                    "temperature": data["temperature"],
                    "pressure": data["pressure"],
                    "density": data["density"],
                    "speed_of_sound": data["speed_of_sound"],
                    "dynamic_viscosity": data["dynamic_viscosity"],
                }
            upper_index += 1

        lower_index = upper_index - 1

        lower_data = self.atmosphere_data[lower_index]
        upper_data = self.atmosphere_data[upper_index]

        lower_altitude = lower_data["altitude"]
        upper_altitude = upper_data["altitude"]

        return {
            "altitude": altitude,
            "temperature": interpolate(
                lower_data["temperature"],
                upper_data["temperature"],
                lower_altitude,
                upper_altitude,
            ),
            "pressure": interpolate(
                lower_data["pressure"],
                upper_data["pressure"],
                lower_altitude,
                upper_altitude,
            ),
            "density": interpolate(
                lower_data["density"],
                upper_data["density"],
                lower_altitude,
                upper_altitude,
            ),
            "speed_of_sound": interpolate(
                lower_data["speed_of_sound"],
                upper_data["speed_of_sound"],
                lower_altitude,
                upper_altitude,
            ),
            "dynamic_viscosity": interpolate(
                lower_data["dynamic_viscosity"],
                upper_data["dynamic_viscosity"],
                lower_altitude,
                upper_altitude,
            ),
        }
