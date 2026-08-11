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

        with open(self.input_file_path, mode='r') as file:
            reader = csv.DictReader(file)

            for row in reader:
                atmosphere_data.append({
                    'altitude': float(row['Altitude_m']),
                    'temperature': float(row['Temperature_K']),
                    'pressure': float(row['Pressure_Pa']),
                    'density': float(row['Density_kg_m3']),
                    'speed_of_sound': float(row['SpeedOfSound_m_s']),
                    'dynamic_viscosity': float(row['DynamicViscosity_Pa_s'])
                })

        return atmosphere_data


    def get_density(self, altitude: float) -> float:
        """
        Retrieves the atmospheric density at a given altitude.

        Parameters:
            Self: The environment object.
            altitude (float): The altitude (m) in meters.

        Returns:
            float: The atmospheric density (kg/m^3) at the specified altitude.
        """
        if altitude <= self.atmosphere_data[0]['altitude']:
            print(f"NOTE: Density value at {altitude} may not be accurate")
            return self.atmosphere_data[0]['density']
    
        if altitude >= self.atmosphere_data[-1]['altitude']:
            print(f"NOTE: Density value at {altitude} may not be accurate")
            return self.atmosphere_data[-1]['density']

        upper_index = 0
        lower_index = 0
        while altitude >= self.atmosphere_data[upper_index]['altitude']:
            if altitude == self.atmosphere_data[upper_index]['altitude']:
                return self.atmosphere_data[upper_index]['density']
            upper_index += 1
        lower_index = upper_index - 1

        lower_altitude = self.atmosphere_data[lower_index]['altitude']
        upper_altitude = self.atmosphere_data[upper_index]['altitude']
        lower_density = self.atmosphere_data[lower_index]['density']
        upper_density = self.atmosphere_data[upper_index]['density']

        interpolated_density = (((upper_density - lower_density) / 
                                 (upper_altitude - lower_altitude)) * 
                                 (altitude - lower_altitude) + lower_density)
        return interpolated_density


    def get_pressure(self, altitude: float) -> float:
        """
        Retrieves the atmospheric pressure at a given altitude.

        Parameters:
            Self: The environment object.
            altitude (float): The altitude (m) in meters.

        Returns:
            float: The atmospheric pressure (Pa) at the specified altitude.
        """
        if altitude <= self.atmosphere_data[0]['altitude']:
            print(f"NOTE: Pressure value at {altitude} may not be accurate")
            return self.atmosphere_data[0]['pressure']
    
        if altitude >= self.atmosphere_data[-1]['altitude']:
            print(f"NOTE: Pressure value at {altitude} may not be accurate")
            return self.atmosphere_data[-1]['pressure']

        upper_index = 0
        lower_index = 0
        while altitude >= self.atmosphere_data[upper_index]['altitude']:
            if altitude == self.atmosphere_data[upper_index]['altitude']:
                return self.atmosphere_data[upper_index]['pressure']
            upper_index += 1
        lower_index = upper_index - 1

        lower_altitude = self.atmosphere_data[lower_index]['altitude']
        upper_altitude = self.atmosphere_data[upper_index]['altitude']
        lower_pressure = self.atmosphere_data[lower_index]['pressure']
        upper_pressure = self.atmosphere_data[upper_index]['pressure']

        interpolated_pressure = (((upper_pressure - lower_pressure) / 
                                 (upper_altitude - lower_altitude)) * 
                                 (altitude - lower_altitude) + lower_pressure)
        return interpolated_pressure

    
    def get_temperature(self, altitude: float) -> float:
        """
        Retrieves the temperature at a given altitude.
        
        Parameters:
            Self: The environment object.
            altitude (float): The altitude (m) in meters.
        
        Returns:
            float: The temperature (K) at the specified altitude.
        """
        if altitude <= self.atmosphere_data[0]['altitude']:
            print(f"NOTE: Temperature value at {altitude} may not be accurate")
            return self.atmosphere_data[0]['temperature']
    
        if altitude >= self.atmosphere_data[-1]['altitude']:
            print(f"NOTE: Temperature value at {altitude} may not be accurate")
            return self.atmosphere_data[-1]['temperature']

        upper_index = 0
        lower_index = 0
        while altitude >= self.atmosphere_data[upper_index]['altitude']:
            if altitude == self.atmosphere_data[upper_index]['altitude']:
                return self.atmosphere_data[upper_index]['temperature']
            upper_index += 1
        lower_index = upper_index - 1

        lower_altitude = self.atmosphere_data[lower_index]['altitude']
        upper_altitude = self.atmosphere_data[upper_index]['altitude']
        lower_temperature = self.atmosphere_data[lower_index]['temperature']
        upper_temperature = self.atmosphere_data[upper_index]['temperature']

        interpolated_temperature = (((upper_temperature - lower_temperature) / 
                                 (upper_altitude - lower_altitude)) * 
                                 (altitude - lower_altitude) + lower_temperature)
        return interpolated_temperature


    def get_atmospheric_value(self, altitude: float, data_type: str) -> float:
        """
        Retrieves the an atmospherric value at a given altitude.
        
        Parameters:
            Self: The environment object.
            altitude (float): The altitude (m) in meters.
        
        Returns:
            Any of the following
            - float: The temperature (K) at the specified altitude.
            - float: The atmospheric pressure (Pa) at the specified altitude.
            - float: The atmospheric density (kg/m^3) at the specified altitude.
        """

        if altitude <= self.atmosphere_data[0]['altitude']:
            print(f"NOTE: {data_type} value at {altitude} may not be accurate")
            return self.atmosphere_data[0][data_type]
    
        if altitude >= self.atmosphere_data[-1]['altitude']:
            print(f"NOTE: Temperature value at {altitude} may not be accurate")
            return self.atmosphere_data[-1][data_type]

        upper_index = 0
        lower_index = 0
        while altitude >= self.atmosphere_data[upper_index]['altitude']:
            if altitude == self.atmosphere_data[upper_index]['altitude']:
                return self.atmosphere_data[upper_index][data_type]
            upper_index += 1
        lower_index = upper_index - 1

        lower_altitude = self.atmosphere_data[lower_index]['altitude']
        upper_altitude = self.atmosphere_data[upper_index]['altitude']
        lower_data_type = self.atmosphere_data[lower_index][data_type]
        upper_data_type = self.atmosphere_data[upper_index][data_type]

        interpolated_data_type = (((upper_data_type - lower_data_type) / 
                                 (upper_altitude - lower_altitude)) * 
                                 (altitude - lower_altitude) + lower_data_type)
        return interpolated_data_type