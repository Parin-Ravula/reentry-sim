import math

class Vehicle:
    def __init__(self, name: str, mass: float, nose_radius: float, drag_coefficient: float, lift_coefficient: float):
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
        return math.pi * self.nose_radius ** 2


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
    