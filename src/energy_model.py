"""
Energy Consumption Model

Responsible for calculating energy consumption and travel times for drone movements.
Follows Single Responsibility Principle.
"""

import numpy as np
from typing import Tuple, List


class EnergyModel:
    """
    Calculates energy consumption and travel time for drone movements.
    
    Energy consumption rates:
    - 50 J/m when ascending
    - 5 J/m when descending
    - 10 J/m for horizontal movement
    
    Speed model:
    - 1 m/s when ascending
    - 2 m/s when descending
    - 1.5 m/s for horizontal movement
    - For oblique: max(horizontal_time, vertical_time)
    """
    
    # Energy consumption rates (J/m)
    ENERGY_ASCENDING = 50.0    # J/m
    ENERGY_DESCENDING = 5.0    # J/m
    ENERGY_HORIZONTAL = 10.0   # J/m
    
    # Speed rates (m/s)
    SPEED_ASCENDING = 1.0      # m/s
    SPEED_DESCENDING = 2.0     # m/s
    SPEED_HORIZONTAL = 1.5     # m/s
    
    # Battery capacity in Joules
    BATTERY_EDIFICIO1 = 1.0 * 3600    # 1 Wh = 3600 J
    BATTERY_EDIFICIO2 = 6.0 * 3600    # 6 Wh = 21600 J
    
    @staticmethod
    def calculate_energy(point1: Tuple[float, float, float],
                        point2: Tuple[float, float, float]) -> float:
        dx = point2[0] - point1[0]
        dy = point2[1] - point1[1]
        dz = point2[2] - point1[2]
        
        # Horizontal distance (in xy plane)
        horizontal_dist = np.sqrt(dx**2 + dy**2)
        
        # Horizontal energy
        energy_horizontal = horizontal_dist * EnergyModel.ENERGY_HORIZONTAL
        
        # Vertical energy (depends on direction)
        if dz > 0:  # Ascending
            energy_vertical = dz * EnergyModel.ENERGY_ASCENDING
        else:  # Descending
            energy_vertical = abs(dz) * EnergyModel.ENERGY_DESCENDING
        
        total_energy = energy_horizontal + energy_vertical
        
        return total_energy
    
    @staticmethod
    def calculate_time(point1: Tuple[float, float, float],
                      point2: Tuple[float, float, float]) -> float:
        dx = point2[0] - point1[0]
        dy = point2[1] - point1[1]
        dz = point2[2] - point1[2]
        
        # Horizontal distance and time
        horizontal_dist = np.sqrt(dx**2 + dy**2)
        time_horizontal = horizontal_dist / EnergyModel.SPEED_HORIZONTAL
        
        # Vertical distance and time
        if dz > 0:  # Ascending
            time_vertical = dz / EnergyModel.SPEED_ASCENDING
        else:  # Descending
            time_vertical = abs(dz) / EnergyModel.SPEED_DESCENDING
        
        # For oblique movements: max of the two components
        total_time = max(time_horizontal, time_vertical)
        
        return total_time
    
    @staticmethod
    def get_battery_capacity(building_name: str) -> float:
        if building_name.lower() == "edificio1":
            return EnergyModel.BATTERY_EDIFICIO1
        elif building_name.lower() == "edificio2":
            return EnergyModel.BATTERY_EDIFICIO2
        else:
            raise ValueError(f"Unknown building: {building_name}")
    
    @staticmethod
    def build_energy_matrix(nodes: List[Tuple]) -> np.ndarray:
        n = len(nodes)
        energy_matrix = np.zeros((n, n))
        
        for i, node_i in enumerate(nodes):
            point_i = (node_i[1], node_i[2], node_i[3])
            for j, node_j in enumerate(nodes):
                if i != j:
                    point_j = (node_j[1], node_j[2], node_j[3])
                    energy_matrix[i][j] = EnergyModel.calculate_energy(point_i, point_j)
        
        return energy_matrix
    
    @staticmethod
    def build_time_matrix(nodes: List[Tuple]) -> np.ndarray:
        n = len(nodes)
        time_matrix = np.zeros((n, n))
        
        for i, node_i in enumerate(nodes):
            point_i = (node_i[1], node_i[2], node_i[3])
            for j, node_j in enumerate(nodes):
                if i != j:
                    point_j = (node_j[1], node_j[2], node_j[3])
                    time_matrix[i][j] = EnergyModel.calculate_time(point_i, point_j)
        
        return time_matrix
    
    @staticmethod
    def is_trip_feasible(route: List[int],
                        nodes: List[Tuple],
                        battery_capacity: float) -> bool:
        total_energy = 0.0
        
        for i in range(len(route) - 1):
            idx1, idx2 = route[i], route[i + 1]
            point1 = (nodes[idx1][1], nodes[idx1][2], nodes[idx1][3])
            point2 = (nodes[idx2][1], nodes[idx2][2], nodes[idx2][3])
            
            total_energy += EnergyModel.calculate_energy(point1, point2)
            
            if total_energy > battery_capacity:
                return False
        
        return True
    
    @staticmethod
    def calculate_route_energy(route: List[int], nodes: List[Tuple]) -> float:
        total_energy = 0.0
        
        for i in range(len(route) - 1):
            idx1, idx2 = route[i], route[i + 1]
            point1 = (nodes[idx1][1], nodes[idx1][2], nodes[idx1][3])
            point2 = (nodes[idx2][1], nodes[idx2][2], nodes[idx2][3])
            total_energy += EnergyModel.calculate_energy(point1, point2)
        
        return total_energy
    
    @staticmethod
    def calculate_route_time(route: List[int], nodes: List[Tuple]) -> float:
        total_time = 0.0
        
        for i in range(len(route) - 1):
            idx1, idx2 = route[i], route[i + 1]
            point1 = (nodes[idx1][1], nodes[idx1][2], nodes[idx1][3])
            point2 = (nodes[idx2][1], nodes[idx2][2], nodes[idx2][3])
            total_time += EnergyModel.calculate_time(point1, point2)
        
        return total_time

