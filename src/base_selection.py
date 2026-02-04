
import numpy as np
from typing import List, Tuple


class BaseSelector:
    """
    Selects optimal base location from candidate points.
    
    Strategy: Find the candidate point closest to the centroid of all grid points.
    This minimizes average distance to all points.
    """
    
    def __init__(self, grid_points: List[Tuple[float, float, float]]):
        self.grid_points = grid_points
        self.centroid = self._calculate_centroid()
    
    def _calculate_centroid(self) -> Tuple[float, float, float]:
        if not self.grid_points:
            raise ValueError("Cannot calculate centroid of empty point set")
        
        x_coords = [p[0] for p in self.grid_points]
        y_coords = [p[1] for p in self.grid_points]
        z_coords = [p[2] for p in self.grid_points]
        
        centroid_x = np.mean(x_coords)
        centroid_y = np.mean(y_coords)
        centroid_z = np.mean(z_coords)
        
        return (centroid_x, centroid_y, centroid_z)
    
    def select_base(self, 
                    x_range: Tuple[int, int],
                    y_range: Tuple[int, int],
                    z_value: int,
                    y_threshold: float) -> Tuple[int, int, int]:
        """
        Select the best base location from candidate integer points.
        
        """
        candidates = self._generate_candidates(x_range, y_range, z_value, y_threshold)
        
        if not candidates:
            raise ValueError("No valid candidate points found with given constraints")
        
        # Find candidate closest to centroid
        best_base = min(candidates, key=lambda c: self._distance_to_centroid(c))
        
        print(f"\nBase Selection:")
        print(f"  Centroid of grid points: ({self.centroid[0]:.2f}, {self.centroid[1]:.2f}, {self.centroid[2]:.2f})")
        print(f"  Selected base location: {best_base}")
        print(f"  Distance to centroid: {self._distance_to_centroid(best_base):.2f} m")
        
        return best_base
    
    def _generate_candidates(self,
                            x_range: Tuple[int, int],
                            y_range: Tuple[int, int],
                            z_value: int,
                            y_threshold: float) -> List[Tuple[int, int, int]]:
        candidates = []
        
        for x in range(x_range[0], x_range[1] + 1):
            for y in range(y_range[0], y_range[1] + 1):
                if y <= y_threshold:
                    candidates.append((x, y, z_value))
        
        return candidates
    
    def _distance_to_centroid(self, point: Tuple[int, int, int]) -> float:
        dx = point[0] - self.centroid[0]
        dy = point[1] - self.centroid[1]
        dz = point[2] - self.centroid[2]
        
        return np.sqrt(dx**2 + dy**2 + dz**2)


def select_base_for_building(grid_points: List[Tuple[float, float, float]],
                             building_name: str) -> Tuple[int, int, int]:
    selector = BaseSelector(grid_points)
    
    if building_name.lower() == "edificio1":
        # Edificio1 constraints
        return selector.select_base(
            x_range=(-8, 5),
            y_range=(-17, -15),
            z_value=0,
            y_threshold=-12.5
        )
    elif building_name.lower() == "edificio2":
        # Edificio2 constraints
        return selector.select_base(
            x_range=(-10, 10),
            y_range=(-31, -30),
            z_value=0,
            y_threshold=-20.0
        )
    else:
        raise ValueError(f"Unknown building: {building_name}")

