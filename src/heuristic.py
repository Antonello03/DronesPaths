"""
Heuristic Solution Builder

Greedy battery-aware heuristic for multi-trip drone routing.
Follows Single Responsibility Principle.
"""

import numpy as np
from typing import List, Tuple, Set
from src.energy_model import EnergyModel


class GreedyTripBuilder:
    """
    Builds multi-trip solution using greedy nearest-neighbor with battery awareness.
    
    Strategy:
    1. Start from base with full battery
    2. Greedily select nearest unvisited point that keeps trip feasible
    3. When no more points can be added, return to base (new trip)
    4. Repeat until all points covered
    """
    
    def __init__(self,
                 nodes: List[Tuple],
                 connection_matrix: np.ndarray,
                 battery_capacity: float):
        self.nodes = nodes
        self.connection_matrix = connection_matrix
        self.battery_capacity = battery_capacity
        self.n = len(nodes)
    
    def build_solution(self) -> List[List[int]]:
        trips = []
        unvisited = set(range(1, self.n))  # All grid points (excluding base)
        
        trip_number = 1
        max_trips = self.n + 10  # Safety limit
        
        while unvisited and trip_number <= max_trips:
            trip = self._build_single_trip(unvisited)
            
            # Remove visited points
            visited_in_trip = set(trip[1:-1])  # Exclude base at start and end
            
            # Safety check: if no points visited, we're stuck
            if not visited_in_trip:
                print(f"\n⚠️  Warning: Could not reach {len(unvisited)} remaining points from base")
                print(f"     Unreachable points: {sorted(list(unvisited))[:20]}...")
                break
            
            unvisited -= visited_in_trip
            trips.append(trip)
            
            # Progress info
            energy = EnergyModel.calculate_route_energy(trip, self.nodes)
            time = EnergyModel.calculate_route_time(trip, self.nodes)
            print(f"  Trip {trip_number}: {len(visited_in_trip)} points, "
                  f"{energy:.0f}/{self.battery_capacity:.0f} J, {time:.1f}s")
            
            trip_number += 1
        
        return trips
    
    def _build_single_trip(self, available_points: Set[int]) -> List[int]:
        trip = [0]  # Start at base
        current = 0
        remaining = available_points.copy()
        
        while remaining:
            # Find nearest feasible point
            next_point = self._select_next_point(trip, current, remaining)
            
            if next_point is None:
                # No more points can be added within battery constraint
                break
            
            trip.append(next_point)
            remaining.remove(next_point)
            current = next_point
        
        trip.append(0)  # Return to base
        
        return trip
    
    def _select_next_point(self,
                          current_trip: List[int],
                          current_position: int,
                          candidates: Set[int]) -> int:
        best_point = None
        best_distance = float('inf')
        
        # Try each candidate in order of distance
        for candidate in candidates:
            # Check if connection exists
            if not self.connection_matrix[current_position][candidate]:
                continue
            
            # Check if adding this point keeps trip feasible
            test_trip = current_trip + [candidate, 0]  # Test trip with return to base
            
            if EnergyModel.is_trip_feasible(test_trip, self.nodes, self.battery_capacity):
                # Calculate distance to candidate
                point_current = (self.nodes[current_position][1],
                               self.nodes[current_position][2],
                               self.nodes[current_position][3])
                point_candidate = (self.nodes[candidate][1],
                                 self.nodes[candidate][2],
                                 self.nodes[candidate][3])
                
                distance = self._euclidean_distance(point_current, point_candidate)
                
                if distance < best_distance:
                    best_distance = distance
                    best_point = candidate
        
        return best_point
    
    @staticmethod
    def _euclidean_distance(p1: Tuple[float, float, float],
                           p2: Tuple[float, float, float]) -> float:
        dx = p2[0] - p1[0]
        dy = p2[1] - p1[1]
        dz = p2[2] - p1[2]
        return np.sqrt(dx**2 + dy**2 + dz**2)


def build_heuristic_solution(nodes: List[Tuple],
                             connection_matrix: np.ndarray,
                             battery_capacity: float) -> dict:
    print("\n" + "="*60)
    print("Building Heuristic Solution (Greedy Battery-Aware)")
    print("="*60)
    print(f"Battery capacity: {battery_capacity:.0f} J ({battery_capacity/3600:.2f} Wh)")
    print(f"Grid points to visit: {len(nodes) - 1}")
    print()
    
    builder = GreedyTripBuilder(nodes, connection_matrix, battery_capacity)
    trips = builder.build_solution()
    
    # Calculate statistics
    total_time = sum(EnergyModel.calculate_route_time(trip, nodes) for trip in trips)
    total_energy = sum(EnergyModel.calculate_route_energy(trip, nodes) for trip in trips)
    
    # Verify all points are covered
    all_visited = set()
    for trip in trips:
        all_visited.update(trip[1:-1])  # Exclude base
    
    expected_points = set(range(1, len(nodes)))
    if all_visited != expected_points:
        missing = expected_points - all_visited
        print(f"\n⚠️  Warning: {len(missing)} points not covered: {missing}")
    
    print()
    print(f"Heuristic solution complete:")
    print(f"  Total trips: {len(trips)}")
    print(f"  Total time: {total_time:.2f} seconds")
    print(f"  Total energy: {total_energy:.0f} J")
    print(f"  Points covered: {len(all_visited)}/{len(nodes)-1}")
    print("="*60)
    
    return {
        'trips': trips,
        'total_time': total_time,
        'total_energy': total_energy,
        'num_trips': len(trips)
    }

