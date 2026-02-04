"""
Multi-Trip MIP Model

MIP formulation for single-drone multi-trip routing with battery constraints.
Follows Single Responsibility and Open/Closed Principles.
"""

from mip import Model, xsum, minimize, BINARY, INTEGER, CONTINUOUS, OptimizationStatus
import numpy as np
from typing import List, Tuple, Optional
from src.energy_model import EnergyModel


class MultiTripModel:
    """
    MIP model for multi-trip drone routing with battery constraints.
    
    Decision variables:
    - x[t,i,j]: binary, =1 if drone uses arc (i,j) in trip t
    - y[t,i]: binary, =1 if point i is visited in trip t
    - u[t,i]: integer, MTZ order variable for subtour elimination
    - T_t: continuous, time for trip t
    
    Constraints:
    - Each point visited exactly once (across all trips)
    - Flow conservation per trip
    - MTZ subtour elimination per trip
    - Battery constraint per trip
    - Trip time calculation
    
    Objective: minimize total time (sum of all trip times)
    """
    
    def __init__(self,
                 nodes: List[Tuple],
                 connection_matrix: np.ndarray,
                 energy_matrix: np.ndarray,
                 time_matrix: np.ndarray,
                 battery_capacity: float,
                 max_trips: int = 20,
                 time_limit: int = 7200):
        self.nodes = nodes
        self.connection_matrix = connection_matrix
        self.energy_matrix = energy_matrix
        self.time_matrix = time_matrix
        self.battery_capacity = battery_capacity
        self.max_trips = max_trips
        self.time_limit = time_limit
        
        self.n = len(nodes) - 1  # Number of grid points
        self.N = list(range(len(nodes)))  # All nodes
        self.V = list(range(1, len(nodes)))  # Grid points only
        self.T = list(range(1, max_trips + 1))  # Trip indices
        
        # Build arc set
        self.A = set()
        for i in self.N:
            for j in self.N:
                if i != j and self.connection_matrix[i][j]:
                    self.A.add((i, j))
        
        self.model = None
        self.x = {}
        self.y = {}
        self.u = {}
        self.T_t = {}
        self.T_total = None
    
    def build_model(self):
        print("\n" + "="*60)
        print("Building Multi-Trip MIP Model")
        print("="*60)
        print(f"Grid points: {self.n}")
        print(f"Max trips: {self.max_trips}")
        print(f"Feasible arcs: {len(self.A)}")
        print(f"Battery capacity: {self.battery_capacity:.0f} J")
        print()
        
        self.model = Model("MultiTripDroneRouting")
        self.model.verbose = 1
        self.model.max_seconds = self.time_limit
        
        print("Creating variables...")
        self._create_variables()
        
        print("Setting objective...")
        self._set_objective()
        
        print("Adding constraints...")
        self._add_constraints()
        
        print()
        print(f"Model built successfully!")
        print(f"  Variables: {self.model.num_cols}")
        print(f"  Constraints: {self.model.num_rows}")
        print("="*60)
    
    def _create_variables(self):
        # x[t,i,j]: arc usage in trip t
        for t in self.T:
            for (i, j) in self.A:
                self.x[t, i, j] = self.model.add_var(
                    name=f"x_{t}_{i}_{j}",
                    var_type=BINARY
                )
        
        # y[t,i]: point i visited in trip t
        for t in self.T:
            for i in self.V:
                self.y[t, i] = self.model.add_var(
                    name=f"y_{t}_{i}",
                    var_type=BINARY
                )
        
        # u[t,i]: MTZ order variable
        for t in self.T:
            for i in self.V:
                self.u[t, i] = self.model.add_var(
                    name=f"u_{t}_{i}",
                    var_type=INTEGER,
                    lb=1,
                    ub=self.n
                )
        
        # T_t: time for trip t
        for t in self.T:
            self.T_t[t] = self.model.add_var(
                name=f"T_{t}",
                var_type=CONTINUOUS,
                lb=0
            )
        
        # T_total: total time
        self.T_total = self.model.add_var(
            name="T_total",
            var_type=CONTINUOUS,
            lb=0
        )
    
    def _set_objective(self):
        self.model.objective = minimize(self.T_total)
    
    def _add_constraints(self):
        # C1: Each grid point visited exactly once
        print("  C1: Coverage constraints")
        for i in self.V:
            self.model += (
                xsum(self.y[t, i] for t in self.T) == 1,
                f"cover_{i}"
            )
        
        # C2: Flow conservation (if point visited, must enter and leave)
        print("  C2: Flow conservation")
        for t in self.T:
            for i in self.V:
                # Outflow
                self.model += (
                    xsum(self.x[t, i, j] for j in self.N if (i, j) in self.A) == self.y[t, i],
                    f"outflow_{t}_{i}"
                )
                # Inflow
                self.model += (
                    xsum(self.x[t, j, i] for j in self.N if (j, i) in self.A) == self.y[t, i],
                    f"inflow_{t}_{i}"
                )
        
        # C3: Each trip starts and ends at base
        print("  C3: Base departure/return")
        for t in self.T:
            # Leave base if trip is used
            self.model += (
                xsum(self.x[t, 0, j] for j in self.V if (0, j) in self.A) >= 
                xsum(self.y[t, i] for i in self.V) / self.n,
                f"start_{t}"
            )
            # Return to base if trip is used
            self.model += (
                xsum(self.x[t, i, 0] for i in self.V if (i, 0) in self.A) >= 
                xsum(self.y[t, i] for i in self.V) / self.n,
                f"return_{t}"
            )
        
        # C4: MTZ subtour elimination
        print("  C4: MTZ subtour elimination")
        for t in self.T:
            for i in self.V:
                for j in self.V:
                    if i != j and (i, j) in self.A:
                        self.model += (
                            self.u[t, i] - self.u[t, j] + 1 <= 
                            self.n * (1 - self.x[t, i, j]),
                            f"mtz_{t}_{i}_{j}"
                        )
        
        # C5: Battery constraint per trip
        print("  C5: Battery constraints")
        for t in self.T:
            self.model += (
                xsum(self.energy_matrix[i][j] * self.x[t, i, j]
                     for (i, j) in self.A) <= self.battery_capacity,
                f"battery_{t}"
            )
        
        # C6: Trip time calculation
        print("  C6: Trip time calculation")
        for t in self.T:
            self.model += (
                self.T_t[t] == xsum(self.time_matrix[i][j] * self.x[t, i, j]
                                   for (i, j) in self.A),
                f"time_{t}"
            )
        
        # C7: Total time calculation
        print("  C7: Total time calculation")
        self.model += (
            self.T_total == xsum(self.T_t[t] for t in self.T),
            "total_time"
        )
        
        # C8: Trip ordering (break symmetry)
        print("  C8: Symmetry breaking")
        for t in range(1, self.max_trips):
            self.model += (
                xsum(self.y[t+1, i] for i in self.V) <= 
                xsum(self.y[t, i] for i in self.V),
                f"order_{t}"
            )
    
    def solve(self, heuristic_solution: Optional[dict] = None) -> dict:
        if self.model is None:
            raise ValueError("Model not built. Call build_model() first.")
        
        # Set MIP start if provided
        if heuristic_solution is not None:
            self._set_mip_start(heuristic_solution)
        
        print("\n" + "="*60)
        print("Starting optimization...")
        print("="*60)
        
        status = self.model.optimize()
        
        print("="*60)
        
        if status in [OptimizationStatus.OPTIMAL, OptimizationStatus.FEASIBLE]:
            return self._extract_solution(status)
        else:
            print(f"\n❌ No solution found. Status: {status}")
            return {
                'trips': None,
                'total_time': None,
                'status': status
            }
    
    def _set_mip_start(self, heuristic_solution: dict):
        print("\nSkipping MIP start (not supported by python-mip)")
        # MIP start not supported in python-mip
        # The solver will start from scratch
        pass
    
    def _extract_solution(self, status) -> dict:
        print(f"\n✓ Solution found! Status: {status}")
        print(f"Objective value (total time): {self.model.objective_value:.2f} seconds")
        print()
        
        trips = []
        
        for t in self.T:
            # Check if trip is used
            points_in_trip = [i for i in self.V if self.y[t, i].x > 0.5]
            
            if not points_in_trip:
                continue  # Empty trip
            
            # Reconstruct route
            route = [0]  # Start at base
            current = 0
            visited = set([0])
            
            while True:
                next_node = None
                for j in self.N:
                    if (t, current, j) in self.x and self.x[t, current, j].x > 0.5:
                        next_node = j
                        break
                
                if next_node is None or next_node == 0:
                    route.append(0)
                    break
                
                if next_node in visited and next_node != 0:
                    break
                
                route.append(next_node)
                visited.add(next_node)
                current = next_node
                
                if len(route) > len(self.N) + 5:
                    break
            
            trips.append(route)
        
        return {
            'trips': trips,
            'total_time': self.model.objective_value,
            'status': status,
            'gap': self.model.gap if hasattr(self.model, 'gap') else None
        }


def solve_multi_trip(nodes: List[Tuple],
                     connection_matrix: np.ndarray,
                     battery_capacity: float,
                     building_name: str,
                     heuristic_solution: Optional[dict] = None,
                     time_limit: int = 7200) -> dict:
    # Build energy and time matrices
    energy_matrix = EnergyModel.build_energy_matrix(nodes)
    time_matrix = EnergyModel.build_time_matrix(nodes)
    
    # Determine max_trips based on problem size and heuristic
    if heuristic_solution is not None:
        max_trips = min(heuristic_solution['num_trips'] + 2, 30)
    else:
        max_trips = min(len(nodes) // 5, 30)  # More generous: n/5 instead of n/10
    
    # Build and solve model
    model = MultiTripModel(
        nodes=nodes,
        connection_matrix=connection_matrix,
        energy_matrix=energy_matrix,
        time_matrix=time_matrix,
        battery_capacity=battery_capacity,
        max_trips=max_trips,
        time_limit=time_limit
    )
    
    model.build_model()
    solution = model.solve(heuristic_solution=heuristic_solution)
    
    return solution

