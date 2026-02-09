"""
Goal: cover all points with multiple trips (each trip starts/ends at base 0)
under battery capacity per trip, minimizing total time.

Output expected:
Trip i: 0-...-0
"""

from mip import (
    Model,
    xsum,
    minimize,
    BINARY,
    INTEGER,
    CONTINUOUS,
    SearchEmphasis,
)
import numpy as np
from typing import List, Tuple, Optional, Dict
from src.energy_model import EnergyModel


class MultiTripModel:
    def __init__(
        self,
        nodes: List[Tuple],
        connection_matrix: np.ndarray,
        energy_matrix: np.ndarray,
        time_matrix: np.ndarray,
        battery_capacity: float,
        max_trips: int = 20,
        time_limit: Optional[int] = 7200,
        max_mip_gap: Optional[float] = 0.11,
        stall_seconds: Optional[int] = None,
        threads: int = 8,
        pump_passes: int = 300,
        trip_upper_bound: Optional[int] = None,
        heuristic_time_upper_bound: Optional[float] = None,
        verbose: int = 1
    ):
        self.nodes = nodes
        self.connection_matrix = connection_matrix
        self.energy_matrix = energy_matrix
        self.time_matrix = time_matrix
        self.battery_capacity = battery_capacity
        self.max_trips = max_trips
        self.time_limit = time_limit
        self.max_mip_gap = max_mip_gap
        self.stall_seconds = stall_seconds
        self.threads = threads
        self.pump_passes = pump_passes
        self.trip_upper_bound = trip_upper_bound
        self.heuristic_time_upper_bound = heuristic_time_upper_bound
        self.verbose = verbose

        self.n = len(nodes) - 1           # points excluding base
        self.N = list(range(len(nodes)))  # including base 0
        self.V = list(range(1, len(nodes)))
        self.T = list(range(1, max_trips + 1))

        # arc set
        self.A = [(i, j) for i in self.N for j in self.N
                  if i != j and bool(self.connection_matrix[i][j])]

        self.model: Optional[Model] = None

        # vars
        self.x = {}      # x[t,i,j]
        self.y = {}      # y[t,i]
        self.f = {}      # connectivity flow from base to visited nodes
        self.z = {}      # trip active
        self.T_t = {}
        self.T_total = None

    def build_model(self):
        self.model = Model("MultiTripDroneRouting")
        self.model.verbose = self.verbose

        # Hard-set a wall-clock limit; passing it again in optimize() guarantees CBC respects it.
        if self.time_limit is not None:
            self.model.max_seconds = self.time_limit
        # Optional quality stop; disabled by default.
        if self.max_mip_gap is not None:
            self.model.max_mip_gap = self.max_mip_gap
        # CBC tuning for finding feasible solutions faster.
        self.model.emphasis = SearchEmphasis.FEASIBILITY
        self.model.threads = self.threads
        self.model.pump_passes = self.pump_passes

        self._create_variables()
        self._set_objective()
        self._add_constraints()

        print(f"Model built with {self.model.num_cols} variables and {self.model.num_rows} constraints")

    def apply_heuristic_start(self, trips: List[List[int]]):
        """
        Use heuristic trips as a warm-start to speed up CBC
    
        """
        if self.model is None or trips is None or len(trips) == 0:
            return

        start_map = {}
        used_trips = 0

        for t_idx, trip in enumerate(trips, start=1):
            if t_idx > self.max_trips:
                break
            if len(trip) < 2:
                continue
            if trip[0] != 0 or trip[-1] != 0:
                continue

            local_map = {}
            local_map[self.z[t_idx]] = 1.0
            feasible_arc = True
            visited_nodes = set()
            for i in range(len(trip) - 1):
                u, v = trip[i], trip[i + 1]
                if (u, v) not in self.A:
                    feasible_arc = False
                    break
                x_var = self.x[t_idx, u, v]
                local_map[x_var] = local_map.get(x_var, 0.0) + 1.0
                if v != 0 and v in self.V:
                    visited_nodes.add(v)
            if (not feasible_arc) or (not visited_nodes):
                continue

            for v in visited_nodes:
                local_map[self.y[t_idx, v]] = 1.0

            time_val = 0.0
            for (i, j) in self.A:
                x_var = self.x[t_idx, i, j]
                count = local_map.get(x_var, 0.0)
                if count > 1e-9:
                    time_val += count * self.time_matrix[i][j]
            if time_val > 0:
                local_map[self.T_t[t_idx]] = float(time_val)

            used_trips += 1
            for var, value in local_map.items():
                start_map[var] = value

        total_time = sum(start_map.get(self.T_t[t], 0.0) for t in self.T)
        if total_time > 0:
            start_map[self.T_total] = float(total_time)

        if start_map:
            self.model.start = [(var, val) for var, val in start_map.items()]
            if self.verbose:
                print(f"Warm-start applied: {len(start_map)} variable hints on {used_trips} trips")

    def _create_variables(self):
        # arc usage
        for t in self.T:
            for (i, j) in self.A:
                self.x[t, i, j] = self.model.add_var(
                    var_type=INTEGER,
                    lb=0,
                    ub=self.n,
                    name=f"x_{t}_{i}_{j}"
                )

        # node visited in trip
        for t in self.T:
            for i in self.V:
                self.y[t, i] = self.model.add_var(var_type=BINARY, name=f"y_{t}_{i}")

        # flow variables for connectivity 
        for t in self.T:
            for (i, j) in self.A:
                self.f[t, i, j] = self.model.add_var(
                    var_type=CONTINUOUS,
                    lb=0,
                    ub=self.n,
                    name=f"f_{t}_{i}_{j}"
                )

        # trip active
        for t in self.T:
            self.z[t] = self.model.add_var(var_type=BINARY, name=f"z_{t}")

        # trip time and total time
        for t in self.T:
            self.T_t[t] = self.model.add_var(var_type=CONTINUOUS, lb=0, name=f"T_{t}")
        self.T_total = self.model.add_var(var_type=CONTINUOUS, lb=0, name="T_total")

    def _set_objective(self):
        # project says: cover all points + minimize total time
        # We enforce full coverage as constraints, so objective = minimize time.
        self.model.objective = minimize(self.T_total)

    def _add_constraints(self):
        # C0: every node must be covered at least once (project requirement).
        for i in self.V:
            self.model += (xsum(self.y[t, i] for t in self.T) >= 1, f"cover_{i}")

        # link z[t] to existence of visits
        for t in self.T:
            self.model += (xsum(self.y[t, i] for i in self.V) <= self.n * self.z[t], f"z_up_{t}")
            self.model += (xsum(self.y[t, i] for i in self.V) >= self.z[t], f"z_low_{t}")
            for i in self.V:
                self.model += (self.y[t, i] <= self.z[t], f"y_link_{t}_{i}")

        # C1: per-trip in/out balance; nodes can be revisited (in/out >= 1 if visited)
        for t in self.T:
            for i in self.V:
                out_i = xsum(self.x[t, i, j] for j in self.N if (i, j) in self.A)
                in_i = xsum(self.x[t, j, i] for j in self.N if (j, i) in self.A)
                self.model += (
                    out_i == in_i,
                    f"bal_{t}_{i}"
                )
                self.model += (
                    in_i >= self.y[t, i],
                    f"in_lb_{t}_{i}"
                )
                self.model += (
                    in_i <= self.n * self.y[t, i],
                    f"in_ub_{t}_{i}"
                )

        # C2: base start/end exactly 1 if trip active, else 0
        for t in self.T:
            self.model += (
                xsum(self.x[t, 0, j] for j in self.V if (0, j) in self.A) == self.z[t],
                f"start_{t}"
            )
            self.model += (
                xsum(self.x[t, i, 0] for i in self.V if (i, 0) in self.A) == self.z[t],
                f"end_{t}"
            )
            # Valid inequality: each visited node needs at least one incoming arc,
            # and active trips need one return-to-base arc.
            self.model += (
                xsum(self.x[t, i, j] for (i, j) in self.A)
                >= xsum(self.y[t, i] for i in self.V) + self.z[t],
                f"arc_lb_{t}"
            )

        # C3: connectivity via single-commodity flow
        # If y[t,i]=1, one unit of flow must reach node i from base.
        # Enforces that all visited nodes in a trip are connected to the base.
        # Compared to MTZ subtour-elimination, this avoids ordering variables and big-M constants and
        # provides a stronger LP relaxation,  which scales better on larger instances:
        # disconnected subtours are penalized much earlier in the search, resulting in faster
        #  pruning and a smaller branch-and-bound tree.

        for t in self.T:
            for i in self.V:
                self.model += (
                    xsum(self.f[t, j, i] for j in self.N if (j, i) in self.A)
                    - xsum(self.f[t, i, j] for j in self.N if (i, j) in self.A)
                    == self.y[t, i],
                    f"flow_node_{t}_{i}"
                )
            self.model += (
                xsum(self.f[t, 0, j] for j in self.V if (0, j) in self.A)
                - xsum(self.f[t, j, 0] for j in self.V if (j, 0) in self.A)
                == xsum(self.y[t, i] for i in self.V),
                f"flow_base_{t}"
            )
            for (i, j) in self.A:
                self.model += (
                    self.f[t, i, j] <= self.n * self.x[t, i, j],
                    f"flow_cap_{t}_{i}_{j}"
                )

        # C4: battery per trip
        for t in self.T:
            self.model += (
                xsum(self.energy_matrix[i][j] * self.x[t, i, j] for (i, j) in self.A) <= self.battery_capacity,
                f"battery_{t}"
            )

        # C5: trip time + total time
        for t in self.T:
            self.model += (
                self.T_t[t] == xsum(self.time_matrix[i][j] * self.x[t, i, j] for (i, j) in self.A),
                f"time_{t}"
            )
        self.model += (self.T_total == xsum(self.T_t[t] for t in self.T), "total_time")
        if self.trip_upper_bound is not None:
            self.model += (
                xsum(self.z[t] for t in self.T) <= self.trip_upper_bound,
                "trip_upper_bound"
            )
        if self.heuristic_time_upper_bound is not None:
            self.model += (
                self.T_total <= self.heuristic_time_upper_bound,
                "heuristic_time_upper_bound"
            )

        # Symmetry breaker: use trips in order (no gaps)
        for t in range(2, self.max_trips + 1):
            self.model += (self.z[t] <= self.z[t - 1], f"order_{t}")
            # Stronger symmetry break: earlier trips cover at least as many nodes.
            self.model += (
                xsum(self.y[t - 1, i] for i in self.V) >= xsum(self.y[t, i] for i in self.V),
                f"load_order_{t}"
            )

    def solve(self) -> Optional[Dict]:
        if self.model is None:
            raise RuntimeError("Model not built")

        # Pass max_seconds explicitly so the solver cannot ignore the limit.
        if self.stall_seconds is not None:
            status = self.model.optimize(
                max_seconds=self.time_limit,
                max_seconds_same_incumbent=self.stall_seconds
            )
        else:
            status = self.model.optimize(max_seconds=self.time_limit)

        # MIP SUMMARY
        print("=== MIP SUMMARY ===")
        print("Status:", status)
        print("Best bound:", self.model.objective_bound)
        print("Gap:", self.model.gap)

        # If no feasible solution founded
        if self.model.num_solutions == 0:
            print("Best value (T_total): None (no feasible solution)")
            print("===================")
            print(f"No solution found. Status = {status}")
            return None

        # qui siamo sicuri che esiste almeno una soluzione
        print("Best value (T_total):", self.T_total.x)

        print(f"Solver status: {status} (solutions: {self.model.num_solutions})")

        trips = self._extract_trips()
        covered = {i for i in self.V if any(self.y[t, i].x > 0.5 for t in self.T)}

        print(f"\n✓ Covered nodes: {len(covered)} / {self.n}")
        print(f"✓ Trips used: {len(trips)}")
        print(f"✓ Total time: {self.T_total.x:.2f} s")

        return {
            "trips": trips,
            "covered": len(covered),
            "total_time": float(self.T_total.x)
        }

    def _extract_trips(self) -> List[List[int]]:
        trips = []

        for t in self.T:
            if self.z[t].x < 0.5:
                continue

            # Build directed multigraph of selected arcs for trip t.
            adj = {i: [] for i in self.N}
            edges_used = 0
            for (i, j) in self.A:
                var = self.x[t, i, j]
                copies = int(round(var.x))
                if copies > 0:
                    adj[i].extend([j] * copies)
                    edges_used += copies

            if edges_used == 0:
                continue

            # Hierholzer algorithm to extract an Eulerian circuit from base.
            stack = [0]
            circuit = []
            while stack:
                u = stack[-1]
                if adj[u]:
                    v = adj[u].pop()
                    stack.append(v)
                else:
                    circuit.append(stack.pop())

            route = list(reversed(circuit))
            if len(route) < 2:
                route = [0, 0]
            if route[0] != 0:
                route = [0] + route
            if route[-1] != 0:
                route.append(0)

            trips.append(route)

        return trips


def solve_multi_trip(
    nodes: List[Tuple],
    connection_matrix: np.ndarray,
    battery_capacity: float,
    building_name: str,
    heuristic_solution=None,
    time_limit: Optional[int] = 7200,
    # Allow more trips for large instances 
    max_trips_cap: int = 80,
    max_mip_gap: Optional[float] = 0.11,
    stall_seconds: Optional[int] = None,
    threads: int = 8,
    pump_passes: int = 300,
    verbose: int = 1
):
    energy_matrix = EnergyModel.build_energy_matrix(nodes)
    time_matrix = EnergyModel.build_time_matrix(nodes)
    n_points = len(nodes) - 1
    trip_upper_bound = None
    heuristic_time_upper_bound = None
    warm_start_trips = None

    if heuristic_solution is not None and "num_trips" in heuristic_solution:
        heur_trips = max(1, int(heuristic_solution["num_trips"]))
        heur_covered = set()
        for trip in heuristic_solution.get("trips", []):
            heur_covered.update(trip[1:-1])

        # Heuristic routes may repeat nodes. The MIP also allows revisits (x are integers),
        # but the heuristic trip count can still be too optimistic due to different modeling
        # choices and connectivity/battery constraints.

        if len(heur_covered) >= n_points:
            max_trips = min(max(5, heur_trips + 3), max_trips_cap)
        else:
            max_trips = min(max(10, heur_trips + 8), max_trips_cap)
    else:
        # Without heuristic info, pick a conservative upper bound to avoid infeasibility from too few trips
        max_trips = min(max(5, len(nodes) // 4), max_trips_cap)

    if verbose:
        print(f"MIP max_trips: {max_trips}")

    if heuristic_solution is not None and "trips" in heuristic_solution:
        trips = heuristic_solution["trips"]
        heur_trips = max(1, int(heuristic_solution.get("num_trips", len(trips) or 1)))
        heur_covered = set()
        for trip in trips:
            heur_covered.update(trip[1:-1])
        full_coverage = len(heur_covered) >= n_points

        arc_set = set((i, j) for i in range(len(nodes)) for j in range(len(nodes))
                      if i != j and bool(connection_matrix[i][j]))
        arcs_ok = True
        energy_ok = True
        for trip in trips:
            e_sum = 0.0
            for i in range(len(trip) - 1):
                u, v = trip[i], trip[i + 1]
                if (u, v) not in arc_set:
                    arcs_ok = False
                    break
                e_sum += energy_matrix[u][v]
            if (not arcs_ok) or e_sum > battery_capacity + 1e-6:
                energy_ok = False
                break

        if full_coverage and arcs_ok and energy_ok:
            # Keep one extra trip for safety while still reducing symmetry and branching.
            trip_upper_bound = min(max_trips, heur_trips + 1)
            heur_total_time = heuristic_solution.get("total_time")
            if heur_total_time is not None:
                heuristic_time_upper_bound = float(heur_total_time) + 1e-6
            warm_start_trips = trips
            if verbose:
                print(f"Applying trip bound: sum(z) <= {trip_upper_bound}")
                if heuristic_time_upper_bound is not None:
                    print(f"Applying time upper bound: T_total <= {heuristic_time_upper_bound:.3f}")
        elif verbose:
            print("Skipping heuristic warm start: heuristic not fully model-feasible")

    model = MultiTripModel(
        nodes=nodes,
        connection_matrix=connection_matrix,
        energy_matrix=energy_matrix,
        time_matrix=time_matrix,
        battery_capacity=battery_capacity,
        max_trips=max_trips,
        time_limit=time_limit,
        max_mip_gap=max_mip_gap,
        stall_seconds=stall_seconds,
        threads=threads,
        pump_passes=pump_passes,
        trip_upper_bound=trip_upper_bound,
        heuristic_time_upper_bound=heuristic_time_upper_bound,
        verbose=verbose
    )
    model.build_model()
    if warm_start_trips is not None:
        model.apply_heuristic_start(warm_start_trips)
    return model.solve()
