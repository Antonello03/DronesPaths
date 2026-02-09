"""
Entry-seeded greedy heuristic for multi-trip routing under battery constraints.
"""

import heapq
import time as _time
import numpy as np
from typing import Dict, List, Optional, Tuple, Set
from src.energy_model import EnergyModel


class EntrySeedGreedyTripBuilder:

     
    """
    Greedy multi-trip heuristic: start from a reachable entry node, then keep adding feasible nodes using
    shortest-energy paths, until battery forces us to go back to base.
    """


    def __init__(self,
                 nodes: List[Tuple],
                 connection_matrix: np.ndarray,
                 battery_capacity: float):
        self.nodes = nodes
        self.connection_matrix = connection_matrix
        self.num_cands = 10  # max candidates to evaluate per step
        self.battery_capacity = battery_capacity
        self.n = len(nodes)

        # base coordinates
        self.base_xyz = (nodes[0][1], nodes[0][2], nodes[0][3])
        # entry nodes = quelli direttamente collegati al base (regola problema)
        self.entry_nodes = {j for j in range(1, self.n) if self.connection_matrix[0][j]}
        self.energy_matrix = EnergyModel.build_energy_matrix(nodes)
        self._entry_path_cache: Dict[int, Optional[List[int]]] = {}
        self._path_cache: Dict[Tuple[int, int], Optional[List[int]]] = {}

    def build_solution(self) -> List[List[int]]:
        start_time = _time.perf_counter()
        trips = []
        unvisited: Set[int] = set(range(1, self.n))
        trip_id = 1

        print("\n" + "="*60)
        print("Improved Heuristic (Entry-seeded greedy)")
        print("="*60)

        print(">>> entering build_solution")
        print("nodes:", self.n)
        print("unvisited initial:", len(set(range(1, self.n))))


        while unvisited:
            elapsed = _time.perf_counter() - start_time
            covered = self.n - 1 - len(unvisited)

            print(
                f"[{elapsed:6.1f}s] Trip {trip_id} | "
                f"covered {covered}/{self.n-1} | remaining {len(unvisited)}"
            )

            trip = None
            newly_visited: Set[int] = set()
            # Try multiple entry seeds and keep the best trip that adds new coverage.
            for seed in self._candidate_entry_seeds(unvisited):
                candidate = self._build_single_trip(seed, unvisited)
                if candidate is None:
                    continue
                candidate_new = set(candidate[1:-1]) & unvisited
                if len(candidate_new) > len(newly_visited):
                    trip = candidate
                    newly_visited = candidate_new
                # cannot do better than covering all remaining nodes
                if len(newly_visited) == len(unvisited):
                    break

            if trip is None:
                print(f"\nCannot reach remaining {len(unvisited)} points")
                break

            unvisited -= newly_visited
            trips.append(trip)

            energy = EnergyModel.calculate_route_energy(trip, self.nodes)
            time = EnergyModel.calculate_route_time(trip, self.nodes)

            print(f"  Trip {trip_id}: {len(newly_visited)} pts, "
                  f"{energy:.0f}/{self.battery_capacity:.0f} J, {time:.1f}s")

            trip_id += 1

        return trips


    def _candidate_entry_seeds(self, unvisited: Set[int]) -> List[int]:
        "Ordered candidate entry seeds: unvisited entries first, then other entries."
        def sort_key(i):
            p = self.nodes[i]
            return np.linalg.norm([
                p[1] - self.base_xyz[0],
                p[2] - self.base_xyz[1],
                p[3] - self.base_xyz[2]
            ])

        primary = list(self.entry_nodes & unvisited)
        secondary = list(self.entry_nodes - unvisited)
        primary.sort(key=sort_key, reverse=True)
        secondary.sort(key=sort_key, reverse=True)
        return primary + secondary

    def _build_single_trip(self, seed: int, unvisited: Set[int]) -> Optional[List[int]]:
        if seed is None:
            return None
        # require seed be entry so base->seed is allowed
        if seed not in self.entry_nodes:
            return None

        trip = [0, seed]

        remaining = set(unvisited)
        remaining.discard(seed)
        current = seed

        while remaining:

            segment = self._best_neighbor(trip, current, remaining)
            if segment is None:
                break
            trip.extend(segment)
            for node in segment:
                remaining.discard(node)
            current = trip[-1]

        exit_path = self._path_to_entry(current)
        if exit_path is None:
            return None
        trip.extend(exit_path[1:])  # include path to entry
        # Closing step: we return to base from an entry node.
        # Assumes connection_matrix allows (entry -> 0) (e.g., symmetric graph).
        trip.append(0)


        if not EnergyModel.is_trip_feasible(trip, self.nodes, self.battery_capacity):
            return None
        return trip

    def _path_to_entry(self, start: int) -> Optional[List[int]]:
        """
        Minimum-energy path from start to any entry node (nodes 1..n-1 only).
        Returns inclusive node path [start, ..., entry].
        """
        if start in self._entry_path_cache:
            cached = self._entry_path_cache[start]
            if cached is None:
                return None
            return list(cached)

        if start in self.entry_nodes:
            self._entry_path_cache[start] = [start]
            return [start]

        dist = [float("inf")] * self.n
        prev = [-1] * self.n
        dist[start] = 0.0
        pq = [(0.0, start)]
        target = -1

        while pq:
            d, u = heapq.heappop(pq)
            if d > dist[u]:
                continue
            if u in self.entry_nodes:
                target = u
                break
            for v in range(1, self.n):
                if not self.connection_matrix[u][v]:
                    continue
                nd = d + self.energy_matrix[u][v]
                if nd < dist[v]:
                    dist[v] = nd
                    prev[v] = u
                    heapq.heappush(pq, (nd, v))

        if target == -1:
            self._entry_path_cache[start] = None
            return None

        path = [target]
        cur = target
        while prev[cur] != -1:
            cur = prev[cur]
            path.append(cur)
        path.reverse()
        self._entry_path_cache[start] = path
        return list(path)

    def _path_between(self, start: int, target: int) -> Optional[List[int]]:
        "Minimum-energy path between two non-base nodes."
        key = (start, target)
        if key in self._path_cache:
            cached = self._path_cache[key]
            return None if cached is None else list(cached)
        if start == target:
            self._path_cache[key] = [start]
            return [start]

        dist = [float("inf")] * self.n
        prev = [-1] * self.n
        dist[start] = 0.0
        pq = [(0.0, start)]

        while pq:
            d, u = heapq.heappop(pq)
            if d > dist[u]:
                continue
            if u == target:
                break
            for v in range(1, self.n):
                if not self.connection_matrix[u][v]:
                    continue
                nd = d + self.energy_matrix[u][v]
                if nd < dist[v]:
                    dist[v] = nd
                    prev[v] = u
                    heapq.heappush(pq, (nd, v))

        if dist[target] == float("inf"):
            self._path_cache[key] = None
            return None

        path = [target]
        cur = target
        while prev[cur] != -1:
            cur = prev[cur]
            path.append(cur)
        path.reverse()
        self._path_cache[key] = path
        return list(path)

    def _best_neighbor(self,
                       current_trip: List[int],
                       current: int,
                       candidates: Set[int]) -> Optional[List[int]]:
        best_segment = None
        best_cost = float("inf")
        cands = sorted(
            candidates,
            key=lambda i: self.energy_matrix[current][i]
        )
        for c in cands[:self.num_cands]:  # limit number of candidates to evaluate

            path_to_c = self._path_between(current, c)
            if path_to_c is None:
                continue

            exit_path = self._path_to_entry(c)
            if exit_path is None:
                continue

            # segment excludes current and includes candidate (+ transit nodes)
            segment = path_to_c[1:]
            test_trip = current_trip + segment + exit_path[1:] + [0]
            if not EnergyModel.is_trip_feasible(test_trip, self.nodes, self.battery_capacity):
                continue

            # Prefer low added energy, and reward segments covering many still-unvisited nodes.
            seg_cost = 0.0
            for i in range(len(path_to_c) - 1):
                seg_cost += self.energy_matrix[path_to_c[i]][path_to_c[i + 1]]
            covered_in_segment = len(set(segment) & candidates)
            score = seg_cost / max(1, covered_in_segment)

            if score < best_cost:
                best_cost = score
                best_segment = segment

        return best_segment


def build_heuristic_solution(nodes: List[Tuple],
                             connection_matrix: np.ndarray,
                             battery_capacity: float) -> dict:

    builder = EntrySeedGreedyTripBuilder(nodes, connection_matrix, battery_capacity)
    trips = builder.build_solution()

    total_time = sum(EnergyModel.calculate_route_time(t, nodes) for t in trips)
    total_energy = sum(EnergyModel.calculate_route_energy(t, nodes) for t in trips)

    visited = set()
    for t in trips:
        visited.update(t[1:-1])

    print("\nHeuristic complete")
    print(f"  Trips: {len(trips)}")
    print(f"  Covered: {len(visited)}/{len(nodes)-1}")

    return {
        "trips": trips,
        "total_time": total_time,
        "total_energy": total_energy,
        "num_trips": len(trips)
    }
