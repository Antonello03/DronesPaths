"""
MIP Model for Multi-Drone Routing Problem
Person 3 implementation based on Person 2's mathematical formulation
"""

from mip import Model, xsum, minimize, BINARY, INTEGER, CONTINUOUS, OptimizationStatus
import numpy as np


def solve_instance(nodes, connection_matrix, distance_matrix, k_drones=4, time_limit=300):
    """
    Solves the multi-drone routing problem using MIP.
    
    Parameters:
    -----------
    nodes : list of tuples (index, x, y, z)
        List of all nodes including base (index 0) and grid points (1...n)
    connection_matrix : np.ndarray
        Boolean matrix where [i][j] = True if arc (i,j) exists
    distance_matrix : np.ndarray  
        Travel time matrix where [i][j] = time from node i to j
    k_drones : int
        Number of drones (default 4)
    time_limit : int
        Maximum solver time in seconds
        
    Returns:
    --------
    dict with keys:
        'routes': list of routes, one per drone (each route is list of node indices)
        'route_times': list of total time for each drone
        'makespan': maximum route time (objective value)
        'status': optimization status
    """
    
    print("=" * 60)
    print("Building MIP Model for Multi-Drone Routing Problem")
    print("=" * 60)
    
    # Extract problem data
    n = len(nodes) - 1  # Number of grid points (excluding base)
    N = list(range(len(nodes)))  # All nodes: [0, 1, 2, ..., n]
    V = list(range(1, len(nodes)))  # Grid points only: [1, 2, ..., n]
    K = list(range(1, k_drones + 1))  # Drones: [1, 2, 3, 4]
    
    # Build arc set A from connection matrix (as set for fast lookup)
    A = set()
    for i in N:
        for j in N:
            if i != j and connection_matrix[i][j]:
                A.add((i, j))
    
    print(f"Problem size:")
    print(f"  - Nodes: {len(N)} (base + {n} grid points)")
    print(f"  - Drones: {k_drones}")
    print(f"  - Feasible arcs: {len(A)}")
    print()
    
    # Create model
    m = Model("MultiDroneRouting")
    m.verbose = 1
    m.max_seconds = time_limit
    
    print("Creating decision variables...")
    
    # ========== DECISION VARIABLES ==========
    
    # Routing variables: x[d,i,j] = 1 if drone d travels from i to j
    x = {}
    for d in K:
        for (i, j) in A:
            x[d, i, j] = m.add_var(name=f"x_{d}_{i}_{j}", var_type=BINARY)
    
    # MTZ order variables per drone: u[d,i] = position of node i in drone d tour
    u = {}
    for d in K:
        for i in V:
            u[d, i] = m.add_var(name=f"u_{d}_{i}", var_type=INTEGER, lb=1, ub=n)
    
    # Time variables
    T_d = {}  # T_d[d] = total route time for drone d
    for d in K:
        T_d[d] = m.add_var(name=f"T_{d}", var_type=CONTINUOUS, lb=0)
    
    T = m.add_var(name="T", var_type=CONTINUOUS, lb=0)  # Makespan
    
    print(f"  - Routing variables (x_dij): {len(x)}")
    print(f"  - MTZ order variables (u_i): {len(u)}")
    print(f"  - Time variables: {len(T_d)} + 1 (makespan)")
    print()
    
    # OBJECTIVE FUNCTION 
    print("Setting objective: minimize makespan T")
    m.objective = minimize(T)
    
    # CONSTRAINTS 
    print("Adding constraints...")
    
    # C1. Each grid point is visited exactly once
    print("  C1: Each grid point visited exactly once")
    c1_count = 0
    for i in V:
        m += xsum(x[d, j, i] for d in K for j in N if (j, i) in A) == 1, f"visit_{i}"
        c1_count += 1
    print(f"     Added {c1_count} C1 constraints")
    
    # C2. Flow conservation at each grid point (per drone)
    print("  C2: Flow conservation at each grid point")
    c2_count = 0
    for d in K:
        for i in V:
            outgoing = [j for j in N if (i, j) in A]
            incoming = [j for j in N if (j, i) in A]
            # Add flow conservation: what comes in must go out
            m += (xsum(x[d, i, j] for j in outgoing) - 
                  xsum(x[d, j, i] for j in incoming) == 0), f"flow_{d}_{i}"
            c2_count += 1
    print(f"     Added {c2_count} C2 constraints")
    
    # C3. Each drone departs from base exactly once
    print("  C3: Each drone departs from base exactly once")
    c3_count = 0
    for d in K:
        outgoing_from_base = [j for j in N if (0, j) in A]
        print(f"     Drone {d}: base can reach {len(outgoing_from_base)} entry points")
        m += xsum(x[d, 0, j] for j in outgoing_from_base) == 1, f"depart_{d}"
        c3_count += 1
    print(f"     Added {c3_count} C3 constraints")
    
    # C4. Each drone returns to base exactly once
    print("  C4: Each drone returns to base exactly once")
    c4_count = 0
    for d in K:
        incoming_to_base = [j for j in N if (j, 0) in A]
        m += xsum(x[d, j, 0] for j in incoming_to_base) == 1, f"return_{d}"
        c4_count += 1
    print(f"     Added {c4_count} C4 constraints")
    
    # C5. Route time definition for each drone
    print("  C5: Route time definition")
    for d in K:
        m += (T_d[d] == xsum(distance_matrix[i][j] * x[d, i, j] 
                             for (i, j) in A)), f"route_time_{d}"
    print(f"     Added {len(K)} C5 constraints")
    
    # C6. Makespan definition
    print("  C6: Makespan definition")
    for d in K:
        m += T >= T_d[d], f"makespan_{d}"
    print(f"     Added {len(K)} C6 constraints")
    
    # C7. MTZ subtour elimination constraints 
    # we prevent subtours with the constraint instead of the iterative approach
    print("  C7: MTZ subtour elimination")
    mtz_count = 0
    for d in K:
        for i in V:
            for j in V:
                if i != j and (i, j) in A:
                    # u_di - u_dj + 1 <= (n-1)(1 - x_dij)
                    m += (u[d, i] - u[d, j] + 1 <= (n - 1) * (1 - x[d, i, j])), \
                         f"mtz_{d}_{i}_{j}"
                    mtz_count += 1
    print(f"     Added {mtz_count} MTZ constraints")
    
    print()
    print("Model built successfully!")
    print(f"Total variables: {m.num_cols}")
    print(f"Total constraints: {m.num_rows}")
    print("=" * 60)
    print()
    
    # ========== SOLVE ==========
    print("Starting optimization...")
    print("=" * 60)
    status = m.optimize()
    print("=" * 60)
    
    # ========== EXTRACT SOLUTION ==========
    if status in [OptimizationStatus.OPTIMAL, OptimizationStatus.FEASIBLE]:
        print(f"\nSolution found! Status: {status}")
        print(f"Objective value (makespan): {m.objective_value:.2f} seconds")
        print()
        
        # Extract routes for each drone
        routes = []
        route_times = []
        
        for d in K:
            route = [0]  # Start from base
            current = 0
            
            # Follow the path by finding x[d, current, j] = 1
            visited = set([0])
            while True:
                next_node = None
                for j in N:
                    if (current, j) in A and (d, current, j) in x:
                        if x[d, current, j].x > 0.5:  # Binary variable is 1
                            next_node = j
                            break
                
                if next_node is None or next_node == 0:
                    route.append(0)  # Return to base
                    break
                
                if next_node in visited and next_node != 0:
                    print(f"Warning: Cycle detected in drone {d} route at node {next_node}")
                    break
                    
                route.append(next_node)
                visited.add(next_node)
                current = next_node
                
                if len(route) > len(N) + 5:  # Safety check
                    print(f"Warning: Route too long for drone {d}, stopping")
                    break
            
            routes.append(route)
            route_times.append(T_d[d].x if d in T_d else 0)
        
        result = {
            'routes': routes,
            'route_times': route_times,
            'makespan': m.objective_value,
            'status': status,
            'gap': m.gap if hasattr(m, 'gap') else None
        }
        
        return result
    
    else:
        print(f"\nNo solution found. Status: {status}")
        return {
            'routes': None,
            'route_times': None,
            'makespan': None,
            'status': status
        }


def format_solution_output(result, drone_offset=1):
    """
    Format the solution for output according to requirements.
    
    Output format:
    Trip 1: 0-4-11-17-...-2-0
    Trip 2: 0-5-6-3-...-7-0
    etc.
    
    Parameters:
    -----------
    result : dict
        Result from solve_instance()
    drone_offset : int
        Starting index for drone numbering (default 1)
        
    Returns:
    --------
    str : Formatted solution string
    """
    if result['routes'] is None:
        return "No solution found."
    
    lines = []
    for i, route in enumerate(result['routes']):
        trip_num = i + drone_offset
        route_str = "-".join(map(str, route))
        lines.append(f"Trip {trip_num}: {route_str}")
    
    return "\n".join(lines)


def print_solution_summary(result):
    """
    Print a detailed summary of the solution.
    """
    if result['routes'] is None:
        print("\nNo solution found")
        return
    
    print("\n" + "=" * 60)
    print("SOLUTION SUMMARY")
    print("=" * 60)
    
    for i, (route, time) in enumerate(zip(result['routes'], result['route_times'])):
        drone_num = i + 1
        points_visited = len([p for p in route if p != 0])
        print(f"Drone {drone_num}:")
        print(f"  Route: {' → '.join(map(str, route))}")
        print(f"  Points visited: {points_visited}")
        print(f"  Total time: {time:.2f} seconds")
        print()
    
    print(f"Makespan (objective): {result['makespan']:.2f} seconds")
    if result['gap'] is not None:
        print(f"Optimality gap: {result['gap']*100:.2f}%")
    print("=" * 60)
