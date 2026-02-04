"""
Big Project Main Entry Point

Solves single-drone multi-trip routing with battery constraints.
Orchestrates: base selection, heuristic, and MIP optimization.
"""

import sys
import os
sys.path.append(os.path.dirname(__file__))

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

from src.utils import loadBuildingDots
from src.base_selection import select_base_for_building
from src.energy_model import EnergyModel
from src.heuristic import build_heuristic_solution
from src.multi_trip_model import solve_multi_trip


def build_matrices_with_base(nodes_with_base, building_name):
    n = len(nodes_with_base)
    connection_matrix = np.zeros((n, n), dtype=bool)
    distance_matrix = np.zeros((n, n))
    
    # Determine entry points based on building
    y_threshold = -12.5 if "edificio1" in building_name else -20.0
    entry_points = set()
    for idx, node in enumerate(nodes_with_base):
        if idx > 0 and node[2] <= y_threshold:  # y coordinate <= threshold
            entry_points.add(idx)
    
    print(f"  Entry points: {len(entry_points)} points with y <= {y_threshold}")
    
    for i in range(n):
        for j in range(n):
            if i != j:
                node_i = nodes_with_base[i]
                node_j = nodes_with_base[j]
                
                # Special case: base (index 0) can connect to entry points regardless of distance
                if (i == 0 and j in entry_points) or (j == 0 and i in entry_points):
                    connected = True
                else:
                    # Check normal connectivity rules
                    connected = check_connectivity(node_i, node_j)
                
                connection_matrix[i][j] = connected
                
                if connected:
                    # Calculate time (used as distance in small project)
                    point_i = (node_i[1], node_i[2], node_i[3])
                    point_j = (node_j[1], node_j[2], node_j[3])
                    distance_matrix[i][j] = EnergyModel.calculate_time(point_i, point_j)
    
    return connection_matrix, distance_matrix


def check_connectivity(node1, node2):
    x1, y1, z1 = node1[1], node1[2], node1[3]
    x2, y2, z2 = node2[1], node2[2], node2[3]
    
    # Calculate differences
    dx = abs(x2 - x1)
    dy = abs(y2 - y1)
    dz = abs(z2 - z1)
    
    euclidean = np.sqrt(dx**2 + dy**2 + dz**2)
    
    # Rule 1: distance <= 4m
    if euclidean <= 4.0:
        return True
    
    # Rule 2: distance <= 11m AND two coordinates differ by <= 0.5m
    if euclidean <= 11.0:
        diffs = sorted([dx, dy, dz])
        if diffs[0] <= 0.5 and diffs[1] <= 0.5:
            return True
    
    return False


def format_output(solution, nodes_with_base):
    if solution['trips'] is None:
        return "No solution found."
    
    lines = []
    for i, trip in enumerate(solution['trips']):
        viaggio_num = i + 1
        trip_str = "-".join(map(str, trip))
        lines.append(f"Viaggio {viaggio_num}: {trip_str}")
    
    return "\n".join(lines)


def visualize_solution(solution, nodes_with_base, building_name):
    if solution['trips'] is None:
        print("No solution to visualize")
        return
    
    fig = plt.figure(figsize=(14, 10))
    ax = fig.add_subplot(111, projection='3d')
    
    # Extract coordinates
    coords = {node[0]: (node[1], node[2], node[3]) for node in nodes_with_base}
    
    # Plot grid points
    grid_points = [coords[i] for i in range(1, len(nodes_with_base))]
    if grid_points:
        xs, ys, zs = zip(*grid_points)
        ax.scatter(xs, ys, zs, c='lightgray', marker='o', s=30, alpha=0.6, label='Grid Points')
    
    # Plot base
    base_coord = coords[0]
    ax.scatter([base_coord[0]], [base_coord[1]], [base_coord[2]],
               c='red', marker='s', s=200, label='Base', edgecolors='black', linewidths=2)
    
    # Plot each trip in different color
    colors = plt.cm.tab10(np.linspace(0, 1, len(solution['trips'])))
    
    for trip_idx, trip in enumerate(solution['trips']):
        color = colors[trip_idx]
        
        # Extract route coordinates
        route_coords = [coords[node] for node in trip]
        xs, ys, zs = zip(*route_coords)
        
        # Plot route
        ax.plot(xs, ys, zs, c=color, linewidth=2, alpha=0.8,
                label=f'Viaggio {trip_idx + 1} ({len(trip)-2} pts)')
        
        # Mark visited points
        visited_coords = route_coords[1:-1]  # Exclude base
        if visited_coords:
            vxs, vys, vzs = zip(*visited_coords)
            ax.scatter(vxs, vys, vzs, c=[color], marker='o', s=50, edgecolors='black', linewidths=1)
    
    ax.set_xlabel('X (m)', fontsize=10)
    ax.set_ylabel('Y (m)', fontsize=10)
    ax.set_zlabel('Z (m)', fontsize=10)
    ax.set_title(f'Multi-Trip Drone Solution - {building_name}\n'
                 f'Total Time: {solution["total_time"]:.1f}s, Trips: {len(solution["trips"])}',
                 fontsize=12, fontweight='bold')
    ax.legend(loc='upper left', fontsize=8)
    ax.grid(True, alpha=0.3)
    
    # Save visualization
    os.makedirs("data/solutions", exist_ok=True)
    output_file = f"data/solutions/{building_name}_visualization.png"
    plt.savefig(output_file, dpi=150, bbox_inches='tight')
    print(f"\n✓ Visualization saved to: {output_file}")
    
    plt.close()


def print_solution_summary(solution, nodes_with_base):
    if solution['trips'] is None:
        print("\n❌ No solution found")
        return
    
    print("\n" + "="*60)
    print("SOLUTION SUMMARY")
    print("="*60)
    
    for i, trip in enumerate(solution['trips']):
        viaggio_num = i + 1
        points_visited = len(trip) - 2  # Exclude start and end base
        
        # Calculate trip statistics
        trip_time = EnergyModel.calculate_route_time(trip, nodes_with_base)
        trip_energy = EnergyModel.calculate_route_energy(trip, nodes_with_base)
        
        print(f"Viaggio {viaggio_num}:")
        print(f"  Route: {' → '.join(map(str, trip))}")
        print(f"  Points visited: {points_visited}")
        print(f"  Time: {trip_time:.2f} seconds")
        print(f"  Energy: {trip_energy:.0f} J ({trip_energy/3600:.3f} Wh)")
        print()
    
    print(f"Total time: {solution['total_time']:.2f} seconds")
    print(f"Total trips: {len(solution['trips'])}")
    
    if solution.get('gap') is not None:
        print(f"Optimality gap: {solution['gap']*100:.2f}%")
    
    print("="*60)


def main():
    if len(sys.argv) < 2:
        print("Error: Missing input file argument")
        print("Usage: python big_project_main.py <path_to_edificio.csv> [--heuristic-only]")
        print("Example: python big_project_main.py data/buildings/Edificio1.csv")
        print("         python big_project_main.py data/buildings/Edificio1.csv --heuristic-only")
        sys.exit(1)
    
    input_path = sys.argv[1]
    heuristic_only = "--heuristic-only" in sys.argv or "--no-mip" in sys.argv
    
    print(f"Using input file: {input_path}")
    
    # Determine building
    if "Edificio1" in input_path or "edificio1" in input_path.lower():
        building_name = "edificio1"
        matrix_name = "building1"  # Precomputed matrices use this name
        use_mip = not heuristic_only  # Use MIP unless flag set
    elif "Edificio2" in input_path or "edificio2" in input_path.lower():
        building_name = "edificio2"
        matrix_name = "building2"
        use_mip = False  # Too large, always use heuristic only
    else:
        print("Error: Unknown building (must be Edificio1 or Edificio2)")
        sys.exit(1)
    
    print(f"\n{'='*60}")
    print(f"Big Project: Single-Drone Multi-Trip Routing")
    print(f"{'='*60}")
    print(f"Building: {building_name}")
    print(f"Strategy: {'MIP with heuristic start' if use_mip else 'Heuristic only'}")
    print(f"Mode: {'HEURISTIC ONLY' if heuristic_only else 'HEURISTIC + MIP'}")
    print(f"{'='*60}\n")
    
    # Load building data
    print("Step 1: Loading building data...")
    grid_points = loadBuildingDots(input_path)
    grid_points_xyz = [(x, y, z) for (_, x, y, z) in grid_points]
    print(f"  Loaded {len(grid_points)} grid points\n")
    
    # Select base location
    print("Step 2: Selecting base location...")
    base_location = select_base_for_building(grid_points_xyz, building_name)
    
    # Build nodes list with base
    nodes_with_base = [(0, base_location[0], base_location[1], base_location[2])] + \
                      [(i+1, x, y, z) for i, (_, x, y, z) in enumerate(grid_points)]
    
    print(f"\nTotal nodes: {len(nodes_with_base)} (1 base + {len(grid_points)} grid points)\n")
    
    # Build matrices (cannot use precomputed since base is dynamically selected)
    print("Step 3: Building connectivity matrices...")
    connection_matrix, distance_matrix = build_matrices_with_base(nodes_with_base, building_name)
    print()
    
    # Get battery capacity
    battery_capacity = EnergyModel.get_battery_capacity(building_name)
    print(f"Battery capacity: {battery_capacity:.0f} J ({battery_capacity/3600:.2f} Wh)\n")
    
    # Decide whether to use MIP
    if use_mip and len(grid_points) <= 200:
        print("\nStep 4: Solving with MIP (no heuristic)...")
        final_solution = solve_multi_trip(
            nodes=nodes_with_base,
            connection_matrix=connection_matrix,
            battery_capacity=battery_capacity,
            building_name=building_name,
            heuristic_solution=None,  # No heuristic
            time_limit=7200  # 2 hours
        )
        
        # If MIP failed, try with heuristic
        if final_solution['trips'] is None:
            print("\nMIP failed. Building heuristic solution as fallback...")
            heuristic_solution = build_heuristic_solution(
                nodes=nodes_with_base,
                connection_matrix=connection_matrix,
                battery_capacity=battery_capacity
            )
            final_solution = heuristic_solution
    else:
        print("Step 4: Building heuristic solution...")
        heuristic_solution = build_heuristic_solution(
            nodes=nodes_with_base,
            connection_matrix=connection_matrix,
            battery_capacity=battery_capacity
        )
        print(f"\nStep 5: Using heuristic solution (problem too large for MIP)")
        final_solution = heuristic_solution
    
    # Print summary
    print_solution_summary(final_solution, nodes_with_base)
    
    # Format and print output
    print("\n" + "="*60)
    print("OUTPUT (Required Format):")
    print("="*60)
    output_str = format_output(final_solution, nodes_with_base)
    print(output_str)
    print("="*60)
    
    # Save solution
    method_suffix = "_heuristic" if heuristic_only else "_mip"
    output_file = f"data/solutions/{building_name.capitalize()}{method_suffix}_solution.txt"
    with open(output_file, 'w') as f:
        f.write(output_str)
    print(f"\n✓ Solution saved to: {output_file}")
    
    # Visualize
    viz_name = f"{building_name.capitalize()}{method_suffix}"
    visualize_solution(final_solution, nodes_with_base, viz_name)
    
    print("\n✓ Big project complete!")


if __name__ == "__main__":
    main()

