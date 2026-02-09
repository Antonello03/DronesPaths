

import sys
import os
sys.path.append(os.path.dirname(__file__))

import numpy as np
import matplotlib.pyplot as plt

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
            if i == j:
                continue

            node_i = nodes_with_base[i]
            node_j = nodes_with_base[j]

            # Special case: base can connect to entry points regardless of distance
            if (i == 0 and j in entry_points) or (j == 0 and i in entry_points):
                connected = True
            else:
                connected = check_connectivity(node_i, node_j)

            connection_matrix[i][j] = connected

            if connected:
                point_i = (node_i[1], node_i[2], node_i[3])
                point_j = (node_j[1], node_j[2], node_j[3])
                distance_matrix[i][j] = EnergyModel.calculate_time(point_i, point_j)

    return connection_matrix, distance_matrix


def check_connectivity(node1, node2):
    x1, y1, z1 = node1[1], node1[2], node1[3]
    x2, y2, z2 = node2[1], node2[2], node2[3]

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


def format_output(solution):
    if solution is None or solution.get('trips') is None:
        return "No solution found."

    if len(solution['trips']) == 0:
        return "No trips extracted from solution."

    lines = []
    for i, trip in enumerate(solution['trips']):
        viaggio_num = i + 1
        trip_str = "-".join(map(str, trip))
        lines.append(f"Viaggio {viaggio_num}: {trip_str}")

    return "\n".join(lines)


def visualize_solution(solution, nodes_with_base, out_name):
    if solution is None or solution.get('trips') is None:
        print("No solution to visualize")
        return

    fig = plt.figure(figsize=(14, 10))
    ax = fig.add_subplot(111, projection='3d')

    coords = {node[0]: (node[1], node[2], node[3]) for node in nodes_with_base}

    grid_points = [coords[i] for i in range(1, len(nodes_with_base))]
    if grid_points:
        xs, ys, zs = zip(*grid_points)
        ax.scatter(xs, ys, zs, c='lightgray', marker='o', s=30, alpha=0.6, label='Grid Points')

    base_coord = coords[0]
    ax.scatter([base_coord[0]], [base_coord[1]], [base_coord[2]],
               c='red', marker='s', s=200, label='Base', edgecolors='black', linewidths=2)

    colors = plt.cm.tab10(np.linspace(0, 1, len(solution['trips'])))

    for trip_idx, trip in enumerate(solution['trips']):
        color = colors[trip_idx]

        route_coords = [coords[node] for node in trip]
        xs, ys, zs = zip(*route_coords)
        ax.plot(xs, ys, zs, c=color, linewidth=2, alpha=0.8,
                label=f'Viaggio {trip_idx + 1} ({len(trip)-2} pts)')

        visited_coords = route_coords[1:-1]
        if visited_coords:
            vxs, vys, vzs = zip(*visited_coords)
            ax.scatter(vxs, vys, vzs, c=[color], marker='o', s=50, edgecolors='black', linewidths=1)

    ax.set_xlabel('X (m)', fontsize=10)
    ax.set_ylabel('Y (m)', fontsize=10)
    ax.set_zlabel('Z (m)', fontsize=10)
    ax.set_title(f'Multi-Trip Drone Solution {out_name}\n'
                 f'Total Time: {solution["total_time"]:.1f}s, Trips: {len(solution["trips"])}',
                 fontsize=12, fontweight='bold')
    ax.legend(loc='upper left', fontsize=8)
    ax.grid(True, alpha=0.3)

    os.makedirs("data/solutions", exist_ok=True)
    output_file = f"data/solutions/{out_name}_visualization.png"
    plt.savefig(output_file, dpi=150, bbox_inches='tight')
    print(f"\n✓ Visualization saved to: {output_file}")

    plt.close()


def main():
    if len(sys.argv) < 2:
        print("Usage: python big_project_main.py <path_to_edificio.csv> [--heuristic-only] [--mip-only] [--both] [--force-mip] [--time-limit <sec>] [--threads <n>] [--pump-passes <n>]")
        sys.exit(1)

    input_path = sys.argv[1]
    args = sys.argv[2:]

    heuristic_only = ("--heuristic-only" in args) or ("--no-mip" in args)
    mip_only = ("--mip-only" in args)
    both = ("--both" in args)
    force_mip = ("--force-mip" in args)

    # Default project time limit: 2 hours
    time_limit = 7200
    if "--time-limit" in args:
        idx = args.index("--time-limit")
        if idx + 1 >= len(args):
            print("Error: --time-limit requires an integer value (seconds)")
            sys.exit(1)
        try:
            time_limit = int(args[idx + 1])
        except ValueError:
            print("Error: --time-limit must be an integer (seconds)")
            sys.exit(1)
        if time_limit <= 0:
            print("Error: --time-limit must be > 0")
            sys.exit(1)

    # CBC settings for faster incumbent search
    default_threads = max(1, min(8, os.cpu_count() or 1))
    threads = default_threads
    if "--threads" in args:
        idx = args.index("--threads")
        if idx + 1 >= len(args):
            print("Error: --threads requires an integer value")
            sys.exit(1)
        try:
            threads = int(args[idx + 1])
        except ValueError:
            print("Error: --threads must be an integer")
            sys.exit(1)
        if threads <= 0:
            print("Error: --threads must be > 0")
            sys.exit(1)

    pump_passes = 100
    if "--pump-passes" in args:
        idx = args.index("--pump-passes")
        if idx + 1 >= len(args):
            print("Error: --pump-passes requires an integer value")
            sys.exit(1)
        try:
            pump_passes = int(args[idx + 1])
        except ValueError:
            print("Error: --pump-passes must be an integer")
            sys.exit(1)
        if pump_passes < 0:
            print("Error: --pump-passes must be >= 0")
            sys.exit(1)

    
    if not heuristic_only and not mip_only and not both:
        both = True

    print(f"Using input file: {input_path}")

    # Determine building
    if "Edificio1" in input_path or "edificio1" in input_path.lower():
        building_name = "edificio1"
    elif "Edificio2" in input_path or "edificio2" in input_path.lower():
        building_name = "edificio2"
    else:
        print("Error: Unknown building (must be Edificio1 or Edificio2)")
        sys.exit(1)

    print(f"\n{'='*60}")
    print(f"Big Project: Single-Drone Multi-Trip Routing")
    print(f"{'='*60}")
    print(f"Building: {building_name}")

    if heuristic_only:
        print("Strategy: Heuristic only")
        print("Mode: HEURISTIC ONLY")
    elif mip_only:
        print("Strategy: MIP only")
        print("Mode: MIP ONLY")
    else:
        print("Strategy: MIP with heuristic start (comparison)")
        print("Mode: HEURISTIC + MIP")
    print(f"{'='*60}\n")
    print(f"Solver settings: threads={threads}, pump_passes={pump_passes}, time_limit={time_limit}s\n")

    # Load building data
    print("Step 1: Loading building data...")
    grid_points = loadBuildingDots(input_path)
    grid_points_xyz = [(x, y, z) for (_, x, y, z) in grid_points]
    print(f"  Loaded {len(grid_points)} grid points\n")

    # Select base location
    print("Step 2: Selecting base location...")
    base_location = select_base_for_building(grid_points_xyz, building_name)

    nodes_with_base = [(0, base_location[0], base_location[1], base_location[2])] + \
                      [(i+1, x, y, z) for i, (_, x, y, z) in enumerate(grid_points)]

    print(f"\nTotal nodes: {len(nodes_with_base)} (1 base + {len(grid_points)} grid points)\n")

    # Build matrices
    print("Step 3: Building connectivity matrices...")
    connection_matrix, _distance_matrix = build_matrices_with_base(nodes_with_base, building_name)
    print()

    battery_capacity = EnergyModel.get_battery_capacity(building_name)
    print(f"Battery capacity: {battery_capacity:.0f} J ({battery_capacity/3600:.2f} Wh)\n")

    os.makedirs("data/solutions", exist_ok=True)

    heuristic_solution = None
    mip_solution = None


    # HEURISTIC
   
    if not mip_only:
        print("Step 4: Building heuristic solution...")
        heuristic_solution = build_heuristic_solution(
            nodes=nodes_with_base,
            connection_matrix=connection_matrix,
            battery_capacity=battery_capacity
        )

        out_name = f"{building_name.capitalize()}_heuristic"
        out_txt = format_output(heuristic_solution)

        print("\n================ HEURISTIC SOLUTION =================")
        print(out_txt)
        print("====================================================")

        out_file = f"data/solutions/{out_name}_solution.txt"
        with open(out_file, "w") as f:
            f.write(out_txt)

        print(f"\n✓ Heuristic solution saved to: {out_file}")

        visualize_solution(heuristic_solution, nodes_with_base, out_name)


    # MIP 

    if not heuristic_only:
        if building_name == "edificio2" and (not force_mip) and (not mip_only):
            print("\nStep 5: Solving with MIP on Edificio2 (may take longer).")
            print("       Use --heuristic-only if you want to skip MIP.")
        else:
            print("\nStep 5: Solving with MIP...")

       
        mip_solution = solve_multi_trip(
            nodes=nodes_with_base,
            connection_matrix=connection_matrix,
            battery_capacity=battery_capacity,
            building_name=building_name,
            heuristic_solution=heuristic_solution,
            time_limit=time_limit,
            threads=threads,
            pump_passes=pump_passes
        )

        # if MIP fails to find a feasible solution, fall back to heuristic
        if mip_solution is None:
            print("\nMIP failed to find a feasible solution within the limit. Falling back to heuristic to produce required output.")
            mip_solution = build_heuristic_solution(
                nodes=nodes_with_base,
                connection_matrix=connection_matrix,
                battery_capacity=battery_capacity
            )
            out_suffix = "heuristic_fallback"
        else:
            out_suffix = "mip"

        out_name = f"{building_name.capitalize()}_{out_suffix}"
        out_txt = format_output(mip_solution)   

        print("\n===================== MIP SOLUTION =====================")
        print(out_txt)
        print("========================================================")

        out_file = f"data/solutions/{out_name}_solution.txt"
        with open(out_file, "w") as f:
            f.write(out_txt)

        print(f"\n✓ MIP solution saved to: {out_file}")

        visualize_solution(mip_solution, nodes_with_base, out_name)

    print("\n✓ Big project complete!")


if __name__ == "__main__":
    main()
