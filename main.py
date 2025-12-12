import sys, os
sys.path.append(os.path.dirname(__file__))

from src.utils import *
from src.model import solve_instance, format_solution_output, print_solution_summary

if __name__ == "__main__":
    
    # Check if input file is provided as command line argument
    if len(sys.argv) < 2:
        print("Error: Missing input file argument")
        print("Usage: python main.py <path_to_edificio.csv>")
        print("Example: python main.py data/buildings/Edificio1.csv")
        sys.exit(1)
    
    input_path = sys.argv[1]
    print(f"Using input file: {input_path}")
    
    # Determine which building we're using to load the right matrices
    if "Edificio1" in input_path or "Building1" in input_path:
        building_name = "building1"
        instance_name = "Edificio1"
        startingPoint = (0, -16, 0)
        yThreshold = -12.5
    elif "Test" in input_path:
        building_name = "buildingtest"
        instance_name = "BuildingTest"
        startingPoint = (0, 0, 0)
        yThreshold = 2.0  # Half the points as entry points
    else:
        building_name = "building2"
        instance_name = "Edificio2"
        startingPoint = (0, -40, 0)
        yThreshold = -20.0
    
    print(f"\n{'='*60}")
    print(f"Multi-Drone Routing Problem Solver")
    print(f"{'='*60}")
    print(f"Building: {input_path}")
    print(f"Starting point: {startingPoint}")
    print(f"Entry points threshold: y <= {yThreshold}")
    print(f"{'='*60}\n")
    
    # Load building data
    print("Loading building data...")
    nodes = loadBuildingDots(input_path, removeDuplicates=True)
    
    print(f"Loaded {len(nodes)} grid points\n")
    
    # Load or compute connection and distance matrices
    print("Loading connection and distance matrices...")

    cm_path = f"data/connection_matrix/connection_matrix_{building_name}_compressed.npz"
    dm_path = f"data/distance_matrix/distance_matrix_{building_name}_compressed.npz"

    # --- connection matrix ---
    if os.path.exists(cm_path):
        connectionMatrix = np.load(cm_path)["data"]
        print("Loaded connection matrix from disk.")
    else:
        print("Connection matrix not found. Computing...")
        connectionMatrix = createConnectionMatrixWithStartingPoints(nodes, cm_path, startingPoint, yThreshold)
        print("Computed connection matrix.")

    # --- distance matrix ---
    if os.path.exists(dm_path):
        distanceMatrix = np.load(dm_path)["data"]
        print("Loaded distance matrix from disk.")
    else:
        print("Distance matrix not found. Computing...")
        distanceMatrix = createDistanceMatrix(nodes, dm_path, startingPoint)
        print("Computed distance matrix.")

    print(f"Connection matrix shape: {connectionMatrix.shape}")
    print(f"Distance matrix shape: {distanceMatrix.shape}")

    num_arcs = int(np.sum(connectionMatrix))
    print(f"Total feasible arcs: {num_arcs}\n")

    
    # The pre-computed matrices already include base at index 0
    # So we need to add base to nodes list to match matrix indexing
    nodes_with_base = [(0, startingPoint[0], startingPoint[1], startingPoint[2])] + \
                      [(i+1, x, y, z) for i, x, y, z in nodes]
    
    print(f"Total nodes for solver: {len(nodes_with_base)} (1 base + {len(nodes)} grid points)\n")
    
    # Solve the multi-drone routing problem
    print("Starting MIP solver...\n")
    k_drones = 4  # assignment requirement
    time_limit = 10**9  # practically infinite; do not cut solver early
    print(f"Running with {k_drones} drones, time limit = {time_limit} seconds (practically infinite)")
    result = solve_instance(
        nodes=nodes_with_base,
        connection_matrix=connectionMatrix,
        distance_matrix=distanceMatrix,
        k_drones=k_drones,
        time_limit=time_limit
    )
    
    # Print detailed summary
    print_solution_summary(result)
    
    # Print solution in required format
    print("\n" + "="*60)
    print("OUTPUT (Required Format):")
    print("="*60)
    solution_output = format_solution_output(result)
    print(solution_output)
    print("="*60)
    
    # Save solution to file
    import os
    os.makedirs("data/solutions", exist_ok=True)
    solution_file = f"data/solutions/{instance_name}_solution.txt"
    with open(solution_file, 'w') as f:
        f.write(solution_output)
    print(f"\n✓ Solution saved to: {solution_file}")