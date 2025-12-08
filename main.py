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
    nodes = loadBuildingDots(input_path)
    
    # Add starting point to nodes list (index 0)
    nodes_with_base = [(0, startingPoint[0], startingPoint[1], startingPoint[2])] + \
                      [(i+1, x, y, z) for i, x, y, z in nodes]
    
    print(f"Loaded {len(nodes)} grid points + 1 base point = {len(nodes_with_base)} total nodes\n")
    
    # Load pre-computed matrices
    print("Loading connection and distance matrices...")
    try:
        connectionMatrix = np.load(f"data/connection_matrix/connection_matrix_{building_name}_compressed.npz")['data']
        distanceMatrix = np.load(f"data/distance_matrix/distance_matrix_{building_name}_compressed.npz")['data']
        print(f"Connection matrix shape: {connectionMatrix.shape}")
        print(f"Distance matrix shape: {distanceMatrix.shape}")
        
        # Count feasible arcs
        num_arcs = np.sum(connectionMatrix)
        print(f"Total feasible arcs: {num_arcs}\n")
        
    except FileNotFoundError as e:
        print(f"Error: Could not find pre-computed matrices for {building_name}")
        print("Please run the matrix generation code first.")
        sys.exit(1)
    
    # Solve the multi-drone routing problem
    print("Starting MIP solver...\n")
    result = solve_instance(
        nodes=nodes_with_base,
        connection_matrix=connectionMatrix,
        distance_matrix=distanceMatrix,
        k_drones=4,
        time_limit=300  # 5 minutes time limit
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