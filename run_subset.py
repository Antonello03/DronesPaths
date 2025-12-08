import numpy as np
import os
from src.utils import loadBuildingDots, createConnectionMatrixWithStartingPoints, createDistanceMatrix, visualizeSolution
from src.model import solve_instance, print_solution_summary, format_solution_output

# Config
input_path = "data/buildings/Edificio2.csv"
subset_n = 50  # first 50 nodes
startingPoint = (0, -40, 0)
yThreshold = -20.0
k_drones = 4

print(f"Running subset test: first {subset_n} nodes of Edificio2")

nodes_full = loadBuildingDots(input_path)
nodes = nodes_full[:subset_n]
print(f"Loaded {len(nodes)} nodes (subset)")

print("Building connection matrix (subset)...")
conn = createConnectionMatrixWithStartingPoints(nodes, None, startingPoint, yThreshold)
print(f"Connection matrix shape: {conn.shape}, arcs: {int(np.sum(conn))}")

print("Building distance matrix (subset)...")
dist = createDistanceMatrix(nodes, None, startingPoint)
print(f"Distance matrix shape: {dist.shape}")

# nodes with base
nodes_with_base = [(0, startingPoint[0], startingPoint[1], startingPoint[2])] + \
                  [(i+1, x, y, z) for i, (x, y, z) in enumerate([(n[1], n[2], n[3]) for n in nodes])]

print(f"Total nodes for solver: {len(nodes_with_base)}")

result = solve_instance(
    nodes=nodes_with_base,
    connection_matrix=conn,
    distance_matrix=dist,
    k_drones=k_drones,
    time_limit=300
)

print_solution_summary(result)
print("\nOUTPUT format:\n")
solution_output = format_solution_output(result)
print(solution_output)

# Save solution to file
os.makedirs("data/solutions", exist_ok=True)
solution_file = f"data/solutions/Edificio2_subset{subset_n}_solution.txt"
with open(solution_file, 'w') as f:
    f.write(solution_output)
print(f"\n✓ Solution saved to: {solution_file}")

# Generate visualization
if result['routes'] is not None:
    viz_file = f"data/solutions/Edificio2_subset{subset_n}_visualization.png"
    visualizeSolution(
        building_path=input_path,
        solution=solution_output,
        output_path=viz_file,
        startingPoint=startingPoint
    )
    print(f"✓ Visualization saved to: {viz_file}")
