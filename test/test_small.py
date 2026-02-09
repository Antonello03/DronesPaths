"""Quick test with a small instance"""
import sys
sys.path.append('.')

from src.utils import *
from src.model import solve_instance, format_solution_output, print_solution_summary

# Small test instance
building_path = "data/buildings/BuildingTest.csv"
startingPoint = (0, 0, 0)
yThreshold = 2.0  # Half the points can connect to base

print("Creating small test instance...")
print(f"Building: {building_path}")
print(f"Starting point: {startingPoint}")
print(f"Entry threshold: y <= {yThreshold}\n")

# Load nodes
nodes = loadBuildingDots(building_path)
print(f"Loaded {len(nodes)} grid points\n")

# Create matrices
print("Creating connection matrix...")
connectionMatrix = createConnectionMatrixWithStartingPoints(
    nodes,
    output_path=None,  # Don't save
    startingPoint=startingPoint,
    yThreshold=yThreshold
)

print("\nCreating distance matrix...")
distanceMatrix = createDistanceMatrix(
    nodes,
    output_path=None,  # Don't save
    startingPoint=startingPoint
)

# Add starting point to nodes list
nodes_with_base = [(0, startingPoint[0], startingPoint[1], startingPoint[2])] + \
                  [(i+1, x, y, z) for i, x, y, z in nodes]

print(f"\nConnection matrix: {connectionMatrix.shape}")
print(f"Distance matrix: {distanceMatrix.shape}")
print(f"Total nodes: {len(nodes_with_base)}\n")

# Solve
print("="*60)
print("SOLVING WITH MIP")
print("="*60)

result = solve_instance(
    nodes=nodes_with_base,
    connection_matrix=connectionMatrix,
    distance_matrix=distanceMatrix,
    k_drones=4,
    time_limit=60  # 1 minute should be plenty for 20 nodes
)

# Print results
print_solution_summary(result)

print("\n" + "="*60)
print("OUTPUT (Required Format):")
print("="*60)
print(format_solution_output(result))
print("="*60)

