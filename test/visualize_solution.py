"""
Visualize drone solutions from solution files
"""
import sys
sys.path.append('.')

from src.utils import visualizeSolution
import os

def visualize_from_file(solution_file, building_file, output_file, starting_point=None, show=False):
    """Read solution file and create visualization"""
    
    # Read solution
    with open(solution_file, 'r') as f:
        solution_text = f.read()
    
    print(f"Visualizing solution from: {solution_file}")
    print(f"Building data: {building_file}")
    print(f"Output: {output_file}")
    
    # Determine starting point based on building
    if "Edificio1" in building_file or "Building1" in building_file:
        starting_point = (0, -16, 0)
    elif "Edificio2" in building_file or "Building2" in building_file:
        starting_point = (0, -40, 0)
    elif "Test" in building_file:
        starting_point = (0, 0, 0)
    elif "Edificio4" in building_file or "Building4" in building_file:
        starting_point = (0, -40, 0)
    
    # Create visualization
    visualizeSolution(building_file, solution_text, output_file, starting_point, show=show)
    print(f"✓ Visualization saved to {output_file}")

if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description="Visualize a solution on a building.")
    parser.add_argument("solution_file", help="Path to solution file (e.g., data/solutions/Edificio2_solution.txt)")
    parser.add_argument(
        "--building",
        dest="building_file",
        default=None,
        help="Path to building file (e.g., data/buildings/Edificio2.csv). If omitted, it is inferred from the solution filename.",
    )
    parser.add_argument(
        "--output",
        dest="output_file",
        default=None,
        help="Path to output image. If omitted, defaults to data/solutions/<basename>_visualization.png",
    )
    parser.add_argument(
        "--show",
        action="store_true",
        help="Show interactive plot window (in addition to saving the image).",
    )

    args = parser.parse_args()

    solution_file = args.solution_file
    if not os.path.exists(solution_file):
        print(f"Error: Solution file not found: {solution_file}")
        sys.exit(1)

    basename = os.path.basename(solution_file).replace("_solution.txt", "")

    building_file = args.building_file or f"data/buildings/{basename}.csv"
    output_file = args.output_file or f"data/solutions/{basename}_visualization.png"

    if not os.path.exists(building_file):
        print(f"Error: Building file not found: {building_file}")
        sys.exit(1)

    visualize_from_file(solution_file, building_file, output_file, show=args.show)
    print("\n✓ Visualization complete!")
