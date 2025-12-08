"""
Visualize drone solutions from solution files
"""
import sys
sys.path.append('.')

from src.utils import visualizeSolution
import os

def visualize_from_file(solution_file, building_file, output_file, starting_point=None):
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
    
    # Create visualization
    visualizeSolution(building_file, solution_text, output_file, starting_point)
    print(f"✓ Visualization saved to {output_file}")

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Error: Missing solution file argument")
        print("Usage: python visualize_solution.py <solution_file.txt>")
        print("Example: python visualize_solution.py data/solutions/Edificio2_solution.txt")
        sys.exit(1)
    
    solution_file = sys.argv[1]
    
    if not os.path.exists(solution_file):
        print(f"Error: Solution file not found: {solution_file}")
        sys.exit(1)
    
    # Determine building file and output based on solution filename
    basename = os.path.basename(solution_file).replace("_solution.txt", "")
    building_file = f"data/buildings/{basename}.csv"
    output_file = f"data/solutions/{basename}_visualization.png"
    
    if not os.path.exists(building_file):
        print(f"Error: Building file not found: {building_file}")
        sys.exit(1)
    
    visualize_from_file(solution_file, building_file, output_file)
    print("\n✓ Visualization complete!")

