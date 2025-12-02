import sys, os
sys.path.append(os.path.dirname(__file__))
from src.utils import visualizeSolution, checkSolutionFeasibility, visualizeBuilding, generateTestSolution

if __name__ == "__main__":
    testSolution = generateTestSolution("data/buildings/Building1.txt")
    visualizeSolution("data/buildings/Building1.txt", testSolution, "data/solutions/test_solution_visualization.png")
