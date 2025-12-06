import sys, os
sys.path.append(os.path.dirname(__file__))
from src.utils import *

def visualizationTest():
    visualizeBuilding("data/buildings/Building1.txt", "data/buildings/building1_visualization.png")
    testSolution = generateTestSolution("data/buildings/Building1.txt")
    visualizeSolution("data/buildings/Building1.txt", testSolution, "data/solutions/test_solution_visualization.png")

def euclideanDistanceTest():
    #realizeGraphFromBuilding("data/buildings/Building1.txt")
    a  = (1, 4, 3)
    b = (2, 2, 1.5)
    print(euclidean3DDistance(a,b), checkEuclideanDistanceCustom(a,b))

def graphCreationTest():
    G = realizeGraphFromBuilding("data/buildings/Building2.txt", "data/buildings/connection_matrix_building2.npy")
    print(G.number_of_nodes(), G.number_of_edges())
    visualizeBuildingWithConnections("data/buildings/Building2.txt", G, show=True)

def compressnpy():
    data = np.load("data/buildings/connection_matrix_building1.npy")
    np.savez_compressed("data/buildings/connection_matrix_building1_compressed.npz", data=data)

if __name__ == "__main__":
    pass