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

def compressnpy():
    data = np.load("data/buildings/connection_matrix_building1.npy")
    np.savez_compressed("data/buildings/connection_matrix_building1_compressed.npz", data=data)

def distances():
    possiblevalues = (1,2)
    for x in possiblevalues:
        for y in possiblevalues:
            for z in possiblevalues:
                for a in possiblevalues:
                    for b in possiblevalues:
                        for c in possiblevalues:
                            point1 = (x,y,z)
                            point2 = (a,b,c)
                            try:
                                time = travelTime(point1, point2)
                                print(f"From {point1} to {point2}: {time}")
                            except ValueError as e:
                                print(f"From {point1} to {point2}: {e}")

def distanceMatrix():
    nodes = loadBuildingDots("data/buildings/Building1.txt")
    distanceMatrix = createConnectionMatrixWithStartingPoints(
        nodes,
        output_path="data/buildings/distance_matrix_building1_compressed.npz",
        startingPoint=(0,-16,0),
        yThreshold=-12.5)
    print(distanceMatrix)

if __name__ == "__main__":
    distanceMatrix()
