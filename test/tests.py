import sys, os
sys.path.append(os.path.dirname(__file__))
from src.utils import *

def visualizationTest():
    visualizeBuilding("data/buildings/Edificio1.csv", "data/plots/building1.png")
    connectionMatrix = np.load("data/connection_matrix/connection_matrix_building1_compressed.npz")["data"]
    visualizeBuildingWithConnections("data/buildings/Edificio1.csv", connectionMatrix, "data/plots/building1_connections.png", show=True, startingPoint=(0,-16,0))

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
    visualizationTest()