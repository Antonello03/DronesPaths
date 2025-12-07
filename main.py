import sys, os
sys.path.append(os.path.dirname(__file__))

from src.utils import *

if __name__ == "__main__":
    
    #input parameters
    input_path = "data/buildings/Building2.txt"
    startingPoint = (0,-40,0) # index 0 in the connection matrix and distance matrix
    yThreshold = -20.0 # all nodes with y <= yThreshold will be connected to the starting point

    #create connection and distance matrix
    nodes = loadBuildingDots(input_path)
    # connectionMatrix = createConnectionMatrixWithStartingPoints(
    #     nodes,
    #     output_path="data/connection_matrix/connection_matrix_building1_compressed.npz",
    #     startingPoint=startingPoint,
    #     yThreshold=yThreshold)
    # distanceMatrix = createDistanceMatrix(
    #     nodes,
    #     output_path="data/distance_matrix/distance_matrix_building1_compressed.npz",
    #     startingPoint=startingPoint
    # )
    connectionMatrix = np.load("data/connection_matrix/connection_matrix_building2_compressed.npz")['data']
    distanceMatrix = np.load("data/distance_matrix/distance_matrix_building2_compressed.npz")['data']
    #visualize building with connections (don't do it with building 1)

    #TODO: mip solver here


    # visualizeBuildingWithConnections(startingPoint=startingPoint, building_path=input_path, connectionMatrix=connectionMatrix, output_path="data/plots/building2_visualization_with_connections.png", show=True)