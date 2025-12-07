from matplotlib import pyplot as plt
import networkx as nx
import numpy as np
from math import sqrt

def loadBuildingDots(building_path: str) -> list[tuple[int, float, float, float]]:
    """
    Loads the building dots from the given file.
    Returns a list of tuples (index, x, y, z).
    """

    i = 0

    dots = []

    with open(building_path, "r") as f:
        next(f)
        for line in f:
            x, y, z = map(float, line.strip().split(","))
            dots.append((i, x, y, z))
            i += 1

    return dots

def euclidean3DDistance(a: tuple[float, float, float], b: tuple[float, float, float]) -> float:
    """
    Calculate the Euclidean distance between two points in 3D space.
    """
    return sqrt((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2 + (a[2] - b[2]) ** 2)

def checkEuclideanDistanceCustom(a: tuple[float, float, float], b: tuple[float, float, float]) -> bool:
    """
    Check if the Euclidean distance between two points satisfies the custom conditions specified in the small project description.
    """
    # Condition 1
    distance = euclidean3DDistance(a, b)
    condition1 = distance <= 4.0

    # Condition 2
    # the Euclidean distance between A and B is at most 11 m and two among the coordinates x, y, and z differ by at most 0.5 m
    sub_condition_2_1 = distance <= 11
    sub_condition_2_2 = sum([(abs(a[i] - b[i]) <= 0.5) for i in range(3)]) == 2
    condition2 = sub_condition_2_1 and sub_condition_2_2

    return condition1 or condition2

def travelTime(a: tuple[float, float, float], b: tuple[float, float, float]) -> float:
    """
    Calculate the travel time between two points in 3D space according to the specified rules.
    """
    upwardVelocity = 1.0
    downwardVelocity = 2.0
    horizontalVelocity = 1.5

    if a == b:
        return 0.0

    # purely upward/downward movement
    if a[0] == b[0] and a[1] == b[1]:
        if a[2] < b[2]:
            return euclidean3DDistance(a,b) / upwardVelocity
        elif a[2] > b[2]:
            return euclidean3DDistance(a,b) / downwardVelocity
    
    # purely horizontal movement
    if a[2] == b[2]:
        return euclidean3DDistance(a,b) / horizontalVelocity
    
    # horizontal and upward
    if a[2] < b[2] and (a[0] != b[0] or a[1] != b[1]):
        horizontal_distance = sqrt((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2)
        vertical_distance = abs(a[2] - b[2])
        time_horizontal = horizontal_distance / horizontalVelocity
        time_vertical = vertical_distance / upwardVelocity
        return max(time_horizontal, time_vertical)
    
    # horizontal and downward
    if a[2] > b[2] and (a[0] != b[0] or a[1] != b[1]):
        horizontal_distance = sqrt((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2)
        vertical_distance = abs(a[2] - b[2])
        time_horizontal = horizontal_distance / horizontalVelocity
        time_vertical = vertical_distance / downwardVelocity
        return max(time_horizontal, time_vertical)
    
    raise ValueError("Unhandled case for points a and b")

def createConnectionMatrixCustom(nodes: list[tuple[int, float, float, float]], output_path: str) -> np.ndarray:

    N = len(nodes)
    connectionMatrix = np.zeros((N,N), dtype = bool) # forse da togliere

    k = 0
    tot = N * (N - 1) // 2

    for i in range(N):
        a = nodes[i]
        ax, ay, az = a[1], a[2], a[3]

        for j in range(i + 1, N):
            b = nodes[j]
            bx, by, bz = b[1], b[2], b[3]

            k += 1
            print(k, "/", tot, k * 100 // tot, "%", end="\r")

            if checkEuclideanDistanceCustom((ax, ay, az), (bx, by, bz)):
                connectionMatrix[a[0], b[0]] = True
                connectionMatrix[b[0], a[0]] = True

    if output_path is not None:
        np.savez_compressed(output_path, data=connectionMatrix)
    
    return connectionMatrix

def createConnectionMatrixWithStartingPoints(nodes: list[tuple[int, float, float, float]], output_path: str, startingPoint: tuple[float, float, float], yThreshold : int) -> np.ndarray:
    """
    Create a connection matrix for the given nodes following the custom distance rules.
    If startingPoint is provided, it is added as the first node in the matrix.
    Connections from the starting point are only made to nodes with y coordinate less than or equal to yThreshold.
    """
    N = len(nodes)
    connectionMatrix = np.zeros((N+1,N+1), dtype = bool)

    k = 0
    tot = N * (N - 1) // 2

    for i in range(0,N):
        a = nodes[i]
        ax, ay, az = a[1], a[2], a[3]

        for j in range(i + 1, N):
            b = nodes[j]
            bx, by, bz = b[1], b[2], b[3]

            k += 1
            print(k, "/", tot, k * 100 // tot, "%", end="\r")

            if checkEuclideanDistanceCustom((ax, ay, az), (bx, by, bz)):
                connectionMatrix[a[0]+1, b[0]+1] = True
                connectionMatrix[b[0]+1, a[0]+1] = True

    print("\nAdding starting point connections...")
    a = startingPoint
    for j in range(0, N):
        b = nodes[j]
        if b[2] <= yThreshold:
            bx, by, bz = b[1], b[2], b[3]
            connectionMatrix[0, b[0]+1] = True
            connectionMatrix[b[0]+1, 0] = True

    if output_path is not None:
        np.savez_compressed(output_path, data=connectionMatrix)
    
    return connectionMatrix

def createDistanceMatrix(nodes: list[tuple[int, float, float, float]], output_path: str, startingPoint: tuple[float, float, float]) -> np.ndarray:
    """
    Create a distance matrix for the given nodes following the travel time rules.
    If startingPoint is provided, it is added as the first node in the matrix.
    """
    
    if startingPoint is not None:
        nodes = [(node[0]+1, node[1], node[2], node[3]) for node in nodes]
        nodes.insert(0, (0, startingPoint[0], startingPoint[1], startingPoint[2]))

    N = len(nodes)
    distanceMatrix = np.zeros((N,N), dtype = float)

    k = 0
    tot = N * N - N

    for i in range(N):
        a = nodes[i]
        ax, ay, az = a[1], a[2], a[3]

        for j in range(N):
            b = nodes[j]
            bx, by, bz = b[1], b[2], b[3]

            k += 1
            print(k, "/", tot, k * 100 // tot, "%", end="\r")
            distanceMatrix[a[0], b[0]] = travelTime((ax, ay, az), (bx, by, bz))

    if output_path is not None:
        np.savez_compressed(output_path, data=distanceMatrix)

    return distanceMatrix

def visualizeBuilding(building_path: str, output_path: str = None, show: bool = False) -> None:
    """
    Generate a 3D plot to visualize the given building structure.
    """

    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    step = 3000
    count = 0

    print(f"Plotting to file visualization from {building_path}...")

    dots = loadBuildingDots(building_path)

    xs = [dot[1] for dot in dots]
    ys = [dot[2] for dot in dots]
    zs = [dot[3] for dot in dots]

    ax.scatter(xs, ys, zs, c='tab:blue', s= 0.6)

    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_zlabel("Z")

    if output_path is not None:
        print("Saving plot...")
        plt.savefig(output_path, dpi=200)
        print(f"Saved plot to {output_path}")

    if show:
        plt.show()

def visualizeBuildingWithConnections(building_path: str, connectionMatrix: np.ndarray, output_path: str = None, show: bool = False, startingPoint: tuple[float, float, float] = None) -> None:
    """
    Generate a 3D plot to visualize the given building structure with connections.
    """

    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    print(f"Plotting to file visualization from {building_path} with connections...")

    dots = loadBuildingDots(building_path)

    if startingPoint is not None:
        dots.insert(0, (0, startingPoint[0], startingPoint[1], startingPoint[2]))

    xs = [dot[1] for dot in dots]
    ys = [dot[2] for dot in dots]
    zs = [dot[3] for dot in dots]

    ax.scatter(xs, ys, zs, c='tab:blue', s= 0.6)

    # Plot connections
    N = len(dots)
    for i in range(N):
        for j in range(i + 1, N):
            if connectionMatrix[i][j]:
                x_values = [dots[i][1], dots[j][1]]
                y_values = [dots[i][2], dots[j][2]]
                z_values = [dots[i][3], dots[j][3]]
                ax.plot(x_values, y_values, z_values, c='tab:gray', linewidth=0.2)

    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_zlabel("Z")

    if output_path is not None:
        print("Saving plot...")
        plt.savefig(output_path, dpi=200)
        print(f"Saved plot to {output_path}")

    if show:
        plt.show()

# code belowe works only without starting point in connection matrix

def visualizeSolution(building_path, solution:str, output_path: str) -> None:
    """
    Generate a 3D plot to visualize the given drone path.
    """

    #TODO add initial and final positions different colors and visual improvements 

    # Parse solution
    drones_paths = []
    for line in solution.strip().split("\n"):
        path = []
        parts = line.split(":")[1].strip().split("-")
        for part in parts:
            index = int(part)
            path.append(index)
        drones_paths.append(path)

    print(drones_paths)

    # Load building dots
    dots = loadBuildingDots(building_path)

    # Plot paths and building dots
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    colors = ['tab:red', 'tab:green', 'tab:orange', 'tab:purple', 'tab:brown', 'tab:pink', 'tab:gray', 'tab:olive']

    for path in drones_paths:
        xs = []
        ys = []
        zs = []
        for index in path:
            _, x, y, z = dots[index]
            xs.append(x)
            ys.append(y)
            zs.append(z)
        ax.plot(xs, ys, zs, c= colors[drones_paths.index(path) % len(colors)], linewidth=0.4)

    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_zlabel("Z")
    plt.savefig(output_path, dpi=200)

def generateTestSolution(building_path) -> str:
    """
    Generate an unfeasible test solution for the given building to test visualization.
    """
    i = 0

    dots = loadBuildingDots(building_path)

    dots.sort(key=lambda p: (p[1], p[2], p[3]))

    drones_paths = []
    n_drones = 4

    n_dots = len(dots)
    dots_per_drone = n_dots // n_drones

    for i in range(n_drones):
        start_index = i * dots_per_drone
        end_index = (i + 1) * dots_per_drone if i != n_drones - 1 else n_dots
        drone_path = dots[start_index:end_index]
        drones_paths.append(drone_path)

    solution_lines = []
    for drone_id, path in enumerate(drones_paths):
        path_str = ""
        for point in path:
            index, _, _, _ = point
            path_str += f"{index}-"
        solution_lines.append(f"Drone {drone_id + 1}: {path_str[:-1]}")

    return "\n".join(solution_lines)