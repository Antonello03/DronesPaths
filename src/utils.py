from matplotlib import pyplot as plt

def checkSolutionFeasibility(solution):
    """
    Checks if the given solution is feasible.
    """
    pass

def visualizeSolution(solution) -> None:
    """
    Generate a 3D plot to visualize the given drone path.
    """

    #TODO

    pass

def visualizeBuilding(building_path: str, output_path: str = "data/buildings/building_plot.png") -> None:
    """
    Generate a 3D plot to visualize the given building structure.
    """

    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    step = 3000
    count = 0

    print(f"Plotting to file visualization from {building_path}...")

    xs = []
    ys = []
    zs = []

    with open(building_path, "r") as f:
        next(f)
        for line in f:
            count += 1
            if count % step == 0:
                print(f"Loaded {count} points...")
            x, y, z = map(float, line.strip().split(","))
            xs.append(x)
            ys.append(y)
            zs.append(z)

    ax.scatter(xs, ys, zs, c='tab:blue', s= 0.6)

    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_zlabel("Z")

    print("Saving plot...")
    plt.savefig(output_path, dpi=200)
    print(f"Saved plot to {output_path}")

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
    dots = []
    with open(building_path, "r") as f:
        next(f)
        for line in f:
            x, y, z = map(float, line.strip().split(","))
            dots.append((x, y, z))


    # Plot paths and building dots
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    colors = ['tab:red', 'tab:green', 'tab:orange', 'tab:purple', 'tab:brown', 'tab:pink', 'tab:gray', 'tab:olive']

    for path in drones_paths:
        xs = []
        ys = []
        zs = []
        for index in path:
            x, y, z = dots[index]
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

    dots = []

    with open(building_path, "r") as f:
        next(f)
        for line in f:
            x, y, z = map(float, line.strip().split(","))
            dots.append((i, x, y, z))
            i += 1

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