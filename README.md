# Drone TSP on a 3D Building

This project sets up data and solves a **4‑drone Traveling Salesman Problem (TSP)** on a 3D building using a MIP solver.

## Structure

* `main.py` – example entry point: loads a building, matrices, and runs the MIP solver.
* `src/utils.py` – core utilities:

  * `loadBuildingDots`
  * `createConnectionMatrixWithStartingPoints`
  * `createDistanceMatrix`
  * visualization helpers.
* `src/model.py` – MIP model for the 4‑drone TSP.
* `data/`

  * `buildings/` – 3D points (txt).
  * `connection_matrix/` – adjacency matrices (`.npz`).
  * `distance_matrix/` – travel‑time matrices (`.npz`).
  * `plots/`, `solutions/` – saved visualizations.
* `tests.py` – small checks.
* `requirements.txt` – dependencies.

## Building data

Each building file in `data/buildings/` has a header line, then one point per row:

```text
x,y,z
x,y,z
...
```

## Core workflow

```python
from src.utils import (
    loadBuildingDots,
    createConnectionMatrixWithStartingPoints,
    createDistanceMatrix,
)

building = "data/buildings/Building2.txt"
start = (0.0, -40.0, 0.0)
y_thr = -20.0

nodes = loadBuildingDots(building)

connection_matrix = createConnectionMatrixWithStartingPoints(
    nodes=nodes,
    output_path="data/connection_matrix/connection_matrix_building2_compressed.npz",
    startingPoint=start,
    yThreshold=y_thr,
)

distance_matrix = createDistanceMatrix(
    nodes=nodes,
    output_path="data/distance_matrix/distance_matrix_building2_compressed.npz",
    startingPoint=start,
)
```

Then run the MIP (for example from `main.py`) to obtain the four drone tours.

## Visualizing

* `visualizeBuilding` – building point cloud.
* `visualizeBuildingWithConnections` – building + graph edges.
