# Docker Run Guide

Quick reference for running the drone solver and visualizations in Docker.

## Prerequisites

```bash
# Build the Docker image
docker build --platform linux/amd64 -t drone-solver .
```

## Run Solver

### Test Instance (20 nodes)
```bash
docker run --rm --platform linux/amd64 -v $(pwd):/app drone-solver \
  python -u main.py data/buildings/BuildingTest.csv
```

### Edificio1 (13,284 nodes)
```bash
docker run --rm --platform linux/amd64 -v $(pwd):/app drone-solver \
  python -u main.py data/buildings/Edificio1.csv
```

### Edificio2 (696 nodes)
```bash
docker run --rm --platform linux/amd64 -v $(pwd):/app drone-solver \
  python -u main.py data/buildings/Edificio2.csv
```

**Output:** Solution saved to `data/solutions/<name>_solution.txt`

## Create Visualizations

After solving, visualize the routes:

```bash
# Test
docker run --rm --platform linux/amd64 -v $(pwd):/app drone-solver \
  python -u visualize_solution.py data/solutions/BuildingTest_solution.txt

# Edificio2
docker run --rm --platform linux/amd64 -v $(pwd):/app drone-solver \
  python -u visualize_solution.py data/solutions/Edificio2_solution.txt

# Edificio1
docker run --rm --platform linux/amd64 -v $(pwd):/app drone-solver \
  python -u visualize_solution.py data/solutions/Edificio1_solution.txt
```

**Output:** Visualization saved to `data/solutions/<name>_visualization.png`

## Run in Background & Check Logs

```bash
# Run in background
docker run --rm --platform linux/amd64 -v $(pwd):/app --name drone-solver-run \
  drone-solver python -u main.py data/buildings/Edificio2.csv &

# Check logs
docker logs -f drone-solver-run

# Check if still running
docker ps | grep drone-solver
```

## Solution Output Format

```
Drone 1: 0-4-11-17-...-2-0
Drone 2: 0-5-6-3-...-7-0
Drone 3: 0-9-...-0
Drone 4: 0-12-...-0
```

Where `0` is the base point and other numbers are grid point indices.

