import numpy as np
from math import sqrt

def loadBuildingDots(building_path: str, removeDuplicates=True):
    dots_raw = []

    with open(building_path, "r") as f:
        f.readline()  # <-- FIX: niente next(f)
        for line in f:
            if not line.strip():
                continue
            x, y, z = map(float, line.strip().split(","))
            dots_raw.append((x, y, z))

    if removeDuplicates:
        seen = set()
        unique = []
        for p in dots_raw:
            if p not in seen:
                seen.add(p)
                unique.append(p)
        dots_raw = unique

    return [(i, x, y, z) for i, (x, y, z) in enumerate(dots_raw)]

def euclidean(a, b):
    return sqrt((a[0]-b[0])**2 + (a[1]-b[1])**2 + (a[2]-b[2])**2)

def check_connectivity(a, b):
    dx, dy, dz = abs(a[0]-b[0]), abs(a[1]-b[1]), abs(a[2]-b[2])
    dist = euclidean(a, b)

    if dist <= 4.0:
        return True
    if dist <= 11.0:
        diffs = sorted([dx, dy, dz])
        return diffs[0] <= 0.5 and diffs[1] <= 0.5
    return False