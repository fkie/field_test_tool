#!/usr/bin/env python3
"""trajectoryDistance.py: utility functions to compute distance between geographic trajectories."""

__author__ = "Johannes Pellenz, Carlos Tampier Cotoras"
__copyright__ = "Copyright 2021, Fraunhofer FKIE"
__license__ = "MIT"
__maintainer__ = "Carlos Tampier Cotoras"
__email__ = "carlos.tampier.cotoras@fkie.fraunhofer.de"

import math

def distL2(a, b):
    # Calculate 2D point distance in meters.
    diff = [b[0] - a[0], b[1] - a[1]]
    return math.hypot(*diff)

def mdistL2(t0, t1):
    # Calculate 2D distance matrix between the trajectory points.
    mdist = []
    for a in t0:
        row = []
        for b in t1:
            row.append(distL2(a, b))
        mdist.append(row)
    return mdist

def diffInMetersFromLngLat(a, b):
    # Estimate differences of lat and lng degrees in meters.
    # https://en.wikipedia.org/wiki/Geographic_coordinate_system
    latMid = ((a[1] + b[1]) / 2) * (math.pi / 180)
    mPerLat = (
        111132.92
        - 559.82 * math.cos(2 * latMid)
        + 1.175 * math.cos(4 * latMid)
        - 0.0023 * math.cos(6 * latMid)
    )
    mPerLng = (
        111412.84 * math.cos(latMid)
        - 93.5 * math.cos(3 * latMid)
        + 0.118 * math.cos(5 * latMid)
    )
    return [(b[0] - a[0]) * mPerLng, (b[1] - a[1]) * mPerLat]

def distL2FromLngLat(a, b):
    # Estimate the distance between lng, lat coordinates
    diffInMeters = diffInMetersFromLngLat(a, b)
    return math.hypot(*diffInMeters)

def mdistL2FromLngLat(t0, t1):
    # Calculate 2D distance matrix between the trajectory lng,lat coordinates.
    mdist = []
    for a in t0:
        row = []
        for b in t1:
            row.append(distL2FromLngLat(a, b))
        mdist.append(row)
    return mdist

def distPointToSegmentFromLngLat(p, a, b):
    # Calculate the distance between a lng, lat coordinate and the line defined by other two coordinates
    # First, project the points to the euclidean space, taking "a" as reference
    px, py = diffInMetersFromLngLat(a, p)
    bx, by = diffInMetersFromLngLat(a, b)

    # Then, calculate the point to line distance in the euclidean space
    dot = px * bx + py * by
    len_sq = bx * bx + by * by
    param = -1
    if len_sq != 0:
        param = dot / len_sq

    if param < 0:
        xx = 0
        yy = 0
    elif 0 < param < 1:
        xx = param * bx
        yy = param * by
    else:
        xx = bx
        yy = by

    dx = px - xx
    dy = py - yy
    return math.sqrt(dx * dx + dy * dy)

def trajectoryLength(t):
    # Calculate the length in meters of a trajectory with lng,lat coordinates.
    length = 0
    for i in range(len(t) - 1):
        length += distL2FromLngLat(t[i + 1], t[i])
    return length

def distErp(t0, t1, g):
    # Edit distance with real penalty (ERP) between two GNSS trajectories
    n0 = len(t0)
    n1 = len(t1)
    C = [[0 for _ in range(n1 + 1)] for _ in range(n0 + 1)]

    gt0Dist = [distL2FromLngLat(g, x) for x in t0]
    gt1Dist = [distL2FromLngLat(g, x) for x in t1]
    mdist = mdistL2FromLngLat(t0, t1)

    sumgt0Dist = sum(gt0Dist)
    sumgt1Dist = sum(gt1Dist)
    for i in range(1, n0 + 1):
        C[i][0] = sumgt0Dist
    for j in range(1, n1 + 1):
        C[0][j] = sumgt1Dist
    for i in range(1, n0 + 1):
        for j in range(1, n1 + 1):
            derp0 = C[i - 1][j] + gt0Dist[i - 1]
            derp1 = C[i][j - 1] + gt1Dist[j - 1]
            derp01 = C[i - 1][j - 1] + mdist[i - 1][j - 1]
            C[i][j] = min(derp0, derp1, derp01)
    erp = C[n0][n1]
    return erp

def distDtw(t0, t1):
    # Dynamic time wrapping distance (DTW) between two GNSS trajectories
    n0 = len(t0)
    n1 = len(t1)
    C = [[0 for _ in range(n1 + 1)] for _ in range(n0 + 1)]
    for i in range(1, n0 + 1):
        C[i][0] = float('inf')
    for j in range(1, n1 + 1):
        C[0][j] = float('inf')
    for i in range(1, n0 + 1):
        for j in range(1, n1 + 1):
            C[i][j] = (
                distL2FromLngLat(t0[i - 1], t1[j - 1])
                + min(C[i][j - 1], C[i - 1][j - 1], C[i - 1][j])
            )
    dtw = C[n0][n1]
    return dtw

def minimumDistance(source, target):
    # Minimum distance (smallest point-to-line distance) between a source and a target GNSS trajectory
    totalError = 0 if len(source) > 0 else float('inf')
    for p in source:
        minDistance = float('inf')
        for i in range(len(target) - 1):
            distance = distPointToSegmentFromLngLat(p, target[i], target[i + 1])
            if distance < minDistance:
                minDistance = distance
        totalError += minDistance
    return totalError