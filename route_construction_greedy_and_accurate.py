"""
Vehicle Routing Problem (VRP) Solver — Shortest Paths + Load Assignment + Route Optimization

This module implements a simplified VRP solution pipeline consisting of three main stages:

1. Shortest-Path Preprocessing:
   Computes all-pairs shortest distances (and optionally paths) between the depot
   and all delivery points using Dijkstra's algorithm.

2. Load Assignment:
   Distributes packages among available trucks based on weight capacity.
   This acts as a constrained bin-packing step: each package is placed into a truck
   whose remaining capacity can accommodate it.

3. Route Construction and Optimization:
   For each truck, a greedy TSP heuristic builds an initial route over its assigned
   delivery points. The route is then improved using the 2-opt local-search algorithm,
   which iteratively replaces two edges with a better pair until no further improvement
   is possible.

The module outputs final per-truck routes, total distances, and improvement statistics.
"""

import math
from typing import List, Dict, Tuple, Any
import random
import time
import matplotlib.pyplot as plt


def dijkstra(
    adjacency_matrix: Dict[int, Dict[int, float]],
    start_node: int
) -> Tuple[Dict[int, float], None]:
    """
    Computes shortest paths from a start node to all other nodes using Dijkstra's algorithm.

    WARNING: This implementation uses a linear list scan instead of a Priority Queue.

    Args:
        adjacency_matrix (Dict[int, Dict[int, float]]): The graph representation where keys
            are node IDs and values are dictionaries of {neighbor_id: edge_weight}.
        start_node (int): The ID of the starting node.

    Returns:
        Tuple[Dict[int, float], None]:
            - A dictionary mapping node IDs to their shortest distance from start_node.
            - None (placeholder for path predecessors, not used here).

    Time Complexity:
        O(V^2) - due to linear search for the minimum element in the queue.
    """
    # Initialize distances to infinity
    distances = {node: float('inf') for node in adjacency_matrix}
    distances[start_node] = 0

    # Queue stores tuples: (distance, node_id)
    # Using a standard list acts as an unsorted priority queue.
    queue = [(0, start_node)]

    while queue:
        # 1. Linear Search for minimum distance node: O(N) operation
        current_dist, current_node = min(queue, key=lambda x: x[0])

        # 2. Remove the processed node
        queue.remove((current_dist, current_node))

        # Optimization: Skip if we found a shorter path previously
        if current_dist > distances[current_node]:
            continue

        # Explore neighbors
        for neighbor, weight in adjacency_matrix[current_node].items():
            distance = current_dist + weight

            # Relaxation step
            if distance < distances[neighbor]:
                distances[neighbor] = distance
                queue.append((distance, neighbor))

    return distances, None


def build_distance_matrix(
    adjacency_matrix: Dict[int, Dict[int, float]],
    points: List[int]
) -> Dict[int, Dict[int, float]]:
    """
    Constructs a dense distance matrix for a subset of points using Dijkstra.

    Args:
        adjacency_matrix: The full graph.
        points: List of specific node IDs (e.g., depot + delivery targets) to include.

    Returns:
        Dict[int, Dict[int, float]]: A square matrix (dictionary of dictionaries)
        representing shortest path distances between all pairs of given points.
    """
    distance_matrix = {}
    for point_a in points:
        distances, _ = dijkstra(adjacency_matrix, point_a)
        distance_matrix[point_a] = {}
        for point_b in points:
            # Use math.inf if a node is unreachable
            distance_matrix[point_a][point_b] = distances.get(point_b, math.inf)
    return distance_matrix


# 2. ASSIGNMENT / CLUSTERING (GREEDY)

def assign_packages_greedy(
    trucks: List[Dict[str, Any]],
    packages: List[Dict[str, Any]],
    distance_matrix: Dict[int, Dict[int, float]],
    depot_index: int
) -> Tuple[Dict[int, List[int]], List[int]]:
    """
    Distributes packages among trucks using a greedy heuristic considering
    capacity and distance.

    Logic:
    1. Sort packages by weight (Descending) -> Fit biggest items first.
    2. Sort trucks by remaining capacity (Descending).
    3. Assign package to the truck that minimizes travel distance from its *current* position.

    Args:
        trucks: List of dicts, must contain 'id' and 'capacity'.
        packages: List of dicts, must contain 'id', 'weight', 'dest'.
        distance_matrix: Precomputed distances between nodes.
        depot_index: Initial position of all trucks.

    Returns:
        Tuple containing:
            - Assignments: {truck_id: [list of package_ids]}
            - Unassigned: List of package_ids that didn't fit.
    """
    # Initialize truck dynamic state
    for t in trucks:
        t["remaining"] = t["capacity"]
        t["current_pos"] = depot_index

    # Sort packages: Heaviest first
    remaining_packages = sorted(packages, key=lambda p: (-p["weight"], p["id"]))

    assignments = {t["id"]: [] for t in trucks}

    progress = True
    while remaining_packages and progress:
        progress = False

        # Sort trucks: Prioritize empty trucks to balance load
        trucks.sort(key=lambda t: (-t["remaining"], t["id"]))

        for t in trucks:
            best_pkg = None
            # Metric tuple: (distance, -weight). Smaller is better.
            best_metric = (math.inf, math.inf)

            for pkg in remaining_packages:
                if pkg["weight"] <= t["remaining"]:
                    d = distance_matrix[t["current_pos"]][pkg["dest"]]
                    metric = (d, -pkg["weight"])

                    if metric < best_metric:
                        best_metric = metric
                        best_pkg = pkg

            if best_pkg:
                assignments[t["id"]].append(best_pkg["id"])
                t["remaining"] -= best_pkg["weight"]
                t["current_pos"] = best_pkg["dest"]
                remaining_packages.remove(best_pkg)
                progress = True

    unassigned = [p["id"] for p in remaining_packages]
    return assignments, unassigned


# 3. ROUTING (TSP: GREEDY + 2-OPT)

def calculate_route_cost(
    route: List[int],
    distance_matrix: Dict[int, Dict[int, float]]
) -> float:
    """Calculates the total travel distance of a sequential route."""
    total_dist = 0.0
    for i in range(len(route) - 1):
        total_dist += distance_matrix[route[i]][route[i+1]]
    return total_dist

def greedy_tsp_route(
    depot: int,
    delivery_points: List[int],
    distance_matrix: Dict[int, Dict[int, float]]
) -> List[int]:
    """
    Constructs an initial route using the Nearest Neighbor algorithm.

    Args:
        depot: Start/End node.
        delivery_points: Nodes to visit.
        distance_matrix: Cost lookup table.

    Returns:
        List[int]: An ordered list of node IDs representing the route.
    """
    route = [depot]
    unvisited = set(delivery_points)
    current = depot

    while unvisited:
        # Linear search for the nearest neighbor
        nearest = min(unvisited, key=lambda point: distance_matrix[current][point])
        route.append(nearest)
        unvisited.remove(nearest)
        current = nearest

    route.append(depot)
    return route

def two_opt_swap(route: List[int], i: int, j: int) -> List[int]:
    """Reverses the route segment between indices i and j (inclusive)."""
    return route[:i+1] + route[i+1:j+1][::-1] + route[j+1:]

def calculate_2opt_delta(
    route: List[int],
    i: int,
    j: int,
    distance_matrix: Dict[int, Dict[int, float]]
) -> float:
    """
    Calculates the cost impact of a 2-opt swap in O(1) time.

    Delta < 0 implies the swap reduces total distance.
    """
    A, B = route[i], route[i+1]
    C, D = route[j], route[j+1]
    # (New edges cost) - (Old edges cost)
    return (distance_matrix[A][C] + distance_matrix[B][D]) - \
           (distance_matrix[A][B] + distance_matrix[C][D])

def two_opt_improve(
    route: List[int],
    distance_matrix: Dict[int, Dict[int, float]],
    max_iters: int = 1000
) -> List[int]:
    """
    Refines a route using the 2-Opt local search algorithm.
    It eliminates self-intersections in the path.

    Args:
        route: Initial route.
        distance_matrix: Distance lookup.
        max_iters: Safety break to prevent infinite loops.

    Returns:
        List[int]: Optimized route.
    """
    improved = True
    iters = 0
    best_route = route[:]

    while improved and iters < max_iters:
        improved = False
        iters += 1
        for i in range(1, len(best_route) - 2):
            for j in range(i + 1, len(best_route) - 1):
                delta = calculate_2opt_delta(best_route, i, j, distance_matrix)
                if delta < -1e-9: # Float comparison tolerance
                    best_route = two_opt_swap(best_route, i, j)
                    improved = True
                    break # First Improvement strategy
            if improved: break

    return best_route


# 4. EXACT ALGORITHM (TSP: BRANCH & BOUND)

def generate_all_routes(
    depot: int,
    distance_matrix: Dict[int, Dict[int, float]],
    current_route: List[int],
    remaining: List[int],
    best_sol: List[Any]
) -> None:
    """
    Recursive DFS helper for Exact TSP with Pruning.

    Args:
        best_sol: Mutable list [min_cost, best_route] passed by reference.
    """
    current_cost = 0.0
    for i in range(len(current_route)-1):
        current_cost += distance_matrix[current_route[i]][current_route[i+1]]

    # Pruning: Stop if current path is already worse than best found
    if current_cost >= best_sol[0]:
        return

    # Base Case: Complete tour
    if not remaining:
        total_cost = current_cost + distance_matrix[current_route[-1]][depot]
        if total_cost < best_sol[0]:
            best_sol[0] = total_cost
            best_sol[1] = current_route + [depot]
        return

    # Recursive Step
    for i, next_point in enumerate(remaining):
        new_remaining = remaining[:i] + remaining[i+1:]
        generate_all_routes(depot, distance_matrix, current_route + [next_point], new_remaining, best_sol)

def exact_tsp(
    depot: int,
    delivery_points: List[int],
    distance_matrix: Dict[int, Dict[int, float]]
) -> Tuple[List[int], float]:
    """
    Solves TSP exactly.
    Complexity: O(N!) - Use only for N <= 10.
    """
    # best_sol = [cost, route]
    best_sol = [float('inf'), []]
    generate_all_routes(depot, distance_matrix, [depot], delivery_points, best_sol)
    return best_sol[1], best_sol[0]


# 5. SMART CONTROLLER & MAIN

def construct_route_smart(
    adjacency_matrix: Dict[int, Dict[int, float]],
    depot: int,
    delivery_points: List[int],
    algorithm: str = 'auto',
    use_2opt: bool = True
) -> Dict[str, Any]:
    """
    Orchestrator function that builds routes based on selected strategy.

    Args:
        algorithm: 'auto' (switches based on size), 'exact', or 'greedy'.
    """
    n = len(delivery_points)
    if algorithm == 'auto':
        chosen_algorithm = 'exact' if n <= 9 else 'greedy'
    else:
        chosen_algorithm = algorithm

    all_points = [depot] + delivery_points
    dist_matrix = build_distance_matrix(adjacency_matrix, all_points)

    # --- EXACT ---
    if chosen_algorithm == 'exact':
        if n > 10:
            print(f"⚠️ WARNING: {n} points is too many for exact algorithm. Switching to greedy.")
            chosen_algorithm = 'greedy'
        else:
            route, cost = exact_tsp(depot, delivery_points, dist_matrix)
            return {'route': route, 'total_distance': cost, 'algorithm': 'exact'}

    # --- GREEDY ---
    if chosen_algorithm == 'greedy':
        route = greedy_tsp_route(depot, delivery_points, dist_matrix)
        if use_2opt:
            route = two_opt_improve(route, dist_matrix)
        cost = calculate_route_cost(route, dist_matrix)
        return {'route': route, 'total_distance': cost, 'algorithm': 'greedy+2opt'}

    return {}
