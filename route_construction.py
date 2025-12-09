""" vevwevqewv"""
def dijkstra(adjacency_matrix, start_vertice):
    """
    Implements Dijkstra's algorithm to find the shortest paths from a start vertice
    to all other nodes in a graph represented by an adjacency matrix.

    This implementation assumes a dense graph where 0 indicates no direct
    connection between distinct vertices.

    Args:
        adjacency_matrix (List[List[int | float]]): A square matrix (NxN) where
            adjacency_matrix[u][v] represents the weight of the edge from u to v.
            - 0 indicates no edge (unless u == v).
            - Weights must be non-negative.
        start_vertice (int): The index of the starting vertice (0-indexed).

    Returns:
        tuple: A tuple containing two lists:
            1. distances (List[float]): The shortest distance from start_vertice to every
               other vertice. Unreachable nodes are marked with infinity float('inf').
            2. predecessors (List[int | None]): A list of parent nodes used to
               reconstruct the shortest path. None if the node is the start node
               or unreachable.

    Examples:
        >>> # Setup: A simple graph with 3 nodes
        >>> # 0 -> 1 (cost 4)
        >>> # 0 -> 2 (cost 2)
        >>> # 2 -> 1 (cost 1) -> Total path 0->2->1 is cost 3 (better than direct 4)
        >>> matrix = [
        ...     [0, 4, 2],
        ...     [0, 0, 0],  # Node 1 has no outgoing edges
        ...     [0, 1, 0]   # Node 2 goes to 1 with cost 1
        ... ]
        >>> dists, preds = dijkstra(matrix, 0)

        >>> # Check distances
        >>> dists[0] == 0
        True
        >>> dists[2] == 2
        True
        >>> dists[1]  # Should be 3 (via node 2), not 4
        3

        >>> # Check path reconstruction logic
        >>> preds[1] == 2 # We arrived at 1 from 2
        True
        >>> preds[2] == 0 # We arrived at 2 from 0
        True

        >>> # Case: Disconnected graph
        >>> # 0 <-> 1, but 2 is isolated
        >>> matrix_iso = [
        ...     [0, 10, 0],
        ...     [10, 0, 0],
        ...     [0, 0, 0]
        ... ]
        >>> dists_iso, preds_iso = dijkstra(matrix_iso, 0)
        >>> dists_iso[2]
        inf
        >>> preds_iso[2] is None
        True

        >>> # Case: Single node
        >>> matrix_single = [[0]]
        >>> dijkstra(matrix_single, 0)
        ([0], [None])

    Complexity:
        Time: O(V^2), where V is the number of vertices.
        Space: O(V) for storing distances and predecessors.
    """
    num_vertices = len(adjacency_matrix)

    # 1. Ініціалізація
    # Відстані до всіх вершин — нескінченність, крім стартової
    distances = [float('inf')] * num_vertices
    distances[start_vertice] = 0

    # Масив для відновлення шляху (хто був попереднім кроком)
    predecessors = [None] * num_vertices

    # Масив відвіданих вершин
    visited = [False] * num_vertices

    for _ in range(num_vertices):
        # 2. Знаходимо невідвідану вершину з мінімальною відстанню (Greedy step)
        min_dist = float('inf')
        u = -1

        for i in range(num_vertices):
            if not visited[i] and distances[i] < min_dist:
                min_dist = distances[i]
                u = i

        # Якщо не знайшли досяжну вершину або всі досяжні вже відвідані
        if u == -1 or distances[u] == float('inf'):
            break

        # Позначаємо вершину як оброблену
        visited[u] = True

        # 3. Релаксація ребер (оновлення сусідів)
        for v in range(num_vertices):
            weight = adjacency_matrix[u][v]

            # Перевіряємо, чи є ребро (weight > 0) і чи вершина v ще не відвідана
            if weight > 0 and not visited[v]:
                new_dist = distances[u] + weight

                # Якщо знайшли коротший шлях
                if new_dist < distances[v]:
                    distances[v] = new_dist
                    predecessors[v] = u  # Запам'ятовуємо, що прийшли з u

    return distances, predecessors







def build_distance_matrix(adjancency_matrix, points):
    """
    Construct a matrix of shortest distances between points

    Args:
    adjacency_matrix: adjacency matrix of the graph
    points: list of point indices (depot + delivery points)

    Returns:
    dict: distance matrix {point_a: {point_b: distance}}
    dist_matrix = {
    0: {0: 0, 1: 10, 2: 15},
    1: {0: 10, 1: 0, 2: 35},
    2: {0: 15, 1: 35, 2: 0}
    }
    """
    distance_matrix = {}
    for point_a in points:
        distances, _ = dijkstra(adjancency_matrix, point_a)
        distance_matrix[point_a] = {}
        for point_b in points:
            distance_matrix[point_a][point_b] = distances[point_b]

    return distance_matrix


def greedy_tsp_route(depot, delivery_points, distance_matrix):
    """
    Greedy Route Construction (Nearest Neighbor)

    Args:
    depot: depot index
    delivery_points: list of delivery point indices for this truck
    distance_matrix: distance matrix

    Returns:
    list: route (sequence of vertex indices)
    """
    route = [depot]
    unvisited = set(delivery_points)
    current = depot

    while unvisited:
        nearest = None
        min_distance = float('inf')
        for point in unvisited:
            distance = distance_matrix[current][point]
            if distance < min_distance:
                min_distance = distance
                nearest = point

        route.append(nearest)
        unvisited.remove(nearest)
        current = nearest

    route.append(depot)
    return route

def calculate_route_cost(route, distance_matrix):
    """
    Calculate the total cost of a route

    Args:
    route: list of vertices in order of visit
    distance_matrix: distance matrix

    Returns:
    float: total distance of the route
    """
    total_distances = 0
    for i in range(len(route) - 1):
        from_node = route[i]
        to_node = route[i + 1]
        total_distances += distance_matrix[from_node][to_node]
    return total_distances

def construct_route_for_truck(adjancency_matrix, depot, delivery_points, use_2opt=True):
    """
    main function: build a route for a single truck

    Args:
    adjacency_matrix: adjacency matrix of the graph
    depot: depot index
    delivery_points: list of point indices for this truck

    Returns:
    dict: route information
    """
    all_points = [depot] + delivery_points
    distance_matrix = build_distance_matrix(adjancency_matrix, all_points)
    route = greedy_tsp_route(depot, delivery_points, distance_matrix)
    initial_cost = calculate_route_cost(route, distance_matrix)

    print(f"Жадібний маршрут: {route}")
    print(f"Жадібна вартість: {initial_cost}")

    # КРОК 2: Покращити за допомогою 2-opt (опціонально)
    if use_2opt:
        print("\nЗастосування 2-opt покращення...")
        route = two_opt_improve(route, distance_matrix)

    # КРОК 3: Обчислити фінальну вартість
    final_cost = calculate_route_cost(route, distance_matrix)
    improvement = initial_cost - final_cost

    print(f"\nФінальний маршрут: {route}")
    print(f"Фінальна вартість: {final_cost}")
    if use_2opt:
        print(f"Покращення: {improvement} ({improvement/initial_cost*100:.1f}%)")

    return {
        'route': route,
        'total_distance': final_cost,
        'num_deliveries': len(delivery_points),
        'initial_distance': initial_cost if use_2opt else None,
        'improvement': improvement if use_2opt else None
    }

def two_opt_swap(route, i, j):
    """
     a 2-opt flip: cuts the route at positions i and j,
    flips the middle segment.

    Args:
    route: list of route vertices
    i, j: indices to cut (i < j)

    Returns:
    new route after 2-opt

    Example:
    route = [0, 1, 2, 3, 4, 0]
    i = 1, j = 3

    Segments:
    - start: route[0:i+1] = [0, 1]
    - middle: route[i+1:j+1] = [2, 3] ← FLIP
    - end: route[j+1:] = [4, 0]

    Result: [0, 1] + [3, 2] + [4, 0] = [0, 1, 3, 2, 4, 0]
    """
    new_route = route[0: i + 1]
    new_route += route[i + 1:j + 1][::-1]
    new_route += route[j + 1:]

    return new_route

def calculate_2opt_delta(route, i, j, distance_matrix):
    """
    Calculates the cost change when executing 2-opt
    WITHOUT actually creating a new route (faster!)

    Args:
    route: current route
    i, j: indices to slice
    distance_matrix: distance matrix

    Returns:
    delta: cost change (negative = improvement)

    Explanation:
    Old edges: (route[i] → route[i+1]) and (route[j] → route[j+1])
    New edges: (route[i] → route[j]) and (route[i+1] → route[j+1])
    """
    A = route[i]      # перша точка першого ребра
    B = route[i+1]    # друга точка першого ребра
    C = route[j]      # перша точка другого ребра
    D = route[j+1]    # друга точка другого ребра

    old_distance = (distance_matrix[A][B] +     # A → B
                   distance_matrix[C][D])       # C → D

    new_distance = (distance_matrix[A][C] +     # A → C
                   distance_matrix[B][D])       # B → D

    # зміна вартості
    delta = new_distance - old_distance
    return delta

def two_opt_improve(route, distance_matrix, max_iterations=1000):
    """
    Improves route using 2-opt algorithm

    Args:
    route: initial route
    distance_matrix: distance matrix
    max_iterations: maximum number of iterations (prevents hangs)

    Returns:
    improved route

    Logic:
    1. Check all possible pairs of indices (i, j)
    2. If improvement found → apply and start again
    3. If no improvement found → done!Docstring for two_opt_improve

    :param route: Description
    :param distance_matrix: Description
    :param max_iterations: Description
    """
    improved = True
    best_route = route[:]
    iteration = 0

    current_cost = calculate_route_cost(best_route, distance_matrix)
    print(f'Початкова вартість: {current_cost}')

    while improved and iteration < max_iterations:
        improved = False
        iteration += 1

        for i in range(1, len(best_route) - 2):
            for j in range(i + 1, len(best_route) - 1):
                delta = calculate_2opt_delta(best_route, i, j, distance_matrix)

                if delta < 0:
                    best_route = two_opt_swap(best_route, i, j)
                    current_cost += delta

                    print(f'Ітерація {iteration}: покращення на {-delta:.2f}, '
                          f'нова вартість: {current_cost:.2f}')
                    improved = True
                    break
            if improved:
                break

    print(f"Фінальна вартість після 2-opt: {current_cost}")
    print(f"Всього ітерацій: {iteration}")

    return best_route
