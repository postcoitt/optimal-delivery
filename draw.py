import random
import math
import matplotlib.pyplot as plt
import time
from typing import Dict, List, Tuple, Set, Union, Any

def generate_random_graph(n_nodes: int, min_weight: int = 1, max_weight: int = 20) -> Dict[int, Dict[int, int]]:
    graph = {i: {} for i in range(n_nodes)}
    for i in range(n_nodes):
        for j in range(i + 1, n_nodes):
            w = random.randint(min_weight, max_weight)
            graph[i][j] = w
            graph[j][i] = w
    return graph

def dijkstra(adj_matrix: Dict[int, Dict[int, int]], start: int) -> Dict[int, float]:
    distances = {node: float('inf') for node in adj_matrix}
    distances[start] = 0
    queue = [(0, start)]
    while queue:
        current_dist, current_node = min(queue, key=lambda x: x[0])
        queue.remove((current_dist, current_node))
        if current_dist > distances[current_node]:
            continue
        for neighbor, weight in adj_matrix[current_node].items():
            distance = current_dist + weight
            if distance < distances[neighbor]:
                distances[neighbor] = distance
                queue.append((distance, neighbor))
    return distances

def build_distance_matrix(adj_matrix: Dict[int, Dict[int, int]], points: List[int]) -> Dict[int, Dict[int, float]]:
    dist_matrix = {}
    for a in points:
        distances = dijkstra(adj_matrix, a)
        dist_matrix[a] = {b: distances.get(b, math.inf) for b in points}
    return dist_matrix

def greedy_tsp_route(depot: int, delivery_points: List[int], distance_matrix: Dict[int, Dict[int, float]]) -> List[int]:
    route = [depot]
    unvisited: Set[int] = set(delivery_points)
    current = depot
    while unvisited:
        nearest = min(unvisited, key=lambda point: distance_matrix[current][point])
        route.append(nearest)
        unvisited.remove(nearest)
        current = nearest
    route.append(depot)
    return route

def calculate_route_cost(route: List[int], distance_matrix: Dict[int, Dict[int, float]]) -> float:
    return sum(distance_matrix[route[i]][route[i + 1]] for i in range(len(route) - 1))

def two_opt_swap(route: List[int], i: int, j: int) -> List[int]:
    return route[:i + 1] + route[i + 1:j + 1][::-1] + route[j + 1:]

def calculate_2opt_delta(route: List[int], i: int, j: int, dist_matrix: Dict[int, Dict[int, float]]) -> float:
    A, B = route[i], route[i + 1]
    C, D = route[j], route[j + 1]
    return (dist_matrix[A][C] + dist_matrix[B][D]) - (dist_matrix[A][B] + dist_matrix[C][D])

def two_opt_improve(route: List[int], dist_matrix: Dict[int, Dict[int, float]], max_iters: int = 1000) -> List[int]:
    best_route = route[:]
    improved = True
    iters = 0
    n = len(best_route)
    while improved and iters < max_iters:
        improved = False
        iters += 1
        for i in range(n - 3):
            for j in range(i + 2, n - 1):
                delta = calculate_2opt_delta(best_route, i, j, dist_matrix)
                if delta < -1e-9:
                    best_route = two_opt_swap(best_route, i, j)
                    improved = True
                    break
            if improved: break
    return best_route

def generate_all_routes(depot: int, dist_matrix: Dict[int, Dict[int, float]], current_route: List[int], remaining: List[int], best_sol: List[Union[float, List[int]]]):
    current_cost = sum(dist_matrix[current_route[i]][current_route[i + 1]] for i in range(len(current_route) - 1))
    if current_cost >= best_sol[0]:
        return
    if not remaining:
        total_cost = current_cost + dist_matrix[current_route[-1]][depot]
        if total_cost < best_sol[0]:
            best_sol[0] = total_cost
            best_sol[1] = current_route + [depot]
        return
    for i, next_point in enumerate(remaining):
        new_remaining = remaining[:i] + remaining[i + 1:]
        generate_all_routes(depot, dist_matrix, current_route + [next_point], new_remaining, best_sol)

def exact_tsp(depot: int, delivery_points: List[int], dist_matrix: Dict[int, Dict[int, float]]) -> Tuple[List[int], float]:
    best_sol: List[Union[float, List[int]]] = [float('inf'), []]
    generate_all_routes(depot, dist_matrix, [depot], delivery_points, best_sol)
    return best_sol[1], best_sol[0] # type: ignore

def benchmark_all(max_nodes: int = 10, n_trials: int = 5) -> Tuple[List[int], Dict[str, List[float]], Dict[str, List[float]]]:
    n_list = list(range(3, max_nodes + 1))
    time_results: Dict[str, List[float]] = {'Greedy': [], 'Greedy+2opt': [], 'Exact': []}
    gap_results: Dict[str, List[float]] = {'Greedy': [], 'Greedy+2opt': []}
    for n in n_list:
        time_g, time_g2, time_e = [], [], []
        cost_g_trials, cost_g2_trials, cost_e_trials = [], [], []
        for _ in range(n_trials):
            graph = generate_random_graph(n)
            nodes = list(graph.keys())
            depot = 0
            delivery_points = nodes[1:]
            dist_matrix = build_distance_matrix(graph, nodes)
            t0 = time.time()
            route_e, cost_e = exact_tsp(depot, delivery_points, dist_matrix)
            t1 = time.time()
            time_e.append(t1 - t0)
            cost_e_trials.append(cost_e)
            t0 = time.time()
            route_g = greedy_tsp_route(depot, delivery_points, dist_matrix)
            t1 = time.time()
            cost_g = calculate_route_cost(route_g, dist_matrix)
            time_g.append(t1 - t0)
            cost_g_trials.append(cost_g)
            route_g_copy = route_g[:]
            t0 = time.time()
            route_g2 = two_opt_improve(route_g_copy, dist_matrix)
            t1 = time.time()
            cost_g2 = calculate_route_cost(route_g2, dist_matrix)
            time_g2.append(t1 - t0)
            cost_g2_trials.append(cost_g2)
        time_results['Greedy'].append(sum(time_g) / len(time_g))
        time_results['Greedy+2opt'].append(sum(time_g2) / len(time_g2))
        time_results['Exact'].append(sum(time_e) / len(time_e))
        avg_cost_e = sum(cost_e_trials) / len(cost_e_trials)
        avg_cost_g = sum(cost_g_trials) / len(cost_g_trials)
        avg_cost_g2 = sum(cost_g2_trials) / len(cost_g2_trials)
        if avg_cost_e > 0:
            avg_gap_g = 100 * (avg_cost_g - avg_cost_e) / avg_cost_e
            avg_gap_g2 = 100 * (avg_cost_g2 - avg_cost_e) / avg_cost_e
        else:
            avg_gap_g = 0.0
            avg_gap_g2 = 0.0
        gap_results['Greedy'].append(avg_gap_g)
        gap_results['Greedy+2opt'].append(avg_gap_g2)
    return n_list, time_results, gap_results

if __name__ == '__main__':
    MAX_NODES = 10
    N_TRIALS = 255
    N_LIST, TIME_RES, GAP_RES = benchmark_all(max_nodes=MAX_NODES, n_trials=N_TRIALS)
    plt.figure(figsize=(10, 6))
    plt.plot(N_LIST, TIME_RES['Greedy'], marker='o', label='Greedy', color='blue')
    plt.plot(N_LIST, TIME_RES['Greedy+2opt'], marker='s', label='Greedy + 2-opt', color='orange')
    plt.plot(N_LIST, TIME_RES['Exact'], marker='^', label='Exact', color='red')
    plt.yscale('log')
    plt.xlabel('Кількість вершин (n)')
    plt.ylabel('Середній час виконання (сек) [Log-шкала]')
    plt.title('Порівняння часу виконання трьох алгоритмів')
    plt.xticks(N_LIST)
    plt.grid(True, which='both', linestyle='--', alpha=0.7)
    plt.legend(loc='upper left')
    plt.show()
    plt.figure(figsize=(10, 6))
    plt.plot(N_LIST, GAP_RES['Greedy'], marker='o', color='red', label='Gap Greedy')
    plt.plot(N_LIST, GAP_RES['Greedy+2opt'], marker='s', color='green', label='Gap Greedy + 2-opt')
    plt.xlabel('Кількість вершин (n)')
    plt.ylabel('Середня похибка відносно Exact (%)')
    plt.title('Похибка алгоритмів Greedy та Greedy+2opt відносно Exact')
    plt.xticks(N_LIST)
    plt.grid(True, which='both', linestyle='--', alpha=0.7)
    plt.legend(loc='upper left')
    plt.show()
