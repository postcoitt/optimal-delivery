from algorithm.route_construction_greedy_and_accurate import *
import json

def load_json(path: str):
    """Load JSON with error handling."""
    try:
        with open(path, "r") as f:
            return json.load(f)
    except FileNotFoundError:
        print(f"File not found: {path}")
        exit(1)
    except json.JSONDecodeError:
        print(f"Invalid JSON in file: {path}")
        exit(1)


def normalize_graph(graph_raw: list) -> dict[int, dict[int, float]]:
    """
    Convert an adjacency matrix into an adjacency dictionary.

    Parameters:
        graph_raw (list[list[float]]): A square adjacency matrix where
            0 or None indicates no edge, and any other value represents the edge weight.

    Returns:
        dict[int, dict[int, float]]: An adjacency dictionary of the form
            {node: {neighbor: weight, ...}, ...}.

    Example:
        [[0, 3, 2],
         [3, 0, 1],
         [2, 1, 0]]
        -> {0: {1: 3, 2: 2}, 1: {0: 3, 2: 1}, 2: {0: 2, 1: 1}}
    """

    # Check if input is a proper list of lists
    if not isinstance(graph_raw, list) or not all(isinstance(row, list) for row in graph_raw):
        raise ValueError("Input must be a list of lists representing an adjacency matrix.")

    n = len(graph_raw)
    adj = {i: {} for i in range(n)}

    # Convert matrix to adjacency dict
    for i in range(n):
        for j in range(n):
            w = graph_raw[i][j]
            # 0 or None indicates no edge
            if w not in [0, None]:
                adj[i][j] = float(w)

    return adj



def get_path_safe(predecessors: Dict[int, int], start: int, end: int) -> List[int]:
    """
    Reconstructs the shortest path from `start` to `end` using a predecessors map

    Args:
        predecessors (Dict[int, int]):
            A mapping from each node to its predecessor on the shortest path from the start node
            Example: {1: 0, 2: 1, 3: 2, ...}

        start (int):
            The starting node ID

        end (int):
            The target node ID

    Returns:
        List[int]:
            A list of node IDs representing the path from `start` to `end`, including both endpoints.ґ
            If no path exists, returns [start, end]
    """
    path = [end]
    while path[-1] != start:
        if predecessors[path[-1]] is None:
            return [start, end]
        path.append(predecessors[path[-1]])
    path.reverse()
    return path


def plot_truck_routes(graph: Dict[int, Dict[int, float]],
                      depot: int,
                      assignments: Dict[int, List[int]],
                      packages: List[Dict[str, Any]],
                      show_gui: bool = True):
    """
    Visualize and/or print delivery routes for multiple trucks on a weighted graph.

    For each truck, the function reconstructs the full route including intermediate nodes
    using Dijkstra's algorithm, annotates edge weights along the path, and prints the
    full route in the console. If `show_gui` is True, it generates a separate plot for
    each truck showing the route over the graph.

    Args:
        graph (Dict[int, Dict[int, float]]):
            Adjacency dictionary representing the weighted undirected graph.
            Example: {0: {1: 3, 2: 2}, 1: {0: 3, 2: 1}, ...}

        depot (int):
            Node ID representing the depot from which trucks start and end their routes.

        assignments (Dict[int, List[int]]):
            Mapping of truck IDs to lists of package IDs assigned to each truck.
            Example: {0: [101, 102], 1: [103]}

        packages (List[Dict[str, Any]]):
            List of package dictionaries, each containing at least 'id' and 'dest' keys.
            Example: [{"id": 101, "dest": 2}, {"id": 102, "dest": 3}]

        show_gui (bool, optional):
            Whether to display graphical plots for each truck route. Default is True.
    """
    coords = {}
    n_nodes = len(graph)
    radius = 1.0
    #кути між вершинами
    for i, node in enumerate(graph.keys()):
        angle = 2 * math.pi * i / n_nodes
        coords[node] = (radius * math.cos(angle), radius * math.sin(angle))

    pkg_map = {p["id"]: p for p in packages}
    colors = ['red', 'green', 'orange', 'purple', 'cyan', 'magenta', 'brown', 'pink', 'olive', 'navy']

    for idx, (truck_id, pkg_ids) in enumerate(assignments.items()):
        if not pkg_ids:
            print(f"Truck {truck_id} is empty.")
            continue

        color = colors[idx % len(colors)]
        d_points = [pkg_map[pid]["dest"] for pid in pkg_ids]

        # --- get route ---
        route_res = construct_route_smart(graph, depot, d_points, algorithm='greedy', use_2opt=True)
        route = route_res['route']

        full_route = [route[0]]
        for i in range(len(route)-1):
            start, end = route[i], route[i+1]
            _, predecessors = dijkstra(graph, start)
            segment = get_path_safe(predecessors, start, end)
            full_route.extend(segment[1:])

        # --- output ---
        print(f"[Truck {truck_id}] full route: {full_route}")

        if show_gui:
            # --- For each truck new figure ---
            plt.figure(figsize=(8, 8))
            plt.clf()

            # --- draw edges in light gray ---
            for node, neighbors in graph.items():
                x1, y1 = coords[node]
                for neigh, w in neighbors.items():
                    if neigh < node:
                        continue
                    x2, y2 = coords[neigh]
                    plt.plot([x1, x2], [y1, y2], color='lightgray', linestyle='--', zorder=1)
                    mid_x, mid_y = (x1 + x2) / 2, (y1 + y2) / 2
                    plt.text(mid_x, mid_y, f"{w:.1f}", color='gray', fontsize=8, zorder=2, ha='center', va='center')

            # --- drawing vertices ---
            for node, (x, y) in coords.items():
                if node == depot:
                    plt.scatter(x, y, color='blue', s=150, zorder=5)
                    plt.text(x, y+0.05, f"Depot {node}", ha='center', fontsize=10, zorder=6)
                else:
                    plt.scatter(x, y, color='gray', s=100, zorder=5)
                    plt.text(x, y+0.05, f"{node}", ha='center', fontsize=8, zorder=6)

            # --- drawing route ---
            plt.plot([coords[n][0] for n in full_route],
                     [coords[n][1] for n in full_route],
                     color=color, linewidth=2, zorder=2, label=f'Truck {truck_id}')

            plt.scatter([coords[n][0] for n in full_route[1:-1]],
                        [coords[n][1] for n in full_route[1:-1]],
                        color=color, s=100, zorder=6)

            # --- full path on graph ---
            path_str = " -> ".join(map(str, full_route))
            plt.text(0, -1.2*radius, f"Full path: {path_str}", color='black', fontsize=9, ha='center')

            plt.title(f"Truck {truck_id} Delivery Route")
            plt.axis('equal')
            plt.axis('off')
            plt.legend()
            plt.show()
            plt.close()

def run_cli():
    parser = argparse.ArgumentParser(
        description="Delivery routing optimizer (graph + trucks + packages)"
    )

    parser.add_argument("--graph", required=True, help="Path to graph JSON file")
    parser.add_argument("--trucks", required=True, help="Path to trucks.json file")
    parser.add_argument("--packages", required=True, help="Path to packages.json file")
    parser.add_argument("--depot", type=int, default=0, help="Depot node id")
    parser.add_argument("--no-gui", action="store_true", help="Disable GUI plotting")

    args = parser.parse_args()

    # --- load input files ---
    graph_raw = load_json(args.graph)
    trucks = load_json(args.trucks)
    packages = load_json(args.packages)

    # --- normalize graph ---
    graph = normalize_graph(graph_raw)
    graph = {int(k): {int(n): w for n, w in v.items()} for k, v in graph.items()}

    depot = args.depot

    all_nodes = list(graph.keys())
    distance_matrix = build_distance_matrix(graph, all_nodes)

    assignments, unassigned = assign_packages_greedy(trucks, packages, distance_matrix, depot)

    print("\n=== PACKAGE ASSIGNMENT ===")
    print(assignments)
    print("Unassigned:", unassigned)

    # --- compare algorithms per truck ---
    pkg_map = {p["id"]: p for p in packages}
    for t_id, pkg_ids in assignments.items():
        if not pkg_ids:
            print(f"Truck {t_id} is empty.")
            continue

        d_points = [pkg_map[pid]["dest"] for pid in pkg_ids]
        print(f"\n[TRUCK {t_id}] Delivery Points: {d_points}")
        compare_algorithms(graph, depot, d_points)

    # --- plot truck routes (skip if --no-gui) ---
    plot_truck_routes(graph, depot, assignments, packages, show_gui=not args.no_gui)


if __name__ == "__main__":
    try:
        run_cli()
    except KeyboardInterrupt:
        print("Exiting...")
