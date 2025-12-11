import argparse
import json
import networkx as nx
import matplotlib.pyplot as plt
from algorithm.algorithm import *


def show_graph_with_path(adjacency_matrix, path=None, title="Graph", plot=True):
    """Display the graph with optional highlighted path."""
    if not plot:
        return

    G = nx.Graph()
    n = len(adjacency_matrix)

    for i in range(n):
        for j in range(n):
            w = adjacency_matrix[i][j]
            if w > 0:
                G.add_edge(i, j, weight=w)

    pos = nx.spring_layout(G, seed=42)

    # Draw base edges lightly
    nx.draw_networkx_edges(G, pos, width=1.5, alpha=0.3)

    # Highlight the route if provided
    if path and len(path) > 1:
        red_edges = list(zip(path, path[1:]))
        nx.draw_networkx_edges(G, pos, edgelist=red_edges, width=3, edge_color="red")

    nx.draw_networkx_nodes(G, pos, node_size=600, node_color="lightblue")
    nx.draw_networkx_labels(G, pos)

    edge_labels = {(u, v): G[u][v]["weight"] for u, v in G.edges()}
    nx.draw_networkx_edge_labels(G, pos, edge_labels=edge_labels)

    plt.title(title)
    plt.axis("off")
    plt.show()


def reconstruct_path(end, predecessors):
    """Reconstruct a path from the predecessors list."""
    path = []
    while end is not None:
        path.append(end)
        end = predecessors[end]
    return path[::-1]


def expand_route_via_dijkstra(route, adjacency_matrix):
    """
    Expand a high-level TSP route into the actual nodes along the shortest paths.

    Example:
        route = [0, 2, 5]
        adjacency_matrix = ...
        expanded = expand_route_via_dijkstra(route, adjacency_matrix)
    """
    full_path = []
    for start, end in zip(route, route[1:]):
        _, preds = dijkstra(adjacency_matrix, start)
        segment = reconstruct_path(end, preds)
        if full_path:
            segment = segment[1:]  # avoid duplicating nodes
        full_path.extend(segment)
    return full_path


def main():
    parser = argparse.ArgumentParser(description="VRP & Graph CLI Tool")
    sub = parser.add_subparsers(dest="command", required=True)

    # --- Show graph ---
    sg = sub.add_parser("show-graph", help="Display the graph")
    sg.add_argument("--matrix", required=True, help="Path to adjacency matrix JSON file")
    sg.add_argument("--route", help="Optional JSON route to highlight")
    sg.add_argument("--no-plot", action="store_true", help="Do not display the graph image")

    # --- Dijkstra ---
    dj = sub.add_parser("dijkstra", help="Run Dijkstra's algorithm and show shortest path")
    dj.add_argument("--matrix", required=True)
    dj.add_argument("--start", type=int, required=True)
    dj.add_argument("--end", type=int, required=True)
    dj.add_argument("--no-plot", action="store_true", help="Do not display the graph image")

    # --- Route ---
    rt = sub.add_parser("route", help="Build a route for a single truck")
    rt.add_argument("--matrix", required=True)
    rt.add_argument("--depot", type=int, required=True)
    rt.add_argument("--deliveries", required=True, help='JSON list of delivery nodes, e.g. "[3,5,7]"')
    rt.add_argument("--no-plot", action="store_true", help="Do not display the graph image")
    rt.add_argument("--number", type=int, default=-1, help="Serial number of the truck")

    args = parser.parse_args()

    # Load adjacency matrix
    try:
        with open(args.matrix, "r", encoding="utf-8") as f:
            matrix = json.load(f)
    except FileNotFoundError:
        print(f"Cannot find {args.matrix}")
        return

    # Handle commands
    if args.command == "show-graph":
        route = json.loads(args.route) if args.route else None
        show_graph_with_path(matrix, route, title="Graph", plot=not args.no_plot)
        return

    elif args.command == "dijkstra":
        distances, preds = dijkstra(matrix, args.start)
        path = reconstruct_path(args.end, preds)
        print(f"Найкоротша: {distances[args.end]}")
        print(f"Шлях: {path}")
        show_graph_with_path(matrix, path, title="Дейкстрів шлях", plot=not args.no_plot)
        return

    elif args.command == "route":
        try:
            with open(args.deliveries, "r", encoding="utf-8") as f:
                deliveries = json.load(f)
        except FileNotFoundError:
            print(f"Cannot find {args.deliveries}")
            return

        args.number -= 1

        if args.number < 0:
            print("Invalid number")
            return
        if args.number >= len(deliveries):
            print(f"Wrong number of the truck (choose from 1 to {len(deliveries)})")
            return

        deliveries = deliveries[args.number]
        all_nodes = [args.depot] + deliveries
        dist_matrix = build_distance_matrix(matrix, all_nodes)

        route = greedy_tsp_route(args.depot, deliveries, dist_matrix)
        route = two_opt_improve(route, dist_matrix)
        cost = calculate_route_cost(route, dist_matrix)

        print("Path:", route)
        print("Weight:", cost)

        expanded_route = expand_route_via_dijkstra(route, matrix)
        show_graph_with_path(matrix, expanded_route, title=f"Path (weight: {cost})", plot=not args.no_plot)
        return


if __name__ == "__main__":
    main()

