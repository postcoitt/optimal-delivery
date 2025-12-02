# optimal-delivery
The Vehicle Routing Problem (VRP) is one of the central problems in operations research, logistics, and combinatorial optimization. It generalizes many routing and scheduling problems and is crucial for industries such as transportation, delivery services, and supply chain management.

The task concerns the optimization of logistic operations in a transportation system. A set of delivery vehicles must service a set of customer demands, each associated with a specific location and weight. The underlying transportation network is modeled as an undirected weighted graph. The objective is to construct feasible and efficient routes for all vehicles while minimizing total transportation cost (e.g., distance or fuel consumption).

## Problem Statement
The transportation network is represented as a weighted undirected graph:
- **Nodes**: depots points
- **Edges**: roads with associated distances
- **Vehicles**: each with a capacity constraint

## Goal:
Find a set of routes that:
- Start and end at the depot
- Respect vehicle capacities
- Minimize total routing cost

## Algorithms implemented:
### Dijkstra Algorithm
Dijkstra's Algorithm allows us to find the shortest path between two vertices in a graph

## Team
**[Markhevka Oleksandr](https://github.com/OrGreeM)**<br>
↳ Dijkstra Algorithm. <br>
↳ Calculating the shortest distances between relevant vertices (depot and all delivery points) and corresponding paths between relevant vertices<br>

**[Ryshko Danylo](https://github.com/postcoitt)**<br>
↳ Assignment: alternately assigning each load to the truck where it is logical to "add", according to a greedy criterion.<br>

**[Kopiy Viktoria](https://github.com/gardenviki-prog)**<br>
↳ Route construction for each truck: given a set of points for the car, constructing the route (TSP subtask) greedily.<br>

**[Zavada Sofiia](https://github.com/Zavada-Sofiia)**<br>
↳ Working with graphs, .dot, visualization.<br>
↳ README <br>

**[Shymanska Natalia](https://github.com/natalish28)**<br>
↳ Tester - all doctests, functionality for comparing exact and greedy algorithms, keeping statistics on this.<br>

**[Kremer Sofiia](https://github.com/kremeriatko8)**<br>
↳ Documentation, presentation<br>
