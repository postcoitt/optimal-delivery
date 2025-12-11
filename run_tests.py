def run_tests():
    print("=== TEST 1: dijkstra simple graph ===")
    matrix = [
        [0, 4, 2],
        [0, 0, 0],
        [0, 1, 0]
    ]
    d, p = dijkstra(matrix, 0)
    assert d == [0, 3, 2], f"Expected [0,3,2], got {d}"
    assert p[1] == 2 and p[2] == 0

    print("=== TEST 2: get_path ===")
    preds = [None, 2, 0]
    assert get_path(preds, 1) == [0, 2, 1]

    print("=== TEST 3: build_distance_matrix ===")
    points = [0, 1, 2]
    dm = build_distance_matrix(matrix, points)
    assert dm[0][1] == 3
    assert dm[0][2] == 2

    print("=== TEST 4: greedy_tsp_route ===")
    route = greedy_tsp_route(0, [2, 1], dm)
    assert route[0] == 0 and route[-1] == 0
    assert set(route[1:-1]) == {1, 2}

    print("=== TEST 5: calculate_route_cost ===")
    cost = calculate_route_cost([0,2,1,0], dm)
    assert cost == dm[0][2] + dm[2][1] + dm[1][0]

    print("=== TEST 6: two_opt_swap ===")
    r = [0,1,2,3,0]
    r2 = two_opt_swap(r, 1, 3)
    assert r2 == [0,1,3,2,0]

    print("=== TEST 7: calculate_2opt_delta ===")
    delta = calculate_2opt_delta(r, 1, 3, dm)
    # just ensure it returns a number
    assert isinstance(delta, float)

    print("=== TEST 8: two_opt_improve quick test ===")
    simple_dm = {
        0: {0:0,1:10,2:5},
        1: {0:10,1:0,2:3},
        2: {0:5,1:3,2:0},
    }
    initial_route = [0,1,2,0]
    improved = two_opt_improve(initial_route, simple_dm)
    assert improved[0] == 0 and improved[-1] == 0

    print("=== TEST 9: assign_packages_greedy ===")
    trucks = [{"id":1,"capacity":10},{"id":2,"capacity":8}]
    packages = [
        {"id":101,"weight":6,"dest":1},
        {"id":102,"weight":4,"dest":2},
        {"id":103,"weight":9,"dest":1}, # too heavy for truck 2
    ]
    dist_m = {
        0:{0:0,1:5,2:8},
        1:{0:5,1:0,2:3},
        2:{0:8,1:3,2:0},
    }
    a, un = assign_packages_greedy(trucks, packages, dist_m, 0)
    assert un == [103]
    assert set(a[1] + a[2]) == {101, 102}

    print("=== TEST 10: construct_route_for_truck ===")
    adj = [
        [0,5,8],
        [5,0,3],
        [8,3,0]
    ]
    result = construct_route_for_truck(adj, 0, [1,2], use_2opt=False)
    assert "route" in result
    assert result["total_distance"] > 0

    print("\n ALL TESTS PASSED SUCCESSFULLY!")


if __name__ == "__main__":
    run_tests()
