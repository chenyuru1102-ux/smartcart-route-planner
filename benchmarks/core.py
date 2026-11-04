"""Independent headless solvers. Distances are schematic map units."""

import heapq, json, math, random
from pathlib import Path


def load_map():
    return json.loads(Path(__file__).with_name("map.json").read_text())


def astar(m, start, end):
    h = lambda u: math.dist(m["POS"][u], m["POS"][end])
    q = [(h(start), 0, start)]
    best = {start: 0}
    prev = {start: None}
    while q:
        _, g, u = heapq.heappop(q)
        if g > best[u] + 1e-10:
            continue
        if u == end:
            path = []
            while u is not None:
                path.append(u)
                u = prev[u]
            return g, path[::-1]
        for v, w in m["GRAPH"].get(u, []):
            ng = g + w
            if ng < best.get(v, math.inf) - 1e-10:
                best[v] = ng
                prev[v] = u
                heapq.heappush(q, (ng + h(v), ng, v))
    return math.inf, []


def dijkstra(m, start):
    # Independent O(V^2) oracle, with no heuristic or shared search implementation.
    dist = {u: math.inf for u in m["POS"]}
    dist[start] = 0
    todo = set(dist)
    while todo:
        u = min(todo, key=lambda v: (dist[v], v))
        todo.remove(u)
        if math.isinf(dist[u]):
            break
        for v, w in m["GRAPH"].get(u, []):
            dist[v] = min(dist[v], dist[u] + w)
    return dist


def generate_cases(m, seed=20261021):
    rng = random.Random(seed)
    shelves = sorted(m["SHELF_PICKUP_NODES"])
    out = []
    for n in (5, 8, 10):
        seen = set()
        while len(seen) < 30:
            ss = tuple(sorted(rng.sample(shelves, n)))
            if ss in seen:
                continue
            seen.add(ss)
            out.append(
                {
                    "id": f"A-{n}-{len(seen):02d}",
                    "size": n,
                    "shelves": list(ss),
                    "stops": [m["SHELF_PICKUP_NODES"][s][0] for s in ss],
                }
            )
    return out


def matrix(m, case):
    nodes = [
        m["STOP_TO_NODE"]["Entrance"],
        *case["stops"],
        m["STOP_TO_NODE"]["Parking"],
    ]
    return nodes, [[astar(m, a, b)[0] for b in nodes] for a in nodes]


def cost(route, d):
    return sum(d[a][b] for a, b in zip(route, route[1:]))


def greedy(d):
    route = [0]
    todo = set(range(1, len(d) - 1))
    while todo:
        v = min(todo, key=lambda j: (d[route[-1]][j], j))
        route.append(v)
        todo.remove(v)
    return route + [len(d) - 1]


def local(route, objective):
    route = list(route)
    val = objective(route)
    while True:
        best = route
        bv = val
        for i in range(1, len(route) - 2):
            for j in range(i + 1, len(route) - 1):
                r = route[:i] + route[i : j + 1][::-1] + route[j + 1 :]
                v = objective(r)
                if v < bv - 1e-9:
                    best, bv = r, v
        if bv >= val - 1e-9:
            return route
        route, val = best, bv


def penalty_update(route, d, p):
    edges = list(zip(route, route[1:]))
    utilities = [d[a][b] / (1 + p[a][b]) for a, b in edges]
    mx = max(utilities)
    for (a, b), u in zip(edges, utilities):
        if abs(u - mx) < 1e-9:
            p[a][b] += 1


def solve(method, d, seed=11, lam=0.35):
    n = len(d)
    evaluations = 0
    if any(not math.isfinite(x) for row in d for x in row):
        raise ValueError("Unreachable pair in fixed-stop matrix")

    def obj(r):
        nonlocal evaluations
        evaluations += 1
        return cost(r, d)

    iterations = 0
    status = "completed"
    if method == "greedy":
        r = greedy(d)
    elif method == "two_opt":
        r = local(greedy(d), obj)
    elif method == "dp":
        dp = {(0, 0): (0, [])}
        for mask in range(1 << (n - 2)):
            for last in range(n - 1):
                if (mask, last) not in dp:
                    continue
                val, path = dp[mask, last]
                for j in range(1, n - 1):
                    if mask & (1 << (j - 1)):
                        continue
                    key = (mask | (1 << (j - 1)), j)
                    v = val + d[last][j]
                    evaluations += 1
                    if v < dp.get(key, (math.inf, []))[0] - 1e-9:
                        dp[key] = (v, path + [j])
        full = (1 << (n - 2)) - 1
        _, middle = min(
            (v + d[last][-1], p) for (mask, last), (v, p) in dp.items() if mask == full
        )
        r = [0, *middle, n - 1]
    elif method == "ga":
        rng = random.Random(seed)
        base = list(range(1, n - 1))
        pop = [greedy(d)[1:-1]] + [rng.sample(base, len(base)) for _ in range(63)]

        def score(p):
            return obj([0, *p, n - 1])

        for iterations in range(1, 101):
            ranked = sorted((score(p), p) for p in pop)
            new = [ranked[0][1][:], ranked[1][1][:]]

            def tournament():
                return min(rng.sample(ranked, 3))[1]

            while len(new) < 64:
                a, b = tournament(), tournament()
                child = a[:]
                if rng.random() < 0.9:
                    i, j = sorted(rng.sample(range(len(base) + 1), 2))
                    segment = a[i:j]
                    remaining = [v for v in b if v not in segment]
                    child = remaining[:i] + segment + remaining[i:]
                if rng.random() < 0.2:
                    i, j = rng.sample(range(len(base)), 2)
                    child[i], child[j] = child[j], child[i]
                new.append(child)
            pop = new
        r = [0, *min(pop, key=score), n - 1]
    elif method == "gls":
        p = [[0] * n for _ in range(n)]
        r = local(greedy(d), obj)
        best = r[:]
        for iterations in range(1, 101):
            penalty_update(r, d, p)

            def augmented(rr):
                return obj(rr) + lam * sum(
                    p[a][b] * d[a][b] for a, b in zip(rr, rr[1:])
                )

            r = local(r, augmented)
            if cost(r, d) < cost(best, d) - 1e-9:
                best = r[:]
        r = best
    elif method == "ortools_gls":
        from ortools.constraint_solver import pywrapcp, routing_enums_pb2

        manager = pywrapcp.RoutingIndexManager(n, 1, [0], [n - 1])
        routing = pywrapcp.RoutingModel(manager)
        # Coordinates and edges have 0.01 resolution: 100x costs are exact for this graph.
        cb = routing.RegisterTransitCallback(
            lambda i, j: round(100 * d[manager.IndexToNode(i)][manager.IndexToNode(j)])
        )
        routing.SetArcCostEvaluatorOfAllVehicles(cb)
        params = pywrapcp.DefaultRoutingSearchParameters()
        params.first_solution_strategy = (
            routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC
        )
        params.local_search_metaheuristic = (
            routing_enums_pb2.LocalSearchMetaheuristic.GUIDED_LOCAL_SEARCH
        )
        params.guided_local_search_lambda_coefficient = 0.1
        params.solution_limit = 100
        params.time_limit.FromSeconds(5)
        sol = routing.SolveWithParameters(params)
        if sol is None:
            raise RuntimeError(f"OR-Tools status {routing.status()}")
        idx = routing.Start(0)
        r = []
        while not routing.IsEnd(idx):
            r.append(manager.IndexToNode(idx))
            idx = sol.Value(routing.NextVar(idx))
        r.append(n - 1)
        iterations = routing.solver().Solutions()
        evaluations = -1
        status = f"ortools_{routing.status()}"
    else:
        raise ValueError(method)
    return r, {"evaluations": evaluations, "iterations": iterations, "status": status}


def validate(m, nodes, d, route):
    if (
        not route
        or route[0] != 0
        or route[-1] != len(nodes) - 1
        or sorted(route[1:-1]) != list(range(1, len(nodes) - 1))
    ):
        raise ValueError("Invalid endpoints or shelf coverage")
    total = 0
    for i, j in zip(route, route[1:]):
        length, path = astar(m, nodes[i], nodes[j])
        if not path:
            raise ValueError("Unreachable route")
        for a, b in zip(path, path[1:]):
            weights = dict(m["GRAPH"].get(a, []))
            if b not in weights:
                raise ValueError("Illegal directed edge")
            total += weights[b]
        if abs(length - d[i][j]) > 1e-8:
            raise ValueError("Matrix mismatch")
    if abs(total - cost(route, d)) > 1e-8:
        raise ValueError("Expanded path cost mismatch")
    return total
