import unittest, itertools, math
from benchmarks.core import (
    load_map,
    astar,
    dijkstra,
    matrix,
    solve,
    cost,
    generate_cases,
    validate,
)


class BenchmarkTests(unittest.TestCase):
    def test_astar_all_pairs(self):
        m = load_map()
        for a in m["POS"]:
            oracle = dijkstra(m, a)
            for b in m["POS"]:
                c, p = astar(m, a, b)
                self.assertAlmostEqual(c, oracle.get(b, math.inf))
                if p:
                    self.assertEqual(p[0], a)
                    self.assertEqual(p[-1], b)

    def test_dp_bruteforce_and_solvers(self):
        m = load_map()
        for case in generate_cases(m)[:4]:
            nodes, mat = matrix(m, case)
            exact = min(
                cost([0, *p, len(mat) - 1], mat)
                for p in itertools.permutations(range(1, len(mat) - 1))
            )
            for method in ["greedy", "two_opt", "ga", "gls", "ortools_gls", "dp"]:
                route, meta = solve(method, mat, 11)
                validate(m, nodes, mat, route)
                self.assertGreaterEqual(cost(route, mat) + 1e-8, exact)
                if method == "dp":
                    self.assertAlmostEqual(cost(route, mat), exact)

    def test_repeatable(self):
        m = load_map()
        self.assertEqual(generate_cases(m), generate_cases(m))
        _, mat = matrix(m, generate_cases(m)[0])
        for method in ["greedy", "two_opt", "ga", "gls", "ortools_gls", "dp"]:
            self.assertEqual(solve(method, mat, 11), solve(method, mat, 11))

    def test_invalid_route(self):
        m = load_map()
        nodes, mat = matrix(m, generate_cases(m)[0])
        with self.assertRaises(ValueError):
            validate(m, nodes, mat, [0, len(mat) - 1])

    def test_unreachable(self):
        m = {"POS": {"a": [0, 0], "b": [1, 0]}, "GRAPH": {"a": [], "b": []}}
        self.assertEqual(astar(m, "a", "b"), (math.inf, []))


if __name__ == "__main__":
    unittest.main()


class AdditionalTests(unittest.TestCase):
    def test_obstacles_and_weights(self):
        m = load_map()
        for a, edges in m["GRAPH"].items():
            x1, y1 = m["POS"][a]
            for b, w in edges:
                x2, y2 = m["POS"][b]
                self.assertAlmostEqual(w, math.dist(m["POS"][a], m["POS"][b]))
                for _, (x, y, rw, rh) in m["OBSTACLES"]:
                    overlap = (
                        (
                            y - 0.025 <= y1 <= y + rh + 0.025
                            and max(min(x1, x2), x - 0.025)
                            < min(max(x1, x2), x + rw + 0.025)
                        )
                        if abs(y1 - y2) < 1e-8
                        else (
                            x - 0.025 <= x1 <= x + rw + 0.025
                            and max(min(y1, y2), y - 0.025)
                            < min(max(y1, y2), y + rh + 0.025)
                        )
                    )
                    self.assertFalse(overlap, (a, b))

    def test_asymmetric_dp(self):
        import random

        rng = random.Random(4)
        for n in range(3, 8):
            d = [
                [0 if i == j else rng.randint(1, 30) for j in range(n)]
                for i in range(n)
            ]
            expected = min(
                cost([0, *p, n - 1], d) for p in itertools.permutations(range(1, n - 1))
            )
            self.assertEqual(cost(solve("dp", d)[0], d), expected)
            self.assertLessEqual(
                cost(solve("two_opt", d)[0], d), cost(solve("greedy", d)[0], d)
            )

    def test_penalty_weighted(self):
        from benchmarks.core import penalty_update

        d = [[0, 8.06, 0], [0, 0, 2], [0, 0, 0]]
        p = [[0] * 3 for _ in range(3)]
        r = [0, 1, 2]
        penalty_update(r, d, p)
        self.assertEqual(p[0][1], 1)
        self.assertEqual(p[1][2], 0)
        self.assertAlmostEqual(
            0.35 * sum(p[a][b] * d[a][b] for a, b in zip(r, r[1:])), 2.821
        )

    def test_case_contract(self):
        m = load_map()
        cases = generate_cases(m)
        self.assertEqual(len(cases), 90)
        for n in (5, 8, 10):
            group = [c for c in cases if c["size"] == n]
            self.assertEqual(len({tuple(c["shelves"]) for c in group}), 30)
            for c in group:
                self.assertEqual(len(c["stops"]), n)
                for s, node in zip(c["shelves"], c["stops"]):
                    self.assertIn(node, m["SHELF_PICKUP_NODES"][s])
