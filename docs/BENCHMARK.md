# Benchmark A: reproducible directed shopping-route comparison

This is a NEW experiment run on 2026-11-04, not a recreation of historical slide means.

## Reproduce

From the repository root, with Python 3.12:

```sh
python -m pip install -r requirements-benchmark.txt
python -m pytest tests -q
python -m py_compile SmartCart_Final_Narrow_Aisle.py benchmark.py benchmarks/core.py
python benchmark.py
python benchmark.py --output repeat-results
```

Cases are generated only if `benchmarks/instances.json` is absent; otherwise the saved cases are used.
The generator seed is 20261021. There are 30 unique unordered shelf subsets at each size (5, 8, 10), sampled from all 15 shelves/endcaps. No cases were selected by performance.

## Problem contract and map provenance

`benchmarks/map.json` is an exact export of the original simulator's graph, positions, obstacles, and pickup candidates, with its source SHA-256. The GUI is never imported by the benchmark. The original remains unchanged.
Entrance is N3_0; the fixed final checkout surrogate is the simulator's Parking node N5_0.
All methods share one A* cost matrix per case. Graph costs are Euclidean edge lengths in schematic map units. A* uses the consistent Euclidean heuristic and only directed legal edges.

Track A fixes each shelf to the FIRST candidate listed in the original source, independent of solver. Some shelves share the same physical node; they remain distinct service labels with zero cost between colocated labels. A valid route visits every label exactly once. Passing a stop earlier along a physical shortest path is allowed; service is credited at the scheduled stop. The route may revisit physical nodes and pass checkout before finishing there. No service duration, cart capacity or time windows are modeled.

Track B allows joint pickup choice and shared coverage. The existing simulator implements that richer problem. No Track B comparison was run, and its distances are never included in Track A tables. Fixed-stop DP here is Held-Karp, not the simulator's joint pickup DP.

## Algorithms and declared budgets

- Greedy: nearest unvisited label, ties by matrix index.
- 2-opt: same Greedy initialization, best improvement, full directed path rescore, endpoints fixed. Reversing internal arcs is explicitly priced.
- GA: permutation chromosome; 64 individuals; 100 generations; tournament size 3; 2 elites; order-preserving segment crossover probability 0.9; swap mutation probability 0.2. Population includes Greedy and 63 seeded random permutations. Seeds 11, 22, 33. 6464 objective evaluations per run. No post-hoc tuning or local-search hybrid.
- Meeting-style GLS (`gls`): Greedy then 2-opt; 100 penalty cycles; directed edge utility c/(1+p); increment ALL tied maximum-utility edges; augmented objective L + lambda * sum(p*c), lambda=0.35. Preserve the best original-cost local optimum. Deterministic tie rules: no random seed applies. Sensitivity at 0.15/0.35/0.70 is saved separately; primary results remain 0.35.
- OR-Tools GLS (`ortools_gls`): PATH_CHEAPEST_ARC, GUIDED_LOCAL_SEARCH, coefficient 0.1, 100 accepted solutions and 5-second watchdog; integer scale 100 is exact for this orthogonal map's hundredth-unit edges. This is a separate variant, not a claim of identical slide penalty behavior. It is deterministic in this pinned environment, not a randomized stability study. All measured runs reached 100 solutions.
- DP: exact subset recurrence over (visited labels, last label), fixed terminals. Exponential time and space; used only up to 10 required labels.

These are fixed algorithm-specific budgets, NOT equal-runtime or equal-evaluation budgets. Runtime includes solver initialization but excludes A* preprocessing and route verification. Matrix time is separately recorded. DP transition relaxations and GA/GLS objective evaluations are different units; OR-Tools evaluations are unavailable (-1). Greedy's zero evaluations means no whole-route objective calls, not zero work.

OR-Tools runs locally without a paid API. See [official routing options](https://developers.google.com/optimization/routing/routing_options) for solution/time limits and [the official TSP guide](https://developers.google.com/optimization/routing/tsp) for GLS configuration.

## Measurement and aggregation

`results/benchmark.csv` and `raw_runs.csv` contain the same 720 measured runs: 90 each for five deterministic methods and 270 for GA. `matrices.json` stores all shared matrices. `summary.csv` aggregates GA seeds within each case before computing the between-case sample SD (n=30), preventing seeds from inflating the case count. Within-case seed SD is separately reported. Error bars are between-case SD, not confidence intervals.
Improvement is the mean of paired per-run percentages 100*(Greedy-cost)/Greedy; gap is 100*(cost-DP)/DP. Tiny negative values around 1e-14 are floating-point noise. The experiment is descriptive; no significance claim or hardware-independent runtime claim is made.

## Validation and limitations

Nine pytest tests passed: all 49x49 A*/Dijkstra distances; brute-force optimality on four five-stop cases; random asymmetric DP matrices; all six solvers' valid permutations and fixed endpoints; route expansion; map edge weights and obstacle clearance; unreachable graph; invalid route rejection; case uniqueness; repeatability; weighted GLS penalty example. Tests group multiple assertions per test method.
All 720 measured routes were expanded and validated against legal directed graph arcs and fixed endpoints. No failed cases were observed. A second full 720-run execution reproduced every non-timing CSV field and every matrix byte. Original source bytes match the ZIP. `results/tests.txt`, `reproducibility.json`, and `manifest.json` contain evidence and environment/source hashes. OR-Tools emitted three SWIG deprecation warnings; no test failed.
The GUI was syntax-checked but not interactively exercised; GUI smoke test is SKIPPED. Historical input lists, seeds, GA parameters, exact original graph construction and global GLS lambda policy are unavailable. The historical matrix image is symmetric; the current graph is directed. No historical mean has been reproduced or used as measured data. No claim is made about optimality on larger instances or other maps.


## Observed results

| Shelves | Method | Mean cost | Between-case SD | Mean solver ms | Improvement % | Gap to DP % |
|---|---|---:|---:|---:|---:|---:|
| 5 | greedy | 41.427 | 6.759 | 0.008 | 0.000 | 11.653 |
| 5 | two_opt | 38.527 | 5.480 | 0.031 | 6.428 | 3.953 |
| 5 | ga | 37.262 | 5.887 | 31.179 | 9.506 | 0.077 |
| 5 | gls | 37.233 | 5.883 | 4.480 | 9.578 | -0.000 |
| 5 | ortools_gls | 37.233 | 5.883 | 18.995 | 9.578 | -0.000 |
| 5 | dp | 37.233 | 5.883 | 0.123 | 9.578 | -0.000 |
| 8 | greedy | 49.287 | 7.268 | 0.017 | -0.000 | 16.793 |
| 8 | two_opt | 45.113 | 4.122 | 0.093 | 7.295 | 7.223 |
| 8 | ga | 43.567 | 3.791 | 41.439 | 10.562 | 3.337 |
| 8 | gls | 42.227 | 4.005 | 17.341 | 13.294 | -0.000 |
| 8 | ortools_gls | 42.227 | 4.005 | 32.484 | 13.294 | -0.000 |
| 8 | dp | 42.227 | 4.005 | 2.117 | 13.294 | -0.000 |
| 10 | greedy | 54.780 | 5.362 | 0.028 | -0.000 | 17.057 |
| 10 | two_opt | 50.873 | 5.168 | 0.219 | 6.864 | 8.646 |
| 10 | ga | 49.927 | 4.185 | 46.548 | 8.457 | 6.599 |
| 10 | gls | 46.840 | 2.994 | 32.919 | 13.945 | -0.000 |
| 10 | ortools_gls | 46.840 | 2.994 | 48.675 | 13.945 | -0.000 |
| 10 | dp | 46.840 | 2.994 | 12.744 | 13.945 | -0.000 |

Both GLS variants matched DP costs in all 90 cases. GA was suboptimal in some runs, particularly at 10 shelves; this is consistent with the limited generation budget but does not isolate the cause. DP was faster than either GLS at these small sizes. There is no evidence here that GLS is preferable to exact DP for 5-10 labels.

### GLS sensitivity (separate descriptive experiment)

| Lambda | 5 shelves | 8 shelves | 10 shelves |
|---|---:|---:|---:|
| 0.15 | 37.233 | 42.227 | 47.100 |
| 0.35 | 37.233 | 42.227 | 46.840 |
| 0.7 | 37.233 | 42.227 | 46.840 |
