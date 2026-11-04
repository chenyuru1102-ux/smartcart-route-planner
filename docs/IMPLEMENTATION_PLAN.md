# Implementation plan

Use the existing directed graph exported without GUI imports; preserve the source byte for byte.
Track A fixes each shelf to its first declared pickup candidate. Shared physical nodes remain separate service labels with zero travel cost. All solvers use the same matrix and fixed Entrance/Parking endpoints.
Implement independent A*, nearest neighbor, directed full-cost best-improvement 2-opt, permutation GA, weighted-penalty GLS, OR-Tools GLS and exact Held-Karp DP.
Save 30 unique shelf sets per size, seed 20261021. GA seeds 11/22/33; deterministic GLS; OR-Tools fixed solution budget with watchdog.
Test A* against independent Dijkstra, DP against permutations, all graph edges against obstacles, endpoint/coverage validation, and exact rerun identity excluding timing.
Generate raw CSV, case-level summary, chart and English report. Track B is explicitly deferred, never mixed with A.
Publish original baseline on main before benchmark branch, without private references.
