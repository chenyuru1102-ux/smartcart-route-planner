# Meeting reference review and replication scope

The local 2026-10-07 deck (9 slides) was read for Dijkstra/A* structure and its single illustrative comparison. The local 2026-10-21 deck (11 slides) was read as the primary conceptual specification. Slide text, native table text, and embedded diagrams/charts were inspected locally. Slide layouts were not rendered in PowerPoint; full slide layout QA is not claimed.

Structure: October 21 slides 3-4 define list -> A* -> shared matrix -> order optimizer -> route; slides 5-8 define the penalty loop; slides 9-10 show historical results. Key content: weighted augmented penalty, fixed terminals, original-cost incumbent. Format constraints: independent new results, private decks. Visual evidence: the matrix is symmetric, the store geometry differs from the current simulator, and temporary longer routes are allowed during GLS. Reuse: algorithm structure only; no deck image or historical data copied into this repository.

The flow and weighted penalty are implemented. Fixed terminals, shared matrices, 90 cases and four meeting methods are tested, with OR-Tools and exact DP added. Historical case-level replication is NOT REPRODUCIBLE from supplied materials: original cases, seeds and hyperparameters are missing. New substitutes and the map differences are documented in BENCHMARK.md. Lambda 0.35 is known only for the illustrated trace, so its use across new cases is an explicit engineering choice, not a recovered historical parameter.

Track B comparison is deferred. The original simulator's joint-stop DP remains intact. No historical result is represented as a new measurement.
