import argparse, csv, hashlib, json, platform, statistics as st, time
from pathlib import Path
from benchmarks.core import *

METHODS = ["greedy", "two_opt", "ga", "gls", "ortools_gls", "dp"]


def csvout(p, rows):
    with p.open("w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0]))
        w.writeheader()
        w.writerows(rows)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--output", default="results")
    args = ap.parse_args()
    out = Path(args.output)
    out.mkdir(parents=True, exist_ok=True)
    root = Path(__file__).parent
    m = load_map()
    ip = root / "benchmarks/instances.json"
    if not ip.exists():
        ip.write_text(json.dumps(generate_cases(m), indent=2))
    cases = json.loads(ip.read_text())
    rows = []
    matrices = {}
    for case in cases:
        t = time.perf_counter()
        nodes, d = matrix(m, case)
        mt = time.perf_counter() - t
        matrices[case["id"]] = {"nodes": nodes, "costs": d}
        base = cost(solve("greedy", d)[0], d)
        opt = cost(solve("dp", d)[0], d)
        for method in METHODS:
            for seed in ([11, 22, 33] if method == "ga" else [11]):
                row = dict(
                    case_id=case["id"],
                    size=case["size"],
                    method=method,
                    seed=seed if method == "ga" else "",
                    feasible=False,
                    cost="",
                    runtime_s=0,
                    matrix_runtime_s=mt,
                    improvement_pct="",
                    gap_pct="",
                    route="",
                    evaluations="",
                    iterations="",
                    status="",
                    error="",
                )
                t = time.perf_counter()
                try:
                    r, meta = solve(method, d, seed)
                    row["runtime_s"] = time.perf_counter() - t
                    c = validate(m, nodes, d, r)
                    row.update(
                        feasible=True,
                        cost=c,
                        improvement_pct=100 * (base - c) / base,
                        gap_pct=100 * (c - opt) / opt,
                        route=json.dumps(r),
                        **meta
                    )
                except Exception as exc:
                    row.update(
                        runtime_s=time.perf_counter() - t,
                        error=repr(exc),
                        status="failed",
                    )
                rows.append(row)
        print(case["id"], flush=True)
    csvout(out / "benchmark.csv", rows)
    csvout(out / "raw_runs.csv", rows)
    (out / "matrices.json").write_text(json.dumps(matrices, indent=2))
    summary = []
    for size in (5, 8, 10):
        for method in METHODS:
            rr = [r for r in rows if r["size"] == size and r["method"] == method]
            valid = [r for r in rr if r["feasible"]]
            cc = [
                [r for r in valid if r["case_id"] == c["id"]]
                for c in cases
                if c["size"] == size
            ]
            cc = [r for r in cc if r]
            means = [st.mean(r["cost"] for r in c) for c in cc]
            summary.append(
                dict(
                    size=size,
                    method=method,
                    cases=len(cc),
                    runs=len(rr),
                    feasible_rate=len(valid) / len(rr),
                    mean_cost=st.mean(means),
                    std_cost_between_cases=st.stdev(means),
                    median_cost=st.median(means),
                    mean_runtime_ms=1000 * st.mean(r["runtime_s"] for r in valid),
                    mean_improvement_pct=st.mean(r["improvement_pct"] for r in valid),
                    mean_gap_pct=st.mean(r["gap_pct"] for r in valid),
                    mean_within_case_seed_std=st.mean(
                        st.stdev(r["cost"] for r in c) if len(c) > 1 else 0 for c in cc
                    ),
                )
            )
    csvout(out / "summary.csv", summary)
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, ax = plt.subplots(figsize=(11, 6))
    for j, method in enumerate(METHODS):
        ss = [r for r in summary if r["method"] == method]
        ax.bar(
            [i + (j - 2.5) * 0.13 for i in range(3)],
            [r["mean_cost"] for r in ss],
            width=0.13,
            yerr=[r["std_cost_between_cases"] for r in ss],
            capsize=2,
            label=method,
        )
    ax.set_xticks(range(3), ["5 shelves", "8 shelves", "10 shelves"])
    ax.set_ylim(bottom=0)
    ax.set_ylabel("Route cost (schematic map units)")
    ax.set_title(
        "New directed-map Benchmark A | 30 cases per size\nMean and between-case sample SD; GA averaged over 3 seeds"
    )
    ax.legend(ncol=3)
    fig.tight_layout()
    fig.savefig(out / "comparison.png", dpi=180)
    plt.close(fig)
    import ortools

    manifest = dict(
        date="2026-11-04",
        track="A",
        python=platform.python_version(),
        platform=platform.platform(),
        ortools=ortools.__version__,
        matplotlib=matplotlib.__version__,
        instance_seed=20261021,
        ga_seeds=[11, 22, 33],
        ga=dict(
            population=64,
            generations=100,
            tournament=3,
            elitism=2,
            crossover=0.9,
            mutation=0.2,
        ),
        gls=dict(cycles=100, weighted_lambda=0.35),
        ortools_config=dict(
            solution_limit=100, watchdog_s=5, lambda_coefficient=0.1, scale=100
        ),
        cases=len(cases),
        runs=len(rows),
        failed_runs=sum(not r["feasible"] for r in rows),
        hashes={
            str(p.relative_to(root)): hashlib.sha256(p.read_bytes()).hexdigest()
            for p in [
                root / "benchmark.py",
                root / "benchmarks/core.py",
                root / "benchmarks/map.json",
                ip,
                root / "SmartCart_Final_Narrow_Aisle.py",
            ]
        },
    )
    (out / "manifest.json").write_text(json.dumps(manifest, indent=2))
    print(json.dumps(summary, indent=2))


if __name__ == "__main__":
    main()
