"""Compare independent reruns without pretending runtime is deterministic."""

import argparse
import csv
import json
from pathlib import Path


def compare(first, second):
    ignore = {"runtime_s", "matrix_runtime_s"}

    def read(folder):
        with (folder / "benchmark.csv").open() as f:
            return [
                {k: v for k, v in r.items() if k not in ignore}
                for r in csv.DictReader(f)
            ]

    a, b = read(first), read(second)
    if not a or a != b:
        raise ValueError("Non-timing run records differ or are empty")
    if json.loads((first / "matrices.json").read_text()) != json.loads(
        (second / "matrices.json").read_text()
    ):
        raise ValueError("Matrices differ")
    return {
        "runs_compared": len(a),
        "all_nontiming_fields_identical": True,
        "matrices_identical": True,
    }


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("first", type=Path)
    parser.add_argument("second", type=Path)
    args = parser.parse_args()
    print(json.dumps(compare(args.first, args.second), indent=2))
