#!/usr/bin/env python3
"""Deterministic finite-grid coverage diagnostic for crossed cluster bootstrap.

This research-only simulator mirrors the row/column multiplicity bootstrap used by
gallery-sweep. It deliberately retains known sparse-event undercoverage as a
regression signal; it is not a certification test for arbitrary data-generating
processes.
"""
from __future__ import annotations

import json
import math
import random
import sys


def logistic(value: float) -> float:
    if value >= 0.0:
        z = math.exp(-value)
        return 1.0 / (1.0 + z)
    z = math.exp(value)
    return z / (1.0 + z)


def marginal_probability(alpha: float, subject_effect: float, gallery_effect: float) -> float:
    return sum(
        logistic(alpha + subject_effect * s + gallery_effect * g)
        for s in (-1, 1) for g in (-1, 1)
    ) / 4.0


def intercept_for(target: float, subject_effect: float, gallery_effect: float) -> float:
    low, high = -30.0, 30.0
    for _ in range(100):
        middle = (low + high) / 2.0
        if marginal_probability(middle, subject_effect, gallery_effect) < target:
            low = middle
        else:
            high = middle
    return (low + high) / 2.0


def quantile(sorted_values: list[float], probability: float) -> float:
    position = probability * (len(sorted_values) - 1)
    lower, upper = math.floor(position), math.ceil(position)
    if lower == upper:
        return sorted_values[lower]
    fraction = position - lower
    return sorted_values[lower] * (1.0 - fraction) + sorted_values[upper] * fraction


def crossed_interval(matrix: list[list[int]], rng: random.Random, replicates: int) -> tuple[float, float]:
    n, m = len(matrix), len(matrix[0])
    estimates = []
    for _ in range(replicates):
        row_counts, col_counts = [0] * n, [0] * m
        for _ in range(n):
            row_counts[rng.randrange(n)] += 1
        for _ in range(m):
            col_counts[rng.randrange(m)] += 1
        total = 0
        for i, row in enumerate(matrix):
            if row_counts[i] == 0:
                continue
            for j, value in enumerate(row):
                if col_counts[j]:
                    total += value * row_counts[i] * col_counts[j]
        estimates.append(total / (n * m))
    estimates.sort()
    return quantile(estimates, 0.025), quantile(estimates, 0.975)


def wilson_95(successes: int, trials: int) -> list[float]:
    z = 1.959963984540054
    observed = successes / trials
    denominator = 1.0 + z * z / trials
    center = (observed + z * z / (2 * trials)) / denominator
    half = z * math.sqrt(observed * (1 - observed) / trials + z * z / (4 * trials * trials)) / denominator
    return [round(max(0.0, center - half), 4), round(min(1.0, center + half), 4)]


def simulate(name: str, n: int, m: int, target: float, subject_effect: float,
             gallery_effect: float, studies: int, bootstraps: int, seed: int) -> dict:
    rng = random.Random(seed)
    alpha = intercept_for(target, subject_effect, gallery_effect)
    covered, zero_event_studies, widths = 0, 0, []
    for _ in range(studies):
        subjects = [1 if rng.randrange(2) else -1 for _ in range(n)]
        galleries = [1 if rng.randrange(2) else -1 for _ in range(m)]
        matrix = []
        for subject in subjects:
            row = []
            for gallery in galleries:
                p = logistic(alpha + subject_effect * subject + gallery_effect * gallery)
                row.append(int(rng.random() < p))
            matrix.append(row)
        estimate = sum(map(sum, matrix)) / (n * m)
        zero_event_studies += int(estimate == 0.0)
        low, high = crossed_interval(matrix, rng, bootstraps)
        covered += int(low <= target <= high)
        widths.append(high - low)
    return {
        "scenario": name,
        "subjects": n,
        "gallery_draws": m,
        "true_marginal_event_probability": target,
        "subject_log_odds_effect": subject_effect,
        "gallery_log_odds_effect": gallery_effect,
        "outer_studies": studies,
        "bootstrap_replicates_per_study": bootstraps,
        "observed_95pct_coverage": round(covered / studies, 4),
        "coverage_monte_carlo_wilson_95pct_interval": wilson_95(covered, studies),
        "mean_interval_width": round(sum(widths) / len(widths), 5),
        "fraction_studies_with_zero_events": round(zero_event_studies / studies, 4),
        "interpretation": "finite synthetic diagnostic; no universal coverage claim"
    }


def main() -> int:
    studies = int(sys.argv[1]) if len(sys.argv) > 1 else 250
    bootstraps = int(sys.argv[2]) if len(sys.argv) > 2 else 120
    seed = int(sys.argv[3]) if len(sys.argv) > 3 else 990117
    if studies < 100 or bootstraps < 50:
        raise SystemExit("requires at least 100 outer studies and 50 bootstrap replicates")
    cases = [
        ("iid_sparse_p001", 30, 20, 0.001, 0.0, 0.0),
        ("iid_sparse_p005", 30, 20, 0.005, 0.0, 0.0),
        ("crossed_sparse_small", 20, 10, 0.01, 0.7, 0.7),
        ("subject_clustered_p01", 30, 20, 0.01, 0.7, 0.0),
        ("gallery_clustered_p01", 30, 20, 0.01, 0.0, 0.7),
        ("crossed_clustered_p10", 30, 20, 0.10, 0.7, 0.7),
        ("crossed_clustered_p50", 30, 20, 0.50, 0.7, 0.7),
    ]
    results = [
        simulate(name, n, m, target, su, sg, studies, bootstraps, seed + i * 104729)
        for i, (name, n, m, target, su, sg) in enumerate(cases)
    ]
    sparse = results[0]
    report = {
        "schema_version": "1.0",
        "method": "two_way_crossed_subject_and_gallery_draw_percentile_bootstrap",
        "seed": seed,
        "outer_studies_per_scenario": studies,
        "bootstrap_replicates_per_study": bootstraps,
        "scenarios": results,
        "known_sparse_event_undercoverage_reproduced": sparse["observed_95pct_coverage"] < 0.80,
        "qualification": "finite grid only; undercoverage in sparse-event conditions blocks certification use"
    }
    print(json.dumps(report, indent=2, sort_keys=True))
    if not report["known_sparse_event_undercoverage_reproduced"]:
        print("ERROR: did not reproduce the preregistered sparse-event failure mode", file=sys.stderr)
        return 2
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
