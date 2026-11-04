#!/usr/bin/env python3
"""Synthetic, deterministic stress test for validation-locked open-set FPIR.

Each score is the attacker's maximum non-mated candidate score for one query.
This tests threshold transport under distribution shift; it is NOT a credential
attack, calibrated risk estimator, or certification evidence.
"""
from __future__ import annotations
import argparse
import json
import math
import random
from typing import Callable


def wilson_interval(events: int, trials: int, z: float = 1.959963984540054) -> tuple[float, float]:
    if trials <= 0:
        raise ValueError("trials must be positive")
    p = events / trials
    z2 = z * z
    denom = 1.0 + z2 / trials
    center = (p + z2 / (2.0 * trials)) / denom
    radius = z * math.sqrt((p * (1.0 - p) + z2 / (4.0 * trials)) / trials) / denom
    return max(0.0, center - radius), min(1.0, center + radius)


def nearest_rank_threshold(values: list[float], target_fpir: float) -> float:
    """Choose a threshold where strictly-greater empirical exceedances stay within cap."""
    if not values:
        raise ValueError("calibration sample cannot be empty")
    if not 0.0 < target_fpir < 1.0:
        raise ValueError("target_fpir must be in (0, 1)")
    ordered = sorted(values)
    allowed_events = math.floor(target_fpir * len(ordered))
    index = len(ordered) - allowed_events - 1
    return ordered[max(0, min(len(ordered) - 1, index))]


def normal(rng: random.Random) -> float:
    return rng.gauss(0.0, 1.0)


def baseline(rng: random.Random, n: int) -> list[float]:
    return [normal(rng) for _ in range(n)]


def mean_shift(rng: random.Random, n: int, delta: float) -> list[float]:
    return [normal(rng) + delta for _ in range(n)]


def scale_shift(rng: random.Random, n: int, scale: float) -> list[float]:
    return [normal(rng) * scale for _ in range(n)]


def heavy_tail(rng: random.Random, n: int) -> list[float]:
    return [normal(rng) + 3.0 if rng.random() < 0.01 else normal(rng) for _ in range(n)]


def subgroup_mix(rng: random.Random, n: int) -> tuple[list[float], dict[str, list[float]]]:
    scores: list[float] = []
    groups = {"reference_stratum": [], "shifted_stratum": []}
    for _ in range(n):
        if rng.random() < 0.10:
            value = normal(rng) + 0.75
            groups["shifted_stratum"].append(value)
        else:
            value = normal(rng)
            groups["reference_stratum"].append(value)
        scores.append(value)
    return scores, groups


def describe(scores: list[float], threshold: float) -> dict[str, object]:
    events = sum(value > threshold for value in scores)
    low, high = wilson_interval(events, len(scores))
    return {
        "queries": len(scores),
        "false_identifications": events,
        "observed_fpir": events / len(scores),
        "wilson_95_ci": [low, high],
    }


def run(seed: int, n_calibration: int, n_test: int, target_fpir: float) -> dict[str, object]:
    if n_calibration < 1 or n_test < 1:
        raise ValueError("sample counts must be positive")
    if not 0.0 < target_fpir < 1.0 or math.floor(target_fpir * n_calibration) < 5:
        raise ValueError("target_fpir must be in (0, 1) and calibration must allow at least five events")

    calibration = baseline(random.Random(seed), n_calibration)
    threshold = nearest_rank_threshold(calibration, target_fpir)
    calibration_result = describe(calibration, threshold)
    specs: list[tuple[str, Callable[[random.Random, int], list[float]], str]] = [
        ("no_shift_control", lambda rng, n: baseline(rng, n), "IID reference distribution"),
        ("mean_shift_plus_0_25_sd", lambda rng, n: mean_shift(rng, n, 0.25), "Impostor-query maximum shifts +0.25 standard deviations"),
        ("mean_shift_plus_0_50_sd", lambda rng, n: mean_shift(rng, n, 0.50), "Impostor-query maximum shifts +0.50 standard deviations"),
        ("scale_shift_1_25x", lambda rng, n: scale_shift(rng, n, 1.25), "Score dispersion grows by 25% with threshold held fixed"),
        ("scale_shift_1_50x", lambda rng, n: scale_shift(rng, n, 1.50), "Score dispersion grows by 50% with threshold held fixed"),
        ("one_percent_heavy_tail_contamination", lambda rng, n: heavy_tail(rng, n), "1% of queries receive a +3 SD tail component"),
    ]
    scenarios = []
    for offset, (name, generator, description) in enumerate(specs, start=1):
        result = describe(generator(random.Random(seed + offset * 104729), n_test), threshold)
        result.update({"scenario": name, "description": description, "strata": None})
        scenarios.append(result)

    mixed_scores, group_scores = subgroup_mix(random.Random(seed + 7 * 104729), n_test)
    strata = {name: describe(values, threshold) for name, values in group_scores.items()}
    mixed = describe(mixed_scores, threshold)
    mixed.update({
        "scenario": "new_shifted_stratum_10_percent",
        "description": "Test adds 10% subpopulation with a +0.75 SD shift; one global threshold remains locked",
        "strata": strata,
    })
    scenarios.append(mixed)
    no_shift = next(row for row in scenarios if row["scenario"] == "no_shift_control")
    mean = next(row for row in scenarios if row["scenario"] == "mean_shift_plus_0_50_sd")
    shifted = next(row for row in scenarios if row["scenario"] == "new_shifted_stratum_10_percent")
    shifted_group = shifted["strata"]["shifted_stratum"]
    return {
        "schema_version": "1.0",
        "diagnostic": "open_set_validation_locked_threshold_population_shift",
        "qualification_status": "NOT_A_PRIVACY_PASS_SYNTHETIC_SHIFT_DIAGNOSTIC_ONLY",
        "seed": seed,
        "synthetic_only": True,
        "score_semantics": "one maximum non-mated candidate score per query; not a credential-format or deployed-attacker score",
        "threshold_policy": "calibration-only nearest-rank threshold; count an identification when test score is strictly greater than threshold",
        "target_fpir": target_fpir,
        "calibration": {
            "queries": n_calibration,
            "false_identifications": calibration_result["false_identifications"],
            "observed_fpir": calibration_result["observed_fpir"],
            "threshold": threshold,
            "interpretation": "empirical calibration constraint only; does not guarantee future FPIR under shift",
        },
        "scenarios": scenarios,
        "directional_controls": {
            "no_shift_control_observed_fpir": no_shift["observed_fpir"],
            "mean_shift_plus_0_50_sd_observed_fpir": mean["observed_fpir"],
            "shifted_stratum_observed_fpir": shifted_group["observed_fpir"],
            "mean_shift_exceeds_no_shift": mean["observed_fpir"] > no_shift["observed_fpir"],
            "shifted_stratum_exceeds_target": shifted_group["observed_fpir"] > target_fpir,
        },
        "limitations": [
            "Synthetic score-distribution stress test only; no actual credential format or attacker is evaluated.",
            "The score distribution is specified directly; candidate-gallery generation and candidate-frame uncertainty are not modeled.",
            "Only false-positive identification risk is studied; matched-query FNIR and task utility are not measured.",
            "Wilson intervals quantify binomial sampling uncertainty conditional on this simulation, not transportability to a real population.",
            "Passing directional controls validates the simulator, not a privacy mitigation or release.",
        ],
    }


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--seed", type=int, default=990129)
    parser.add_argument("--calibration-queries", type=int, default=100_000)
    parser.add_argument("--test-queries", type=int, default=100_000)
    parser.add_argument("--target-fpir", type=float, default=0.01)
    parser.add_argument("--self-test", action="store_true", help="assert deterministic replay and directional negative controls")
    args = parser.parse_args()
    report = run(args.seed, args.calibration_queries, args.test_queries, args.target_fpir)
    encoded = json.dumps(report, indent=2, sort_keys=True) + "\n"
    if args.self_test:
        replay = json.dumps(run(args.seed, args.calibration_queries, args.test_queries, args.target_fpir), indent=2, sort_keys=True) + "\n"
        assert encoded == replay, "same seed must reproduce byte-identical report in this runtime"
        controls = report["directional_controls"]
        assert controls["mean_shift_exceeds_no_shift"], "mean-shift negative control did not raise FPIR"
        assert controls["shifted_stratum_exceeds_target"], "shifted subgroup should expose a worst-stratum violation"
        no_shift = next(row for row in report["scenarios"] if row["scenario"] == "no_shift_control")
        low, high = no_shift["wilson_95_ci"]
        assert low <= report["target_fpir"] <= high, "no-shift control should be statistically compatible with target for this pinned seed"
    print(encoded, end="")


if __name__ == "__main__":
    main()
