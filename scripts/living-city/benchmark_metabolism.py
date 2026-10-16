#!/usr/bin/env python3
"""Deterministic scale benchmark for the Living-City schema and semantic validator.

Example:
  python scripts/living-city/benchmark_metabolism.py --sizes 1000,10000 --repeats 5
  python scripts/living-city/benchmark_metabolism.py --sizes 1000 --repeats 1 --output /tmp/living-city-bench.json

This is a developer/CI benchmark, not a production service. It expands only a
synthetic fixture, preserving the fixture's provenance status and scenario
invariants. Timings exclude fixture expansion and catalog loading.
"""
from __future__ import annotations

import argparse
import copy
import hashlib
import importlib.util
from importlib.metadata import version
import json
import math
import os
import platform
import statistics
import subprocess
import sys
import time
from pathlib import Path

SCRIPT_PATH = Path(__file__).resolve()
REPO_ROOT = SCRIPT_PATH.parents[2]
VALIDATOR_PATH = SCRIPT_PATH.with_name("validate_metabolism.py")


def load_validator_module():
    spec = importlib.util.spec_from_file_location("living_city_validator", VALIDATOR_PATH)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"cannot import validator from {VALIDATOR_PATH}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def percentile_nearest_rank(values: list[float], percentile: float) -> float:
    if not values:
        raise ValueError("percentile requires at least one sample")
    ordered = sorted(values)
    index = max(0, math.ceil(percentile * len(ordered)) - 1)
    return ordered[index]


def rss_high_water_mib() -> float | None:
    try:
        import resource
        # GitHub's ubuntu-latest uses Linux, where ru_maxrss is reported in KiB.
        value = resource.getrusage(resource.RUSAGE_SELF).ru_maxrss
        return round(value / 1024.0, 3)
    except (ImportError, AttributeError, OSError):
        return None


def expand_synthetic_record(module, target_flow_count: int):
    if target_flow_count < 12:
        raise ValueError("target flow count must be at least 12")
    record = module.load_json(module.SYNTHETIC_PATH)
    scenarios = record["scenarios"]
    current_count = sum(len(s["flow_records"]) for s in scenarios)
    if target_flow_count < current_count:
        raise ValueError(
            f"target flow count {target_flow_count} is below fixture baseline {current_count}"
        )
    # Cycle across all source fixture flow shapes, not just a single simple metric.
    # This gives the schema and semantic validator a more representative synthetic workload.
    templates = [
        flow for scenario in scenarios for flow in scenario["flow_records"]
    ]
    remaining = target_flow_count - current_count
    quotient, remainder = divmod(remaining, len(scenarios))
    generated = 0
    for scenario_index, scenario in enumerate(scenarios):
        amount = quotient + (1 if scenario_index < remainder else 0)
        for local_index in range(amount):
            flow = copy.deepcopy(templates[generated % len(templates)])
            flow["flow_id"] = f"bench-{scenario['scenario_id']}-{generated:07d}"
            flow["rationale"] = (
                "Deterministic synthetic scale-benchmark flow; deliberately not "
                "bound to a balance, not an observation, and not a city outcome."
            )
            scenario["flow_records"].append(flow)
            generated += 1

    # Exercise cohort and causal metadata on every timed schema/semantic pass.
    # These are synthetic declarations only; they are not findings about a city.
    scenario_ids = {scenario["scenario_id"] for scenario in scenarios}
    if {"baseline", "bioswale-and-canopy"}.issubset(scenario_ids):
        record["distributional_impacts"] = [{
            "impact_id": "synthetic-benchmark-cohort-delta",
            "scenario_id": "bioswale-and-canopy",
            "baseline_scenario_ref": "baseline",
            "affected_group_ref": "synthetic-benchmark-cohort",
            "metric_id": "accessible_person_hours",
            "assessment_status": "quantified",
            "baseline_value": 100,
            "scenario_value": 112,
            "reported_delta": 12,
            "unit": "person_hour",
            "direction": "higher_is_better",
            "uncertainty": {
                "lower_bound": 8,
                "upper_bound": 16,
                "method": "Synthetic benchmark interval; not an observed effect or confidence interval."
            },
            "evidence_refs": [],
            "limitations": ["Synthetic benchmark metadata only; no real cohort outcome is asserted."],
            "constraint_status": "not_evaluated",
            "constraint_ref": None
        }]
        record["distributional_coverage"] = {
            "coverage_schema_version": "living-city-distributional-coverage/0.1",
            "coverage_status": "not_assessed",
            "population_frame": "Synthetic benchmark cohort only; no real population frame is defined.",
            "group_identification_method": "Fixed synthetic identifier; no real group identification was performed.",
            "included_group_refs": ["synthetic-benchmark-cohort"],
            "known_omissions": ["All real-world cohorts and population coverage are omitted."],
            "omission_review_method": "No real population omission review was conducted for this benchmark.",
            "intersectional_review_status": "not_performed",
            "intersectional_dimensions_considered": [],
            "evidence_refs": [],
            "limitations": ["Synthetic coverage metadata only; not proof of completeness."]
        }
        if record.get("interactions"):
            record["causal_evaluations"] = [{
                "evaluation_id": "synthetic-benchmark-descriptive-interaction",
                "interaction_ref": record["interactions"][0]["interaction_id"],
                "claim_level": "descriptive_interaction",
                "design_type": "not_applicable",
                "unit_of_analysis": "Synthetic scenario comparison only",
                "treatment_assignment": "Synthetic fixture values; no real assignment occurred.",
                "counterfactual_strategy": "Four synthetic values test arithmetic only.",
                "confounder_strategy": "Not applicable to a synthetic benchmark fixture.",
                "interference_assumptions": "No real units exist; spillovers are not assessed.",
                "predeclared_outcomes": ["synthetic interaction arithmetic"],
                "falsification_checks": [],
                "evidence_refs": [],
                "review_status": "not_assessed",
                "review_ref": None,
                "reviewer_role": "not_applicable",
                "design_rationale": "Benchmark only that interaction arithmetic remains descriptive.",
                "limitations": ["Synthetic fixture only; it cannot establish a causal effect."]
            }]
    observed = sum(len(s["flow_records"]) for s in scenarios)
    if observed != target_flow_count:
        raise AssertionError(f"generated {observed} flows; expected {target_flow_count}")
    return record, observed


def run_one_size(target_flows: int, warmups: int):
    module = load_validator_module()
    schema = module.load_json(module.SCHEMA_PATH)
    module.Draft202012Validator.check_schema(schema)
    validator = module.Draft202012Validator(schema, format_checker=module.FormatChecker())
    concepts, evidence_ids = module.catalogs()
    prep_started = time.perf_counter()
    record, observed_flows = expand_synthetic_record(module, target_flows)
    preparation_ms = (time.perf_counter() - prep_started) * 1000.0

    def schema_check():
        errors = list(validator.iter_errors(record))
        if errors:
            raise RuntimeError("expanded fixture failed schema validation: " + module.describe(errors[:5]))

    def semantic_check():
        errors = module.semantic_errors(record, concepts, evidence_ids)
        if errors:
            raise RuntimeError("expanded fixture failed semantic validation: " + "; ".join(errors[:5]))

    for _ in range(warmups):
        schema_check()
        semantic_check()

    started = time.perf_counter()
    schema_check()
    schema_ms = (time.perf_counter() - started) * 1000.0

    started = time.perf_counter()
    semantic_check()
    semantic_ms = (time.perf_counter() - started) * 1000.0

    all_flows = [flow for scenario in record["scenarios"] for flow in scenario["flow_records"]]
    return {
        "requested_flows": target_flows,
        "observed_flows": observed_flows,
        "distinct_metric_count": len({flow["metric_id"] for flow in all_flows}),
        "flows_with_uncertainty_count": sum("uncertainty" in flow for flow in all_flows),
        "distributional_impact_count": len(record.get("distributional_impacts", [])),
        "distributional_coverage_count": int("distributional_coverage" in record),
        "causal_evaluation_count": len(record.get("causal_evaluations", [])),
        "referenced_evidence_count": sum(len(flow.get("evidence_refs", [])) for flow in all_flows),
        "conservation_check_count": sum(len(scenario["conservation_checks"]) for scenario in record["scenarios"]),
        "preparation_ms_excluded_from_validation": round(preparation_ms, 3),
        "schema_validation_ms": round(schema_ms, 3),
        "semantic_validation_ms": round(semantic_ms, 3),
        "combined_validation_ms": round(schema_ms + semantic_ms, 3),
        "process_peak_rss_mib": rss_high_water_mib(),
        "fixture_status": "synthetic_test_only",
        "errors": 0,
        "warmup_passes": warmups,
    }


def worker_main(args):
    result = run_one_size(args.flows, args.warmups)
    print(json.dumps(result, sort_keys=True, separators=(",", ":")))


def parent_main(args):
    sizes = [int(part) for part in args.sizes.split(",") if part.strip()]
    if not sizes or any(size < 12 for size in sizes):
        raise ValueError("--sizes must be comma-separated positive flow counts (minimum 12)")
    if args.repeats < 1 or args.warmups < 0:
        raise ValueError("--repeats must be >=1 and --warmups must be >=0")
    samples = []
    for size in sizes:
        for repeat in range(args.repeats):
            command = [
                sys.executable, str(SCRIPT_PATH), "--worker",
                "--flows", str(size), "--warmups", str(args.warmups),
            ]
            completed = subprocess.run(command, capture_output=True, text=True, check=False)
            if completed.returncode != 0:
                raise RuntimeError(
                    f"benchmark worker failed for flows={size}, repeat={repeat + 1}: "
                    f"{completed.stderr[-4000:]}"
                )
            try:
                sample = json.loads(completed.stdout)
            except json.JSONDecodeError as exc:
                raise RuntimeError(f"benchmark worker produced invalid JSON: {completed.stdout[:1000]}") from exc
            sample["repeat"] = repeat + 1
            samples.append(sample)

    groups = []
    for size in sizes:
        rows = [row for row in samples if row["requested_flows"] == size]
        groups.append({
            "flows": size,
            "samples": len(rows),
            "schema_ms_median": round(statistics.median(r["schema_validation_ms"] for r in rows), 3),
            "schema_ms_p95": round(percentile_nearest_rank([r["schema_validation_ms"] for r in rows], 0.95), 3),
            "semantic_ms_median": round(statistics.median(r["semantic_validation_ms"] for r in rows), 3),
            "semantic_ms_p95": round(percentile_nearest_rank([r["semantic_validation_ms"] for r in rows], 0.95), 3),
            "combined_ms_median": round(statistics.median(r["combined_validation_ms"] for r in rows), 3),
            "combined_ms_p95": round(percentile_nearest_rank([r["combined_validation_ms"] for r in rows], 0.95), 3),
            "peak_rss_mib_max": max(
                (r["process_peak_rss_mib"] for r in rows if r["process_peak_rss_mib"] is not None),
                default=None,
            ),
        })

    report = {
        "benchmark_schema": "living-city-validator-benchmark/0.1",
        "classification": "synthetic developer benchmark; not a city result",
        "commit": os.environ.get("LIVING_CITY_BENCHMARK_COMMIT", os.environ.get("GITHUB_SHA", "unknown")),
        "python": sys.version.split()[0],
        "jsonschema": version("jsonschema"),
        "platform": platform.platform(),
        "cpu_count": os.cpu_count(),
        "target_cpu_ram": args.runner_label,
        "repeats_per_size": args.repeats,
        "warmups_per_worker": args.warmups,
        "aggregation": "p95 uses nearest-rank definition; each sample runs in a fresh process",
        "targets": groups,
        "raw_samples": samples,
        "interpretation": "Measurements describe this environment only; compare thresholds only on the pinned reference runner.",
    }
    normalized = json.dumps(report, sort_keys=True, separators=(",", ":"), ensure_ascii=False).encode("utf-8")
    report["report_sha256"] = hashlib.sha256(normalized).hexdigest()
    encoded = json.dumps(report, sort_keys=True, indent=2, ensure_ascii=False) + "\n"
    if args.output:
        output_path = Path(args.output)
        output_path.parent.mkdir(parents=True, exist_ok=True)
        output_path.write_text(encoded, encoding="utf-8")
    print(encoded, end="")


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--sizes", default="1000,10000,100000", help="comma-separated target flow counts")
    parser.add_argument("--repeats", type=int, default=5, help="fresh-process measurements per size")
    parser.add_argument("--warmups", type=int, default=1, help="validation passes before each timed sample")
    parser.add_argument("--runner-label", default="unclassified-runner; CPU/RAM must be recorded externally",
                        help="human-declared runner label; do not present as measured hardware metadata")
    parser.add_argument("--output", help="optional path to also write the JSON report")
    parser.add_argument("--worker", action="store_true", help=argparse.SUPPRESS)
    parser.add_argument("--flows", type=int, default=1000, help=argparse.SUPPRESS)
    return parser.parse_args()


def main():
    args = parse_args()
    if args.worker:
        worker_main(args)
    else:
        parent_main(args)


if __name__ == "__main__":
    main()
