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
import hashlib
import importlib.util
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
    template = next(
        flow for scenario in scenarios for flow in scenario["flow_records"]
        if flow["metric_id"] == "precipitation_input"
    )
    remaining = target_flow_count - current_count
    quotient, remainder = divmod(remaining, len(scenarios))
    generated = 0
    for scenario_index, scenario in enumerate(scenarios):
        amount = quotient + (1 if scenario_index < remainder else 0)
        for local_index in range(amount):
            flow = dict(template)
            flow["flow_id"] = f"bench-{scenario['scenario_id']}-{generated:07d}"
            flow["rationale"] = (
                "Seeded synthetic scale-benchmark flow; deliberately not bound "
                "to a balance, not an observation, and not a city outcome."
            )
            flow.pop("uncertainty", None)
            scenario["flow_records"].append(flow)
            generated += 1
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

    return {
        "requested_flows": target_flows,
        "observed_flows": observed_flows,
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
        "commit": os.environ.get("GITHUB_SHA", "unknown"),
        "python": sys.version.split()[0],
        "jsonschema": load_validator_module().jsonschema.__version__ if hasattr(load_validator_module(), "jsonschema") else "unknown",
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
