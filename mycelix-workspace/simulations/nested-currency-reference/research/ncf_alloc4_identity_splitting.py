#!/usr/bin/env python3
"""NCF-ALLOC-4: identity-splitting attack on equal floors and median caps.

This research harness gives one adversarial principal a fixed total true backlog
and lets it represent that same work with increasing numbers of shard IDs. It is
not a Sybil defence or a production identity model.
"""
from __future__ import annotations
import argparse, hashlib, json
from pathlib import Path
from ncf_alloc3 import allocate, metrics, SPENDABLE, SHARDS, TOTAL_CAPACITY, RESERVE_PPM

HONEST_SHARDS = 15
ATTACKER_TOTAL_DEMAND = 4_000
CLAIM_INFLATION = 20
ATTACKER_IDENTITY_COUNTS = (1, 4, 8, 12, 15, 16, 24, 32)
POLICIES = (
    ("equal", 0, None),
    ("weighted_raw_claims", 0, None),
    ("equal_floor50_raw_claims", 500_000, None),
    ("equal_floor50_cap2x_median", 500_000, 2),
    ("equal_floor50_cap4x_median", 500_000, 4),
    ("oracle_equal_floor50_true_backlog", 500_000, "oracle"),
)

def split_exact(total: int, parts: int) -> list[int]:
    """Split integer workload deterministically without creating additional work."""
    q, r = divmod(total, parts)
    return [q + (1 if i < r else 0) for i in range(parts)]

def make_case(attacker_ids: int) -> dict:
    if attacker_ids < 1:
        raise ValueError("attacker_ids must be >= 1")
    ids = [f"attacker-{i:03d}" for i in range(attacker_ids)] + [f"honest-{i:03d}" for i in range(HONEST_SHARDS)]
    attacker_actual = split_exact(ATTACKER_TOTAL_DEMAND, attacker_ids)
    actual_by_id = {f"attacker-{i:03d}": v for i, v in enumerate(attacker_actual)}
    report_by_id = {k: min((2**63)-1, v*CLAIM_INFLATION) for k, v in actual_by_id.items()}
    for i in range(HONEST_SHARDS):
        key = f"honest-{i:03d}"
        actual_by_id[key] = 4_000
        report_by_id[key] = 4_000
    actual = [actual_by_id[k] for k in ids]
    reports = [report_by_id[k] for k in ids]
    return {"ids": ids, "actual": actual, "reports": reports,
            "attacker_ids": [f"attacker-{i:03d}" for i in range(attacker_ids)],
            "actual_by_id": actual_by_id}

def one_case(attacker_ids: int) -> dict:
    case = make_case(attacker_ids)
    ids, actual, reports = case["ids"], case["actual"], case["reports"]
    out = {}
    for name, floor, cap in POLICIES:
        claims = actual if cap == "oracle" else ([1] * len(ids) if name == "equal" else reports)
        plan = allocate(claims, ids, floor_ppm=floor, cap_multiple=None if cap == "oracle" else cap, capacity=SPENDABLE)
        m = metrics(plan, actual, ids)
        attacker_capacity = sum(plan[k] for k in case["attacker_ids"])
        attacker_actual = sum(case["actual_by_id"][k] for k in case["attacker_ids"])
        attacker_served = sum(min(plan[k], case["actual_by_id"][k]) for k in case["attacker_ids"])
        attacker_stranded = sum(max(0, plan[k] - case["actual_by_id"][k]) for k in case["attacker_ids"])
        out[name] = {
            "demand_served_ppm": m["demand_served_ppm"],
            "worst_shard_service_ppm": m["worst_shard_service_ppm"],
            "jain_fairness_ppm": m["jain_fairness_ppm"],
            "served_units": m["served_units"],
            "unserved_units": m["unserved_units"],
            "stranded_units": m["stranded_units"],
            "attacker_actual_demand_units": attacker_actual,
            "attacker_served_units": attacker_served,
            "attacker_allocated_units": attacker_capacity,
            "attacker_allocation_share_ppm": round(attacker_capacity * 1_000_000 / SPENDABLE),
            "attacker_stranded_units": attacker_stranded,
            "attacker_claimed_units": sum(case["reports"][i] for i, sid in enumerate(ids) if sid in set(case["attacker_ids"])),
        }
    return {"attacker_identity_count": attacker_ids, "total_shard_ids": len(ids),
            "honest_identity_count": HONEST_SHARDS, "actual_attacker_backlog_units": ATTACKER_TOTAL_DEMAND,
            "inflation_multiplier": CLAIM_INFLATION, "policies": out}

def run_sweep() -> dict:
    cases = [one_case(n) for n in ATTACKER_IDENTITY_COUNTS]
    canonical = json.dumps(cases, sort_keys=True, separators=(",", ":")).encode()
    return {
        "schema": "ncf-alloc4-identity-splitting-v1",
        "status": "INDEPENDENT_PYTHON_RESEARCH_MODEL_NOT_RUST_NOT_CI_NOT_PRODUCTION_EVIDENCE",
        "profile": {
            "total_capacity_units": TOTAL_CAPACITY,
            "reserve_ppm": RESERVE_PPM,
            "spendable_capacity_units": SPENDABLE,
            "honest_identity_count": HONEST_SHARDS,
            "attacker_true_backlog_units_fixed": ATTACKER_TOTAL_DEMAND,
            "attacker_claim_inflation_multiplier": CLAIM_INFLATION,
            "attacker_identity_counts": list(ATTACKER_IDENTITY_COUNTS),
            "policies": [p[0] for p in POLICIES],
        },
        "cases": cases,
        "case_set_sha256": hashlib.sha256(canonical).hexdigest(),
        "claim_ceiling": "This explicitly models one principal splitting a fixed backlog across multiple shard IDs; it does not model cryptographic identities, admission costs, governance, or real distributed execution. Median caps do not stop identity multiplication unless another protocol constrains identity creation."
    }

def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", default="ncf-alloc4-identity-splitting-results.json")
    args = parser.parse_args()
    out = run_sweep()
    out["source_sha256"] = hashlib.sha256(Path(__file__).read_bytes()).hexdigest()
    Path(args.output).write_text(json.dumps(out, indent=2, sort_keys=True) + "\n")
    print(f"output={args.output}\nsource_sha256={out['source_sha256']}\ncase_set_sha256={out['case_set_sha256']}")
    for case in out["cases"]:
        print(f"attacker_ids={case['attacker_identity_count']} total_ids={case['total_shard_ids']}")
        for name in ("equal_floor50_raw_claims", "equal_floor50_cap2x_median", "equal_floor50_cap4x_median"):
            m = case["policies"][name]
            print(f"  {name}: served={m['demand_served_ppm']/10_000:.2f}% attacker_share={m['attacker_allocation_share_ppm']/10_000:.2f}% attacker_stranded={m['attacker_stranded_units']}")

if __name__ == "__main__": main()
