#!/usr/bin/env python3
"""NCF-ALLOC-3: adversarial demand and bounded-influence allocation model.

Independent research harness. Not production code, not Rust/CI, not distributed
protocol evidence. It benchmarks whether capping unauthenticated demand claims
bounds individual influence without destroying legitimate service to hot shards.
"""
from __future__ import annotations
import argparse, hashlib, json, random
from pathlib import Path
from statistics import mean

SHARDS = 16
TOTAL_CAPACITY = 100_000
RESERVE_PPM = 50_000
SPENDABLE = TOTAL_CAPACITY * (1_000_000 - RESERVE_PPM) // 1_000_000
SEED = 0x4D5943454C495833
FLOOR_PPM = 500_000

def largest_remainder(total: int, weights: list[int], ids: list[str]) -> list[int]:
    if total < 0 or not weights or len(weights) != len(ids):
        raise ValueError("invalid total or mismatched/empty weights and IDs")
    if any(w < 0 for w in weights): raise ValueError("weights cannot be negative")
    if len(set(ids)) != len(ids): raise ValueError("shard IDs must be unique")
    denom = sum(weights)
    if denom == 0: weights, denom = [1] * len(weights), len(weights)
    floors = [(total * w) // denom for w in weights]
    remainders = [(total * w) % denom for w in weights]
    remaining = total - sum(floors)
    order = sorted(range(len(weights)), key=lambda i: (-remainders[i], ids[i]))
    for i in order[:remaining]: floors[i] += 1
    assert sum(floors) == total
    return floors

def lower_median(values: list[int]) -> int:
    if not values: raise ValueError("median requires at least one value")
    ordered = sorted(values)
    return ordered[(len(ordered) - 1) // 2]

def capped_claims(claims: list[int], cap_multiple: int) -> list[int]:
    """Cap each claim at a multiple of the lower median. Not Sybil resistant."""
    if cap_multiple < 1: raise ValueError("cap_multiple must be >= 1")
    if not claims or any(v < 0 for v in claims): raise ValueError("claims must be non-empty and nonnegative")
    baseline = lower_median(claims)
    if baseline == 0:
        positive = [v for v in claims if v > 0]
        if not positive: return [0] * len(claims)
        baseline = lower_median(positive)
    cap = baseline * cap_multiple
    return [min(v, cap) for v in claims]

def allocate(claims: list[int], ids: list[str] | None = None, floor_ppm: int = FLOOR_PPM,
             cap_multiple: int | None = None, capacity: int = SPENDABLE) -> dict[str, int]:
    if ids is None: ids = [f"shard-{i:02d}" for i in range(len(claims))]
    if len(claims) != len(ids) or not claims: raise ValueError("claims and IDs must be non-empty and same length")
    if any(v < 0 for v in claims) or capacity < 0: raise ValueError("claims/capacity cannot be negative")
    if not 0 <= floor_ppm <= 1_000_000: raise ValueError("floor_ppm must be in [0, 1_000_000]")
    if len(set(ids)) != len(ids): raise ValueError("shard IDs must be unique")
    canonical = sorted(zip(ids, claims), key=lambda pair: pair[0])
    sorted_ids = [i for i, _ in canonical]
    sorted_claims = [c for _, c in canonical]
    weights = capped_claims(sorted_claims, cap_multiple) if cap_multiple is not None else sorted_claims
    # If there is no demand evidence, use one canonical equal allocation instead
    # of separately rounding the equal floor and discretionary pools (which can
    # otherwise differ by two units across shards despite equal claims).
    if not any(weights):
        equal = largest_remainder(capacity, [1] * len(sorted_ids), sorted_ids)
        return {sid: equal[i] for i, sid in enumerate(sorted_ids)}
    floor_pool = capacity * floor_ppm // 1_000_000
    variable_pool = capacity - floor_pool
    base = largest_remainder(floor_pool, [1] * len(sorted_ids), sorted_ids)
    variable = largest_remainder(variable_pool, weights, sorted_ids)
    plan = {sid: base[i] + variable[i] for i, sid in enumerate(sorted_ids)}
    if sum(plan.values()) != capacity: raise AssertionError("capacity conservation violated")
    return plan

def metrics(plan: dict[str, int], actual: list[int], ids: list[str] | None = None) -> dict:
    if ids is None: ids = [f"shard-{i:02d}" for i in range(len(actual))]
    if len(actual) != len(ids) or set(ids) != set(plan): raise ValueError("plan and actual demand IDs mismatch")
    if any(v < 0 for v in actual): raise ValueError("actual demand cannot be negative")
    actual_by_id = dict(zip(ids, actual))
    service_rates, per_shard = [], []
    total_served = total_requested = stranded = 0
    for sid in sorted(ids):
        requested, capacity = actual_by_id[sid], plan[sid]
        served = min(capacity, requested)
        rate = served / requested if requested else 1.0
        service_rates.append(rate)
        total_requested += requested; total_served += served; stranded += capacity - served
        per_shard.append({"shard_id": sid, "actual_demand": requested, "allocation": capacity,
                          "served": served, "unserved": requested-served, "service_ppm": round(rate*1_000_000)})
    n, rate_sum = len(service_rates), sum(service_rates)
    fairness = rate_sum**2 / (n * sum(r*r for r in service_rates)) if any(service_rates) else 1.0
    return {"requested_units": total_requested, "served_units": total_served,
            "unserved_units": total_requested-total_served, "stranded_units": stranded,
            "demand_served_ppm": round(total_served*1_000_000/total_requested) if total_requested else 1_000_000,
            "worst_shard_service_ppm": round(min(service_rates, default=1.0)*1_000_000),
            "jain_fairness_ppm": round(fairness*1_000_000), "per_shard": per_shard}

def hot_demand(epoch: int) -> list[int]:
    hot_group = (epoch // 25) % 4
    hot = set(range(hot_group*4, hot_group*4+4))
    return [16_000 if i in hot else 4_000 for i in range(SHARDS)]

def policy_catalog():
    return [("equal", 0, None), ("weighted_raw_claims", 0, None),
            ("equal_floor50_raw_claims", FLOOR_PPM, None),
            ("equal_floor50_cap2x_median", FLOOR_PPM, 2),
            ("equal_floor50_cap4x_median", FLOOR_PPM, 4)]

def run_legitimate_hotspot() -> dict:
    by_policy = {}
    for name, floor, cap in policy_catalog():
        epochs = []
        for epoch in range(100):
            actual = hot_demand(epoch)
            plan = allocate([1]*SHARDS if name == "equal" else actual, floor_ppm=floor, cap_multiple=cap)
            epochs.append(metrics(plan, actual))
        by_policy[name] = {key: round(mean(float(r[key]) for r in epochs)) for key in
            ["demand_served_ppm", "worst_shard_service_ppm", "jain_fairness_ppm", "unserved_units", "stranded_units"]}
    return {"scenario": "honest_rotating_4x_hotspot_new_rights_each_epoch",
            "claim_ceiling": "same true demand for every policy; assumes fresh rights each epoch and ignores outstanding obligations/partition transfer effects",
            "policies": by_policy}

def attack_sweep(cases: int = 20_000, seed: int = SEED) -> dict:
    rng = random.Random(seed)
    policies = [("weighted_raw_claims", 0, None), ("equal_floor50_raw_claims", FLOOR_PPM, None),
        ("equal_floor50_cap2x", FLOOR_PPM, 2), ("equal_floor50_cap4x", FLOOR_PPM, 4),
        ("oracle_equal_floor50_true_backlog", FLOOR_PPM, "oracle")]
    keys = ["demand_served_ppm", "worst_shard_service_ppm", "jain_fairness_ppm", "unserved_units", "stranded_units", "attacker_allocation_share_ppm"]
    acc = {name: {k: 0 for k in keys} for name, _, _ in policies}
    attack_counts, digest = [], hashlib.sha256()
    for case in range(cases):
        true = [rng.randint(1_000, 20_000) for _ in range(SHARDS)]
        n_attackers = rng.randint(1, min(4, SHARDS//4))
        attackers = sorted(rng.sample(range(SHARDS), n_attackers))
        multiplier = rng.randint(2, 20)
        reports = true[:]
        for i in attackers: reports[i] = min(2**63-1, true[i]*multiplier)
        ids = [f"shard-{i:02d}" for i in range(SHARDS)]
        for name, floor, cap in policies:
            claims = true if cap == "oracle" else reports
            plan = allocate(claims, ids, floor_ppm=floor, cap_multiple=None if cap == "oracle" else cap)
            m = metrics(plan, true, ids)
            attacker_alloc = sum(plan[ids[i]] for i in attackers)
            row = acc[name]
            for k in keys[:-1]: row[k] += m[k]
            row["attacker_allocation_share_ppm"] += round(attacker_alloc*1_000_000/sum(plan.values()))
            digest.update(json.dumps([case, seed, attackers, multiplier, name, m["demand_served_ppm"], m["stranded_units"], attacker_alloc], separators=(",", ":")).encode())
        attack_counts.append(n_attackers)
    return {"scenario": "randomized_minority_claim_inflation", "cases": cases, "seed": seed,
        "attacker_count_min": min(attack_counts), "attacker_count_max": max(attack_counts),
        "inflation_multiplier_min": 2, "inflation_multiplier_max": 20,
        "median_cap_assumption": "attackers <= 25% of shards, so lower median remains non-attacker-controlled in this benchmark; no Sybil identities are created",
        "policies": {name: {k: round(v/cases) for k, v in row.items()} for name, row in acc.items()},
        "result_digest_sha256": digest.hexdigest()}

def deterministic_attack_example() -> dict:
    actual, reports = [4_000]*SHARDS, [4_000]*SHARDS
    reports[0] = reports[1] = 40_000
    ids = [f"shard-{i:02d}" for i in range(SHARDS)]
    outputs = {}
    for name, floor, cap in [("weighted_raw_claims", 0, None), ("equal_floor50_raw_claims", FLOOR_PPM, None),
        ("equal_floor50_cap2x", FLOOR_PPM, 2), ("equal_floor50_cap4x", FLOOR_PPM, 4),
        ("oracle_equal_floor50_true_backlog", FLOOR_PPM, "oracle")]:
        claims = actual if cap == "oracle" else reports
        plan = allocate(claims, ids, floor_ppm=floor, cap_multiple=None if cap == "oracle" else cap)
        m = metrics(plan, actual, ids)
        m["attacker_allocation_share_ppm"] = round(sum(plan[ids[i]] for i in (0, 1))*1_000_000/SPENDABLE)
        # Keep the committed deterministic result compact; the full per-shard
        # vector remains emitted by the in-memory metrics function and is tested.
        m.pop("per_shard", None)
        outputs[name] = m
    return {"actual_demand_units_per_shard": 4000, "reported_demand_units_for_shards_00_01": 40000,
        "all_other_reported_demand_units": 4000, "policies": outputs,
        "claim_ceiling": "oracle assumes trusted queued-backlog signal; median capping bounds some influence but does not establish authenticity or Sybil resistance"}

def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", default="ncf-alloc3-results.json")
    parser.add_argument("--sweep-cases", type=int, default=20_000)
    parser.add_argument("--seed", type=int, default=SEED)
    args = parser.parse_args()
    if args.sweep_cases < 1: parser.error("--sweep-cases must be >= 1")
    params = {"shards": SHARDS, "total_capacity_units": TOTAL_CAPACITY, "reserve_ppm": RESERVE_PPM,
        "spendable_capacity_units": SPENDABLE, "equal_floor_ppm": FLOOR_PPM,
        "adversarial_cases": args.sweep_cases, "seed": args.seed,
        "attack_model": "1-4 of 16 shards inflate claims by 2-20x; actual backlog unchanged",
        "cap_rule": "per-shard cap at 2x or 4x lower median claim"}
    script_hash = hashlib.sha256(Path(__file__).read_bytes()).hexdigest()
    results = {"schema": "ncf-alloc3-research-v1",
        "status": "INDEPENDENT_PYTHON_RESEARCH_MODEL_NOT_RUST_NOT_CI_NOT_PRODUCTION_EVIDENCE",
        "source_sha256": script_hash, "profile": params, "honest_hotspot": run_legitimate_hotspot(),
        "deterministic_attack": deterministic_attack_example(), "randomized_attack_sweep": attack_sweep(args.sweep_cases, args.seed),
        "design_conclusion": "Median capping bounds some minority-claim influence when stable identities are fixed, but is not authenticity, Sybil resistance, or a replacement for verifiable queued demand. Select cap/floor only on the joint honest-demand/adversarial frontier."}
    output = Path(args.output); output.write_text(json.dumps(results, indent=2, sort_keys=True)+"\n")
    print(f"output={output}\nsource_sha256={script_hash}")
    print("honest hotspot (served % / worst shard % / fairness %):")
    for policy, m in results["honest_hotspot"]["policies"].items(): print(f"  {policy}: {m['demand_served_ppm']/10_000:.2f} / {m['worst_shard_service_ppm']/10_000:.2f} / {m['jain_fairness_ppm']/10_000:.2f}")
    print("randomized minority claim inflation (served % / stranded / attacker allocation share %):")
    for policy, m in results["randomized_attack_sweep"]["policies"].items(): print(f"  {policy}: {m['demand_served_ppm']/10_000:.2f} / {m['stranded_units']} / {m['attacker_allocation_share_ppm']/10_000:.2f}")

if __name__ == "__main__": main()
