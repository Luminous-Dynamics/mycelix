#!/usr/bin/env python3
"""Generate deterministic selection/censoring mutation properties.

Research fixture only. The generated corpus is a bounded metamorphic campaign;
it is not qualification evidence.
"""
from __future__ import annotations

import copy
import hashlib
import json
import sys
from pathlib import Path

SEED = 0x53435601
VERSION = "selection-censoring-v1"
ATTEMPT_IDS = ["a01", "a02", "a03", "a04", "a05", "a06"]


def git_blob_sha(path: Path) -> str:
    data = path.read_bytes()
    return hashlib.sha1(f"blob {len(data)}\0".encode("ascii") + data).hexdigest()


def canonical(value):
    return json.dumps(value, ensure_ascii=False, sort_keys=True, separators=(",", ":")).encode("utf-8")


def digest(value):
    return "sha256:" + hashlib.sha256(canonical(value)).hexdigest()


def semantic_normalize(case):
    out = copy.deepcopy(case)
    out["attempts"] = sorted(out["attempts"], key=lambda x: x["id"])
    out["decision_points"] = sorted(out["decision_points"], key=lambda x: x["id"])
    out["analysis"]["included_attempt_ids"] = sorted(out["analysis"]["included_attempt_ids"])
    out["analysis"]["trigger_denominator_ids"] = sorted(out["analysis"]["trigger_denominator_ids"])
    return out


def rng_step(state: int) -> int:
    state ^= (state << 13) & 0xFFFFFFFF
    state ^= state >> 17
    state ^= (state << 5) & 0xFFFFFFFF
    return state & 0xFFFFFFFF


def main() -> int:
    if len(sys.argv) != 4:
        print(
            "usage: generate_selection_censoring_properties.py POLICY.json FIXTURES.json OUTPUT.json",
            file=sys.stderr,
        )
        return 2

    policy_path, fixtures_path, output_path = map(Path, sys.argv[1:4])
    fixed = json.loads(fixtures_path.read_text(encoding="utf-8"))
    policy_path.read_text(encoding="utf-8")

    actual_sha = git_blob_sha(policy_path)
    if fixed.get("policy_binding", {}).get("git_blob_sha") != actual_sha:
        print("policy binding mismatch", file=sys.stderr)
        return 1

    base = fixed["base_case"]
    cases = []
    state = SEED

    for i in range(32):
        state = rng_step(state)
        mutation = [["reverse_collection", "attempts"]]
        if state & 1:
            mutation.append(["reverse_collection", "decision_points"])
        cases.append({
            "case_id": f"SEL-GEN-ORDER-{i:03d}",
            "property": "representation_invariance",
            "mutation": mutation,
            "expected_verdict": "qualified",
        })

    for i in range(32):
        target = ATTEMPT_IDS[i % len(ATTEMPT_IDS)]
        cases.append({
            "case_id": f"SEL-GEN-MISSING-ATTEMPT-{i:03d}",
            "property": "attempt_census_completeness",
            "mutation": [["remove_attempt", target]],
            "expected_verdict": "unresolved",
        })

    for i in range(32):
        drop = ATTEMPT_IDS[(i + 3) % len(ATTEMPT_IDS)]
        survivors = [x for x in ATTEMPT_IDS if x != drop]
        cases.append({
            "case_id": f"SEL-GEN-SURVIVOR-{i:03d}",
            "property": "silent_survivor_selection_rejection",
            "mutation": [
                ["set_analysis", "selection_rule", "SurvivorsAtHorizon"],
                ["set_analysis", "included_attempt_ids", survivors],
            ],
            "expected_verdict": "unqualified",
        })

    for i in range(32):
        target = ATTEMPT_IDS[(i + 1) % len(ATTEMPT_IDS)]
        cases.append({
            "case_id": f"SEL-GEN-INFORMATIVE-{i:03d}",
            "property": "informative_censoring_block",
            "mutation": [
                ["set_attempt", target, "terminal_state", "ActionTerminated"],
                ["set_attempt", target, "observation_end_epoch", "t1"],
                ["set_attempt", target, "observation_at_horizon", False],
                ["set_attempt", target, "censoring_reason", "ActionInducedCensoring"],
                ["set_attempt", target, "action_induced_censoring", True],
                ["set_attempt", target, "horizon_completed", False],
            ],
            "expected_verdict": "unresolved",
        })

    for i in range(32):
        if i % 2 == 0:
            mutation = [["set_analysis", "classification_timing", "post_outcome"]]
        else:
            ids = ["d01", "d02", "d03", "d04", "d05", "d06", "d07", "d08"]
            mutation = [["set_analysis", "trigger_denominator_ids", ids[:-1]]]
        cases.append({
            "case_id": f"SEL-GEN-TIMING-DENOM-{i:03d}",
            "property": "time_or_denominator_rejection",
            "mutation": mutation,
            "expected_verdict": "unqualified",
        })

    for i in range(32):
        target = ATTEMPT_IDS[i % len(ATTEMPT_IDS)]
        patterns = [
            [
                ["set_analysis", "complete_case_filter", True],
                ["set_analysis", "included_attempt_ids", [x for x in ATTEMPT_IDS if x != "a04"]],
                ["set_analysis", "selection_rule", "SurvivorsAtHorizon"],
            ],
            [
                ["set_attempt", target, "terminal_state", "ActionTerminated"],
                ["set_attempt", target, "observation_end_epoch", "t1"],
                ["set_attempt", target, "observation_at_horizon", False],
                ["set_attempt", target, "censoring_reason", "ActionInducedCensoring"],
                ["set_attempt", target, "action_induced_censoring", True],
                ["set_attempt", target, "horizon_completed", False],
                ["set_analysis", "classification_timing", "post_outcome"],
            ],
            [
                ["remove_attempt", target],
                ["set_analysis", "complete_case_filter", True],
                ["set_analysis", "classification_timing", "post_outcome"],
            ],
            [
                ["set_analysis", "estimand", "SurvivorConditionalValue"],
                ["set_analysis", "selection_rule", "SurvivorsAtHorizon"],
                ["set_analysis", "included_attempt_ids", ["a01", "a02", "a05"]],
                ["set_analysis", "complete_case_filter", True],
                ["set_analysis", "trigger_denominator_ids", ["d01", "d02", "d03", "d04", "d05", "d06", "d07"]],
            ],
        ]
        mutation = patterns[i % len(patterns)]
        expected = "unqualified" if i % len(patterns) != 2 else "unresolved"
        cases.append({
            "case_id": f"SEL-GEN-COMPOSE-{i:03d}",
            "property": "compositional_nonpositivity",
            "mutation": mutation,
            "expected_verdict": expected,
        })

    corpus = {
        "schema": "mycelix.continual-adaptation.selection-censoring-generated-properties.v1",
        "status": "research-fixture-only",
        "generator": {"version": VERSION, "seed": SEED, "mutation_count": len(cases)},
        "policy_binding": fixed["policy_binding"],
        "protocol": fixed["protocol"],
        "base_case": base,
        "cases": cases,
    }

    for case in cases:
        mutated = copy.deepcopy(base)
        for op in case["mutation"]:
            if op[0] == "reverse_collection":
                mutated[op[1]].reverse()
            elif op[0] == "set_analysis":
                mutated["analysis"][op[1]] = op[2]
            elif op[0] == "set_attempt":
                for attempt in mutated["attempts"]:
                    if attempt["id"] == op[1]:
                        attempt[op[2]] = op[3]
                        break
            elif op[0] == "remove_attempt":
                mutated["attempts"] = [a for a in mutated["attempts"] if a["id"] != op[1]]
        case["expected_semantic_digest_sha256"] = digest(semantic_normalize(mutated))

    output_path.write_text(json.dumps(corpus, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(f"generated={len(cases)} seed={SEED} version={VERSION}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
