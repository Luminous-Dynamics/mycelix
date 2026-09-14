#!/usr/bin/env python3
from __future__ import annotations

import argparse
import hashlib
import itertools
import json
import statistics

AUTHORITY = "MeasurementOnly"
SOURCE_SUBJECT = "fca2c107a1ea5108823ce617ba4111b6f7f77230"
COORDINATOR_BLOB = "969b845e6186cbcad507c742a718060844f82eb2"
INTEGRITY_BLOB = "658562c8dfaf6a2f1b97a7bfd5cf0fc8a5ab6e66"
GRID = (0.0, 0.25, 0.5, 0.75, 1.0)
FORMULA_A_ID = "multiplicative-bounded-v1"
FORMULA_B_ID = "additive-composite-v1"


def canonical(obj):
    return json.dumps(obj, sort_keys=True, separators=(",", ":"), allow_nan=False).encode()


def sha(obj):
    return hashlib.sha256(canonical(obj)).hexdigest()


def clamp01(value):
    return min(1.0, max(0.0, value))


def multiplicative(phi, k_trust, stake, participation, domain, provenance="Attested"):
    phi, k_trust, stake, participation, domain = map(
        clamp01, (phi, k_trust, stake, participation, domain)
    )
    reputation_squared = k_trust * k_trust
    consciousness = 1.0 if provenance == "Unavailable" else 0.7 + 0.3 * phi
    participation_bonus = 1.0 + 0.1 * participation
    stake_modifier = 1.0 + 0.05 * stake
    domain_modifier = 1.0 + 0.1 * domain
    uncapped = (
        reputation_squared
        * consciousness
        * participation_bonus
        * stake_modifier
        * domain_modifier
    )
    return min(1.5, max(0.1, uncapped))


def additive(phi, k_trust, stake, participation, domain):
    return (
        0.30 * phi
        + 0.25 * k_trust
        + 0.20 * min(stake, 1.0)
        + 0.15 * participation
        + 0.10 * domain
    )


def profiles():
    yield from itertools.product(GRID, repeat=5)


def sign(value, epsilon=1e-15):
    if value > epsilon:
        return 1
    if value < -epsilon:
        return -1
    return 0


def pairwise_order_stats(rows):
    inversions = 0
    tie_disagreements = 0
    both_tie = 0
    comparable = 0
    examples = []

    for i in range(len(rows)):
        additive_i, multiplicative_i, input_i = rows[i]
        for j in range(i + 1, len(rows)):
            additive_j, multiplicative_j, input_j = rows[j]
            additive_sign = sign(additive_i - additive_j)
            multiplicative_sign = sign(multiplicative_i - multiplicative_j)

            if additive_sign == 0 and multiplicative_sign == 0:
                both_tie += 1
            elif additive_sign == 0 or multiplicative_sign == 0:
                tie_disagreements += 1
            else:
                comparable += 1
                if additive_sign != multiplicative_sign:
                    inversions += 1
                    if len(examples) < 5:
                        examples.append(
                            {
                                "left": input_i,
                                "right": input_j,
                                "additive_delta": additive_i - additive_j,
                                "multiplicative_delta": multiplicative_i - multiplicative_j,
                            }
                        )

    return {
        "strict_comparable_pairs": comparable,
        "strict_rank_inversions": inversions,
        "tie_disagreements": tie_disagreements,
        "both_tie_pairs": both_tie,
        "rank_inversion_examples": examples,
    }


def monotonicity(formula, provenance=None):
    names = ["phi", "k_trust", "stake", "participation", "domain"]
    result = {}

    for dimension, name in enumerate(names):
        violations = 0
        flats = 0
        strict_steps = 0
        for fixed in itertools.product(GRID, repeat=4):
            values = []
            for candidate in GRID:
                fixed_iter = iter(fixed)
                full = [candidate if d == dimension else next(fixed_iter) for d in range(5)]
                if provenance is None:
                    value = formula(*full)
                else:
                    value = formula(*full, provenance=provenance)
                values.append(value)

            for left, right in zip(values, values[1:]):
                if right < left - 1e-15:
                    violations += 1
                elif abs(right - left) <= 1e-15:
                    flats += 1
                else:
                    strict_steps += 1

        result[name] = {
            "violations": violations,
            "flat_steps": flats,
            "strict_increase_steps": strict_steps,
        }

    return result


def input_object(values):
    return {
        "phi": values[0],
        "k_trust": values[1],
        "stake": values[2],
        "participation": values[3],
        "domain": values[4],
    }


def build_report():
    rows = []
    unavailable_rows = []
    for values in profiles():
        additive_weight = additive(*values)
        multiplicative_attested = multiplicative(*values, provenance="Attested")
        multiplicative_unavailable = multiplicative(*values, provenance="Unavailable")

        # Production get_voter_phi_weight() materializes Unavailable Phi as 0.0.
        # The additive composite has no provenance branch, so its live unavailable
        # semantics receive a zero Phi contribution rather than a neutral multiplier.
        additive_unavailable = additive(
            0.0, values[1], values[2], values[3], values[4]
        )

        rows.append((additive_weight, multiplicative_attested, values))
        unavailable_rows.append((additive_unavailable, multiplicative_unavailable, values))

    differences = [
        multiplicative_weight - additive_weight
        for additive_weight, multiplicative_weight, _ in rows
    ]
    absolute_differences = [abs(value) for value in differences]

    max_absolute_index = max(range(len(rows)), key=lambda i: absolute_differences[i])
    max_multiplicative_index = max(range(len(rows)), key=lambda i: differences[i])
    max_additive_index = min(range(len(rows)), key=lambda i: differences[i])

    def case(index):
        additive_weight, multiplicative_weight, values = rows[index]
        return {
            "input": input_object(values),
            "additive": additive_weight,
            "multiplicative_attested": multiplicative_weight,
            "signed_delta_m_minus_a": multiplicative_weight - additive_weight,
        }

    unavailable_phi_independent = all(
        abs(
            multiplicative(0.0, k, stake, participation, domain, "Unavailable")
            - multiplicative(1.0, k, stake, participation, domain, "Unavailable")
        )
        < 1e-15
        for k, stake, participation, domain in itertools.product(GRID, repeat=4)
    )

    unavailable_differences = [
        multiplicative_weight - additive_weight
        for additive_weight, multiplicative_weight, _ in unavailable_rows
    ]

    report = {
        "schema": "mycelix-voting-weight-differential-v1",
        "authority": AUTHORITY,
        "source_binding": {
            "git_subject": SOURCE_SUBJECT,
            "coordinator_path": "mycelix-governance/zomes/voting/coordinator/src/lib.rs",
            "coordinator_blob": COORDINATOR_BLOB,
            "integrity_path": "mycelix-governance/zomes/voting/integrity/src/lib.rs",
            "integrity_blob": INTEGRITY_BLOB,
            "binding_kind": "declared_formula_reimplementation_bound_to_observed_source_blobs",
        },
        "grid": list(GRID),
        "profile_count": len(rows),
        "formulas": {
            FORMULA_A_ID: (
                "R^2*(0.7+0.3*Phi)*(1+0.1*P)*(1+0.05*S)*(1+0.1*D), "
                "input clamp [0,1], output clamp [0.1,1.5]; Unavailable Phi uses consciousness=1.0"
            ),
            FORMULA_B_ID: "0.30*Phi+0.25*K+0.20*min(S,1.0)+0.15*P+0.10*D",
        },
        "attested_difference": {
            "mean_signed_m_minus_a": statistics.fmean(differences),
            "mean_absolute": statistics.fmean(absolute_differences),
            "max_absolute": absolute_differences[max_absolute_index],
            "max_multiplicative_over_additive": case(max_multiplicative_index),
            "max_additive_over_multiplicative": case(max_additive_index),
            "max_absolute_case": case(max_absolute_index),
            "multiplicative_floor_count": sum(
                1 for _, weight, _ in rows if abs(weight - 0.1) < 1e-15
            ),
            "multiplicative_cap_count": sum(
                1 for _, weight, _ in rows if abs(weight - 1.5) < 1e-15
            ),
            "pairwise_order": pairwise_order_stats(rows),
        },
        "unavailable_phi": {
            "source_semantics": (
                "get_voter_phi_weight sets phi_score=0.0; multiplicative uses neutral "
                "consciousness=1.0 while additive composite has no provenance branch and "
                "therefore receives zero Phi contribution"
            ),
            "multiplicative_phi_independent": unavailable_phi_independent,
            "pairwise_order_zero_phi_additive_vs_neutral_multiplicative": pairwise_order_stats(
                unavailable_rows
            ),
            "mean_signed_m_minus_a": statistics.fmean(unavailable_differences),
            "mean_absolute": statistics.fmean(
                [abs(value) for value in unavailable_differences]
            ),
        },
        "monotonicity": {
            "additive": monotonicity(additive),
            "multiplicative_attested": monotonicity(multiplicative, "Attested"),
            "multiplicative_unavailable": monotonicity(multiplicative, "Unavailable"),
        },
        "sensitivity_fixtures": {
            "stake_additive_delta_at_mid_others": (
                additive(0.5, 0.5, 1.0, 0.5, 0.5)
                - additive(0.5, 0.5, 0.0, 0.5, 0.5)
            ),
            "stake_multiplicative_delta_at_mid_others": (
                multiplicative(0.5, 0.5, 1.0, 0.5, 0.5)
                - multiplicative(0.5, 0.5, 0.0, 0.5, 0.5)
            ),
            "reputation_additive_delta_025_to_05_mid_others": (
                additive(0.5, 0.5, 0.5, 0.5, 0.5)
                - additive(0.5, 0.25, 0.5, 0.5, 0.5)
            ),
            "reputation_multiplicative_delta_025_to_05_mid_others": (
                multiplicative(0.5, 0.5, 0.5, 0.5, 0.5)
                - multiplicative(0.5, 0.25, 0.5, 0.5, 0.5)
            ),
            "zero_reputation_additive_mid_others": additive(0.5, 0.0, 0.5, 0.5, 0.5),
            "zero_reputation_multiplicative_mid_others": multiplicative(
                0.5, 0.0, 0.5, 0.5, 0.5
            ),
        },
        "non_claims": [
            "no_normative_preference_between_formulas",
            "no_sybil_resistance_claim",
            "no_fairness_claim",
            "no_policy_migration_authority",
        ],
    }
    report["result_sha256"] = sha(report)
    return report


def self_test():
    report = build_report()
    assert report["profile_count"] == 3125
    assert report["unavailable_phi"]["multiplicative_phi_independent"] is True
    for model in report["monotonicity"].values():
        assert all(axis["violations"] == 0 for axis in model.values())
    assert report["attested_difference"]["pairwise_order"]["strict_rank_inversions"] > 0
    assert report["attested_difference"]["multiplicative_cap_count"] == 0
    assert canonical(report) == canonical(build_report())
    return {
        "all_self_tests_passed": True,
        "authority": AUTHORITY,
        "result_sha256": report["result_sha256"],
        "profile_count": 3125,
        "strict_rank_inversions": report["attested_difference"]["pairwise_order"][
            "strict_rank_inversions"
        ],
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--self-test", action="store_true")
    parser.add_argument("--report", action="store_true")
    args = parser.parse_args()
    if args.self_test:
        print(json.dumps(self_test(), sort_keys=True, separators=(",", ":"), allow_nan=False))
    elif args.report:
        print(json.dumps(build_report(), sort_keys=True, separators=(",", ":"), allow_nan=False))
    else:
        parser.error("choose --self-test or --report")


if __name__ == "__main__":
    main()
