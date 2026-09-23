#!/usr/bin/env python3
import hashlib
import json
from pathlib import Path

ROOT = Path("mycelix-civic/fixtures/public_integrity/v0.1")
FILES = {
    "manifest": ROOT / "manifest.json",
    "visible": ROOT / "visible_cases.json",
    "expected": ROOT / "expected.json",
    "hidden": ROOT / "hidden_evaluator.json",
}


def load(name):
    with FILES[name].open("r", encoding="utf-8") as f:
        return json.load(f)


def index(records, key="case_id"):
    out = {}
    for record in records:
        ident = record[key]
        if ident in out:
            raise AssertionError(f"duplicate {key}: {ident}")
        out[ident] = record
    return out


def diff_keys(a, b):
    keys = set(a) | set(b)
    return {k for k in keys if a.get(k) != b.get(k)}


def semantic_record(record):
    return {k: v for k, v in record.items() if k != "case_id"}


manifest = load("manifest")
visible_doc = load("visible")
expected_doc = load("expected")
hidden_doc = load("hidden")

assert manifest["synthetic_only"] is True
assert manifest["real_person_data"] is False
assert manifest["authority"] == "EvaluationOnly"
assert manifest["solver_visible_files"] == ["visible_cases.json"]
assert set(manifest["evaluator_only_files"]) == {"expected.json", "hidden_evaluator.json"}
assert hidden_doc["evaluator_only"] is True
assert hidden_doc["forbidden_as_solver_input"] is True

visible = index(visible_doc["cases"])
expected = index(expected_doc["expected"])
hidden = index(hidden_doc["cases"])
manifest_ids = manifest["case_ids"]
assert len(manifest_ids) == len(set(manifest_ids)), "duplicate manifest case id"
assert set(manifest_ids) == set(visible) == set(expected) == set(hidden)

# Evaluator-only fields must never leak into solver-visible input.
forbidden_visible_keys = {
    "protected_payload",
    "hidden_truth_note",
    "hidden_entity_relation",
    "hidden_causal_relation",
    "hidden_register_universe_contains_other_unobserved_records",
    "irrelevant_label_class",
    "sensitivity_truth",
    "historical_truth",
}

def walk_keys(value):
    if isinstance(value, dict):
        for k, v in value.items():
            yield k
            yield from walk_keys(v)
    elif isinstance(value, list):
        for item in value:
            yield from walk_keys(item)

assert forbidden_visible_keys.isdisjoint(set(walk_keys(visible_doc)))

# Expected outputs must stay below actor judgment / electoral / sanction authority.
forbidden_fragments = (
    "corrupt", "guilty", "bribed", "votefor", "vote_for", "ranking",
    "integrityscore", "integrity_score", "sanctionauthority", "sanction_authority",
    "removalauthority", "removal_authority"
)
expected_text = json.dumps(expected_doc, sort_keys=True).lower().replace(" ", "")
for fragment in forbidden_fragments:
    assert fragment not in expected_text, f"forbidden expected-output fragment: {fragment}"

# Required bounded negative controls.
assert expected["PI-006"]["disposition"] == "DisclosureNotObservedInFrontier"
assert expected["PI-005"]["disposition"] == "CausalInfluenceNotEstablished"
assert visible["PI-010"]["correction"]["supersedes"] == visible["PI-010"]["historical_observation"]["observation_id"]
assert expected["PI-010"]["disposition"] == "HistoricalObservationSupersededByCorrection"

# Metamorphic invariance / positive-sensitivity contracts.
for relation in expected_doc["metamorphic_relations"]:
    left_id = relation["left"]
    right_id = relation["right"]
    left_visible = visible[left_id]
    right_visible = visible[right_id]
    observed_visible_diffs = diff_keys(left_visible, right_visible)
    assert observed_visible_diffs == set(relation["allowed_visible_differences"]), (
        relation["kind"], observed_visible_diffs, relation["allowed_visible_differences"]
    )

    left_hidden = hidden[left_id]
    right_hidden = hidden[right_id]
    observed_hidden_diffs = diff_keys(left_hidden, right_hidden) - {"case_id"}
    assert observed_hidden_diffs == set(relation["required_hidden_differences"]), (
        relation["kind"], observed_hidden_diffs, relation["required_hidden_differences"]
    )

    left_sem = semantic_record(expected[left_id])
    right_sem = semantic_record(expected[right_id])
    if relation["expect_same_semantics"]:
        assert left_sem == right_sem, f"{relation['kind']} semantic drift"
    else:
        assert left_sem != right_sem, f"{relation['kind']} failed positive sensitivity"

# Specific privacy noninterference guarantee.
assert visible["PI-007A"] | {"case_id": "PI-007B"} == visible["PI-007B"]
assert hidden["PI-007A"]["protected_payload"] != hidden["PI-007B"]["protected_payload"]
assert hidden["PI-007A"]["allowed_public_predicate"] == hidden["PI-007B"]["allowed_public_predicate"]

# Emit a deterministic receipt fragment for exact file bytes.
print("CIV-REP-005A v0.1 static oracle: PASS")
for name, path in FILES.items():
    digest = hashlib.sha256(path.read_bytes()).hexdigest()
    print(f"sha256 {name} {digest} {path}")
print(f"cases {len(manifest_ids)}")
print(f"metamorphic_relations {len(expected_doc['metamorphic_relations'])}")
