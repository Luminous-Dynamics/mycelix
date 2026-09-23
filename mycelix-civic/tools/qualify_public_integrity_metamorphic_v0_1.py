#!/usr/bin/env python3
import copy
import hashlib
import json
from collections import Counter
from pathlib import Path

BASE = Path("mycelix-civic/fixtures/public_integrity/v0.1")
PROFILE_PATH = Path("mycelix-civic/fixtures/public_integrity/metamorphic/v0.1/profile.json")

BASE_FILES = {
    "manifest.json": BASE / "manifest.json",
    "visible_cases.json": BASE / "visible_cases.json",
    "expected.json": BASE / "expected.json",
    "hidden_evaluator.json": BASE / "hidden_evaluator.json",
}


def load(path):
    with path.open("r", encoding="utf-8") as f:
        return json.load(f)


def sha256_bytes(data):
    return hashlib.sha256(data).hexdigest()


def sha256_file(path):
    return sha256_bytes(path.read_bytes())


def index(records):
    out = {}
    for record in records:
        cid = record["case_id"]
        if cid in out:
            raise AssertionError(f"duplicate case_id {cid}")
        out[cid] = record
    return out


def diff_keys(a, b):
    return {k for k in set(a) | set(b) if a.get(k) != b.get(k)}


def derive_token(profile, kind, ordinal, base_id):
    material = f"{profile['profile_id']}|{profile['seed']}|{kind}|{ordinal}|{base_id}".encode()
    return hashlib.sha256(material).hexdigest()


profile = load(PROFILE_PATH)
visible = index(load(BASE_FILES["visible_cases.json"])["cases"])
expected = index(load(BASE_FILES["expected.json"])["expected"])
hidden = index(load(BASE_FILES["hidden_evaluator.json"])["cases"])

assert profile["synthetic_only"] is True
assert profile["base_product_head"] == "7a5af22d2fff616a777a57baab8b4324dfe96b9e"
assert profile["base_product_tree"] == "5dc7bd7e6831bf7f8d7864fc402679594a1375aa"
for filename, wanted in profile["base_fixture_sha256"].items():
    actual = sha256_file(BASE_FILES[filename])
    assert actual == wanted, (filename, wanted, actual)


def generate():
    generated = []
    for cls in profile["classes"]:
        kind = cls["kind"]
        bases = cls["base_case_ids"]
        for ordinal in range(cls["count"]):
            base_id = bases[ordinal % len(bases)]
            token = derive_token(profile, kind, ordinal, base_id)
            gid = f"GEN-{kind}-{ordinal:03d}-{token[:10]}"
            base_visible = visible[base_id]
            new_visible = copy.deepcopy(base_visible)
            new_visible["case_id"] = gid
            evaluator_delta = {}

            if kind == "PoliticalLabelInvariance":
                assert "party_label" in new_visible and "office_holder" in new_visible
                new_visible["party_label"] = f"{cls['synthetic_party_prefix']} {token[:12]}"
                new_visible["office_holder"] = f"{cls['synthetic_name_prefix']} {token[12:24]}"
                exp = expected[base_id]["disposition"]

            elif kind == "ProtectedPayloadNoninterference":
                base_hidden = hidden[base_id]
                evaluator_delta["protected_payload"] = f"{cls['synthetic_payload_prefix']}-{token[:24]}"
                assert evaluator_delta["protected_payload"] != base_hidden["protected_payload"]
                exp = cls["expected_disposition"]

            elif kind == "RecusalParticipationSensitivity":
                for key, value in cls["mutation"].items():
                    new_visible[key] = value
                exp = cls["expected_disposition"]
                assert exp != expected[base_id]["disposition"]

            elif kind == "MatterRelationSensitivity":
                for key, value in cls["mutation"].items():
                    new_visible[key] = value
                exp = cls["expected_disposition"]
                assert exp != expected[base_id]["disposition"]

            else:
                raise AssertionError(f"unsupported mutation class {kind}")

            observed_visible_diffs = diff_keys(base_visible, new_visible)
            assert observed_visible_diffs == set(cls["allowed_visible_differences"]), (
                kind, observed_visible_diffs, cls["allowed_visible_differences"]
            )

            if cls["relation"] == "SemanticallyEquivalent":
                if cls["expected_disposition"] == "InheritBase":
                    assert exp == expected[base_id]["disposition"]
            elif cls["relation"] == "LoadBearingChange":
                assert exp != expected[base_id]["disposition"]
            else:
                raise AssertionError(f"unknown relation {cls['relation']}")

            if "office_holder" in new_visible:
                assert new_visible["office_holder"].startswith(("Person ", "Synthetic Person "))
            if "party_label" in new_visible:
                assert new_visible["party_label"].startswith(("Civic ", "Synthetic Civic Group "))

            generated.append({
                "generated_case_id": gid,
                "base_case_id": base_id,
                "mutation_class": kind,
                "mutation_ordinal": ordinal,
                "relation": cls["relation"],
                "visible": new_visible,
                "evaluator_delta": evaluator_delta,
                "expected_disposition": exp,
            })

    return generated


def canonical_bytes(generated):
    document = {
        "profile_id": profile["profile_id"],
        "version": profile["version"],
        "seed": profile["seed"],
        "generated_cases": generated,
    }
    return (json.dumps(document, sort_keys=True, separators=(",", ":"), ensure_ascii=False) + "\n").encode("utf-8")


generated_a = generate()
generated_b = generate()
bytes_a = canonical_bytes(generated_a)
bytes_b = canonical_bytes(generated_b)
assert bytes_a == bytes_b, "generator is not deterministic"

assert len(generated_a) == profile["generated_case_count"] == 288
counts = Counter(item["mutation_class"] for item in generated_a)
profile_counts = {cls["kind"]: cls["count"] for cls in profile["classes"]}
assert dict(counts) == profile_counts
load_bearing = sum(1 for item in generated_a if item["relation"] == "LoadBearingChange")
assert load_bearing == 64
assert len({item["generated_case_id"] for item in generated_a}) == len(generated_a)

forbidden = tuple(x.lower() for x in profile["forbidden_outputs"])
for item in generated_a:
    expected_value = item["expected_disposition"].lower()
    assert all(fragment not in expected_value for fragment in forbidden)

corpus_sha = sha256_bytes(bytes_a)
print("CIV-REP-005M1 deterministic metamorphic generator: PASS")
print(f"profile_sha256 {sha256_file(PROFILE_PATH)}")
print(f"generated_corpus_sha256 {corpus_sha}")
print(f"generated_cases {len(generated_a)}")
for kind in sorted(counts):
    print(f"class_count {kind} {counts[kind]}")
print(f"load_bearing_cases {load_bearing}")
print(f"seed {profile['seed']}")
