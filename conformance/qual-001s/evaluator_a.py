#!/usr/bin/env python3
"""QUAL-001S evaluator A: structural and canonical-input checker.

This evaluator reads only the published corpus/schema. It does not import
Mycelix crates, execute candidate verifier code, or publish authority.
"""
import json
import re
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent
SCHEMA = ROOT / "qualification_vectors_v1.schema.json"
CORPUS = ROOT / "qualification_vectors_v1.json"

def reject_duplicates(pairs):
    out = {}
    for key, value in pairs:
        if key in out:
            raise ValueError(f"duplicate JSON object key: {key}")
        out[key] = value
    return out

def fail(message: str) -> None:
    print(f"FAIL: {message}")
    raise SystemExit(1)

try:
    schema = json.loads(SCHEMA.read_text(encoding="utf-8"), object_pairs_hook=reject_duplicates)
    corpus = json.loads(CORPUS.read_text(encoding="utf-8"), object_pairs_hook=reject_duplicates)
except Exception as exc:
    fail(f"canonical JSON parse error: {exc}")

if corpus.get("schema") != "mycelix.qual-001s.semantic-corpus-v1":
    fail("wrong corpus schema")
if corpus.get("corpus_id") != "QUAL-001S":
    fail("wrong corpus id")
if corpus.get("revision") != "v1":
    fail("wrong corpus revision")
if corpus.get("qualification_target") != "mycelix.qual.static-subject-independence.v0.3":
    fail("wrong qualification target")

vectors = corpus.get("vectors")
if not isinstance(vectors, list) or len(vectors) != 20:
    fail("expected exactly 20 vectors")

vector_ids = [v.get("vector_id") for v in vectors]
if len(set(vector_ids)) != len(vector_ids):
    fail("duplicate vector_id")

claims = set(schema["$defs"]["claim"]["enum"])
states = set(schema["$defs"]["state"]["enum"])
authority = set(schema["$defs"]["authority"]["enum"])
kinds = {"positive", "negative", "metamorphic", "provenance", "availability"}
id_re = re.compile(r"^[A-Z0-9][A-Z0-9._:-]{2,127}$")

required = {
    "vector_id", "kind", "proposition_id", "source_identity", "epoch_id",
    "requested_claims", "admitted_claims", "theorem", "upstream_evidence",
    "evidence_disposition", "authority_outcome", "expected_state",
    "expected_claims", "nonclaims",
}

for v in vectors:
    if set(v) - required - {"mutation"}:
        fail(f"{v.get('vector_id')}: unknown fields")
    if not required.issubset(v):
        fail(f"{v.get('vector_id')}: missing required field")
    if not isinstance(v["vector_id"], str) or not id_re.fullmatch(v["vector_id"]):
        fail(f"{v.get('vector_id')}: invalid id")
    if v["kind"] not in kinds:
        fail(f"{v['vector_id']}: invalid kind")
    for field in ("requested_claims", "admitted_claims", "expected_claims"):
        xs = v[field]
        if not isinstance(xs, list) or len(xs) != len(set(xs)):
            fail(f"{v['vector_id']}: {field} must be a unique list")
        if any(x not in claims for x in xs):
            fail(f"{v['vector_id']}: invalid claim token")
    if not states.issuperset({v["evidence_disposition"], v["expected_state"]}):
        fail(f"{v['vector_id']}: invalid state")
    if v["authority_outcome"] not in authority:
        fail(f"{v['vector_id']}: invalid authority outcome")
    if not isinstance(v["upstream_evidence"], list) or not v["upstream_evidence"]:
        fail(f"{v['vector_id']}: missing upstream evidence")
    if len(v["upstream_evidence"]) != len(set(v["upstream_evidence"])):
        fail(f"{v['vector_id']}: duplicate upstream evidence")
    if not isinstance(v["nonclaims"], list) or not v["nonclaims"]:
        fail(f"{v['vector_id']}: missing nonclaims")
    if len(v["nonclaims"]) != len(set(v["nonclaims"])):
        fail(f"{v['vector_id']}: duplicate nonclaims")
    if v["admitted_claims"] != v["expected_claims"]:
        fail(f"{v['vector_id']}: admitted_claims != expected_claims")

n10 = next(v for v in vectors if v["vector_id"] == "QUALS-N-010")
fixture = n10.get("mutation", {}).get("fixture")
if fixture != '{"authority_outcome":"NONE","authority_outcome":"AUTHORITY_AUTHORIZED"}':
    fail("QUALS-N-010 fixture missing or changed")
try:
    json.loads(fixture, object_pairs_hook=reject_duplicates)
except ValueError as exc:
    if "duplicate JSON object key" not in str(exc):
        fail(f"duplicate-key fixture failed for unexpected reason: {exc}")
else:
    fail("QUALS-N-010 duplicate-key fixture was accepted")

print("QUAL-001S evaluator A: PASS")
