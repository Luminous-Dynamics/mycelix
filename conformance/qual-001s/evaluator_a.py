#!/usr/bin/env python3
"""QUAL-001S evaluator A: structural and canonical-input checker.

This evaluator reads only the published corpus/schema. It does not import
Mycelix crates, execute candidate verifier code, or publish authority.
"""
import hashlib
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
if not isinstance(vectors, list) or len(vectors) != 24:
    fail("expected exactly 24 vectors")

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
    if "mutation" in v:
        m = v["mutation"]
        if not isinstance(m, dict) or "operation" not in m:
            fail(f"{v['vector_id']}: malformed mutation object")
        allowed = {"operation", "target", "expected_effect", "fixture", "before_wire", "after_wire", "expected_canonical"}
        if set(m) - allowed:
            fail(f"{v['vector_id']}: unknown mutation fields")
        for key, value in m.items():
            if not isinstance(value, str) or not value:
                fail(f"{v['vector_id']}: mutation field {key} must be non-empty text")

def canonical_jcs_json(value):
    if isinstance(value, dict):
        return "{" + ",".join(
            json.dumps(str(k), ensure_ascii=False, separators=(",", ":")) + ":" + canonical_jcs_json(value[k])
            for k in sorted(value, key=lambda key: str(key).encode("utf-16-be"))
        ) + "}"
    if isinstance(value, list):
        return "[" + ",".join(canonical_jcs_json(x) for x in value) + "]"
    return json.dumps(value, ensure_ascii=False, separators=(",", ":"))

n10 = next(v for v in vectors if v["vector_id"] == "QUALS-N-010")
fixture = n10.get("mutation", {}).get("fixture")
if fixture != '{"authority_outcome":"NONE","\\u0061uthority_outcome":"AUTHORITY_AUTHORIZED"}':
    fail("QUALS-N-010 fixture missing or changed")
try:
    json.loads(fixture, object_pairs_hook=reject_duplicates)
except ValueError as exc:
    if "duplicate JSON object key" not in str(exc):
        fail(f"duplicate-key fixture failed for unexpected reason: {exc}")
else:
    fail("QUALS-N-010 duplicate-key fixture was accepted")

s0 = json.loads((ROOT / "s0_dispatch_envelope_v1.example.json").read_text(encoding="utf-8"), object_pairs_hook=reject_duplicates)
for key in ("schema", "profile", "canonicalization_profile", "epoch_id", "subject_head_sha",
            "subject_tree_sha", "current_verifier_head_sha", "proposed_bundle_sha256",
            "dispatch_nonce_hex", "dispatch_timestamp", "envelope_sha256"):
    if key not in s0:
        fail(f"S0 example missing {key}")
if s0["schema"] != "mycelix.qual-001s.s0-dispatch-envelope-v1":
    fail("wrong S0 schema")
if s0["canonicalization_profile"] != "RFC8785-JCS-IJSON-v1":
    fail("wrong S0 canonicalization profile")
if not re.fullmatch(r"^[A-Za-z0-9][A-Za-z0-9._:-]{0,127}$", s0["epoch_id"]):
    fail("invalid S0 epoch")
if s0["candidate_code_executed"] is not False:
    fail("S0 candidate execution must be false")
if not re.fullmatch(r"^[0-9a-f]{64}$", s0["envelope_sha256"]):
    fail("invalid S0 envelope commitment")
s0_preimage = dict(s0)
del s0_preimage["envelope_sha256"]
computed_envelope_sha256 = hashlib.sha256(canonical_jcs_json(s0_preimage).encode("utf-8")).hexdigest()
if computed_envelope_sha256 != s0["envelope_sha256"]:
    fail("S0 envelope commitment does not match canonical preimage")

canon = next(v for v in vectors if v["vector_id"] == "QUALS-M-021")["mutation"]
before_obj = json.loads(canon["before_wire"], object_pairs_hook=reject_duplicates)
after_obj = json.loads(canon["after_wire"], object_pairs_hook=reject_duplicates)
if canonical_jcs_json(before_obj) != canon["expected_canonical"]:
    fail("QUALS-M-021 before object canonicalized incorrectly")
if canonical_jcs_json(after_obj) != canon["expected_canonical"]:
    fail("QUALS-M-021 after object canonicalized incorrectly")
ao = json.loads((ROOT / "s0_authority_observation_v1.example.json").read_text(encoding="utf-8"), object_pairs_hook=reject_duplicates)
for key in (
    "schema", "profile", "epoch_id", "repository_id", "repository_full_name",
    "s0_workflow_source_commit_sha", "s1_workflow_source_commit_sha",
    "dispatch_envelope_sha256", "dispatch_nonce_hex", "dispatch_ref", "dispatch_input_commitment_sha256", "dispatch_input_canonical_json",
    "authenticated_principal_identity", "policy_scope_identity", "observation_timestamp", "event_type",
    "request_authentication", "actor_authorization", "event_authorization",
    "workflow_source_authentication", "dispatch_result", "run_attribution", "observation_state"
):
    if key not in ao:
        fail(f"S0 authority observation missing {key}")
if ao["schema"] != "mycelix.qual-001s.s0-authority-observation-v1":
    fail("wrong S0 authority observation schema")
if ao["event_type"] != "workflow_dispatch":
    fail("wrong S0 authority observation event type")
if ao["dispatch_result"] == "ACCEPTED" and ao["run_attribution"]["state"] == "OBSERVED":
    # This example deliberately does not assert that API acceptance implies a run.
    pass
if ao["run_attribution"]["state"] not in {"OBSERVED", "UNOBSERVED", "CONTRADICTED"}:
    fail("invalid S0 run attribution state")
if not re.fullmatch(r"^[0-9a-f]{64}$", ao["dispatch_input_commitment_sha256"]):
    fail("invalid S0 dispatch input commitment")
try:
    dispatch_inputs = json.loads(ao["dispatch_input_canonical_json"], object_pairs_hook=reject_duplicates)
except Exception as exc:
    fail(f"invalid canonical dispatch inputs: {exc}")
if not isinstance(dispatch_inputs, dict):
    fail("canonical dispatch inputs must be a JSON object")
if canonical_jcs_json(dispatch_inputs) != ao["dispatch_input_canonical_json"]:
    fail("dispatch input bytes are not canonical JCS")
if hashlib.sha256(ao["dispatch_input_canonical_json"].encode("utf-8")).hexdigest() != ao["dispatch_input_commitment_sha256"]:
    fail("dispatch input commitment does not match canonical input bytes")
if ao["dispatch_ref"] != "main":
    fail("wrong S0 dispatch ref")
if ao["run_attribution"]["state"] == "UNOBSERVED" and len(ao["run_attribution"]) != 1:
    fail("unobserved run attribution must not carry run identifiers")
for field in ("request_authentication", "actor_authorization", "event_authorization", "workflow_source_authentication"):
    d = ao[field]
    if d["state"] == "OBSERVED" and ("rule_id" not in d or "principal_identity" not in d):
        fail(f"S0 {field} lacks observed authority identity")
if ao["request_authentication"]["state"] == "OBSERVED" and ao["request_authentication"]["principal_identity"] != ao["authenticated_principal_identity"]:
    fail("top-level authenticated principal disagrees with request authentication")
if ao["dispatch_envelope_sha256"] != s0["envelope_sha256"]:
    fail("authority observation is not bound to the exact S0 envelope commitment")
for field in ("epoch_id", "repository_id", "repository_full_name", "dispatch_nonce_hex",
              "s0_workflow_source_commit_sha", "s1_workflow_source_commit_sha"):
    if ao[field] != s0[field]:
        fail(f"authority observation {field} disagrees with S0 envelope")

print("QUAL-001S evaluator A: PASS")
