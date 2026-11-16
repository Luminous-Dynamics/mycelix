#!/usr/bin/env python3
"""Independent structural and digest verifier for D6S-CANON-1 fixture bytes.

This is a CI/test verifier only; it does not import or execute production COS code.
"""
import hashlib
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
MANIFEST_PATH = ROOT / "docs/integral/d6s-canon-1-manifest.json"


def load_json(path: Path):
    try:
        return json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise SystemExit(f"{path.relative_to(ROOT)}: cannot parse fixture: {exc}") from exc


manifest = load_json(MANIFEST_PATH)
if manifest.get("schema_version") != "d6s-canon-1-manifest/1":
    raise SystemExit("D6S-CANON-1 manifest schema mismatch")
fixture_path = ROOT / manifest["fixture_path"]
raw = fixture_path.read_bytes()
actual_digest = hashlib.sha256(raw).hexdigest()
expected_digest = manifest.get("fixture_sha256")
if actual_digest != expected_digest:
    raise SystemExit(
        f"D6S-CANON-1 fixture SHA-256 mismatch: expected {expected_digest}, actual {actual_digest}"
    )
fixture = json.loads(raw)
if fixture.get("schema_version") != "d6s-canon-1-golden-vectors/1":
    raise SystemExit("D6S-CANON-1 fixture schema mismatch")
if fixture.get("status") != "ReferenceModelOnly":
    raise SystemExit("D6S-CANON-1 claim status must remain ReferenceModelOnly")
if fixture.get("corpus_id") != manifest.get("corpus_id") or fixture.get("corpus_id") != "COS-CONF-001":
    raise SystemExit("D6S-CANON-1 corpus identity mismatch")
vectors = fixture.get("vectors")
if not isinstance(vectors, list) or len(vectors) != manifest.get("expected_vector_count") or len(vectors) != 16:
    raise SystemExit("D6S-CANON-1 vector count mismatch")
ids = [v.get("test_id") for v in vectors]
expected_ids = [f"COS-N-{i:03d}" for i in range(1, 17)]
if ids != expected_ids:
    raise SystemExit("D6S-CANON-1 IDs must be ordered COS-N-001 through COS-N-016")
if len(set(ids)) != len(ids):
    raise SystemExit("D6S-CANON-1 duplicate vector ID")
expected_negative = {f"COS-N-{i:03d}": "Rejected" for i in range(1, 17)}
expected_negative["COS-N-005"] = "Stale"
expected_negative["COS-N-014"] = "Unauthorized"
for vector in vectors:
    test_id = vector["test_id"]
    if vector.get("expected_negative") != expected_negative[test_id]:
        raise SystemExit(f"{test_id}: frozen negative disposition mismatch")
    if vector.get("expected_positive") != "Accepted":
        raise SystemExit(f"{test_id}: positive counterpart must be explicit Accepted")
    if not vector.get("formal_obligations"):
        raise SystemExit(f"{test_id}: no formal obligation reference")
    if not vector.get("claim_ceiling", "").strip():
        raise SystemExit(f"{test_id}: missing claim ceiling")
covered = {f for vector in vectors for f in vector["formal_obligations"]}
expected_formal = {f"COS-FV-{i:03d}" for i in range(1, 11)}
if covered != expected_formal:
    raise SystemExit(f"formal-obligation coverage mismatch: covered={sorted(covered)}")
if fixture.get("expected_untested_formal_obligations") != []:
    raise SystemExit("expected_untested_formal_obligations must be empty")
if manifest.get("rust_test_target") != "d6s_canon_1_golden_vectors":
    raise SystemExit("D6S-CANON-1 Rust target mapping mismatch")
if "no physical" not in manifest.get("claim_ceiling", ""):
    raise SystemExit("D6S-CANON-1 manifest claim ceiling was weakened")
print(f"D6S-CANON-1 structural verifier: PASS vectors={len(vectors)} sha256={actual_digest} claim_ceiling=ReferenceModelOnly")
