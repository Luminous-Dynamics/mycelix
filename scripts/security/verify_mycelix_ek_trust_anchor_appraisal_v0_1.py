#!/usr/bin/env python3
"""Appraise EK trust-anchor authorization from an explicit reference registry."""
from __future__ import annotations

import argparse
import base64
import copy
import hashlib
import json
from pathlib import Path
from typing import Any

VERIFIER_ID = "mycelix.tpm.ek-trust-anchor-appraisal.v0.1"
REFERENCE_ANCHOR_ID = "mycelix.synthetic-ek-root.v0.1"
REFERENCE_ROOT_SHA256 = "f9dbfd812b4772854cf32096bca60947ea62164835299e1839bc44c003e46fab"
REFERENCE_REGISTRY_SOURCE_SHA256 = "52" * 32
REFERENCE_REGISTRY = {
    "registry_id": "mycelix.ek-trust-anchor-registry",
    "registry_version": "0.1.0",
    "claim_ceiling": "ReferenceModelOnly",
    "entries": [
        {
            "anchor_id": REFERENCE_ANCHOR_ID,
            "root_certificate_sha256": REFERENCE_ROOT_SHA256,
            "authorization_state": "PASS",
            "authorization_class": "synthetic-reference-fixture",
            "source_tag": REFERENCE_ANCHOR_ID,
            "notes": "Synthetic root used only for deterministic reference-model testing; never a real manufacturer trust root.",
        }
    ],
}

def canonical_hash(value: Any) -> str:
    return hashlib.sha256(
        json.dumps(value, sort_keys=True, separators=(",", ":"), ensure_ascii=False).encode()
    ).hexdigest()

def valid_hash(value: Any) -> bool:
    return isinstance(value, str) and len(value) == 64 and all(
        c in "0123456789abcdef" for c in value
    )

def decode_der(value: Any) -> bytes:
    if not isinstance(value, str):
        raise ValueError("root_certificate_der_base64 must be base64")
    try:
        return base64.b64decode(value, validate=True)
    except Exception as exc:
        raise ValueError(f"invalid root DER base64: {exc}") from exc

def result(state: str, reason: str, details: dict[str, Any] | None = None) -> dict[str, Any]:
    value: dict[str, Any] = {"verifier_id": VERIFIER_ID, "state": state, "reason": reason}
    if details is not None:
        value["details"] = details
    value["content_sha256"] = canonical_hash(value)
    return value

def registry_digest(registry: dict[str, Any]) -> str:
    return hashlib.sha256(
        (json.dumps(registry, sort_keys=True, separators=(",", ":"), ensure_ascii=False) + "
").encode()
    ).hexdigest()

def verify(manifest: dict[str, Any]) -> dict[str, Any]:
    required = {
        "profile_id", "profile_version", "claim_ceiling", "anchor_id",
        "root_certificate_der_base64", "root_certificate_sha256",
        "registry_json", "registry_sha256", "registry_source_sha256",
        "authorization_state",
    }
    missing = sorted(required - set(manifest))
    if missing:
        return result("DENY", "missing-required-fields", {"fields": missing})
    if manifest["profile_id"] != "mycelix.security.tpm.ek-trust-anchor-appraisal":
        return result("DENY", "profile-id-mismatch")
    if manifest["profile_version"] != "0.1.0":
        return result("DENY", "profile-version-mismatch")
    if manifest["claim_ceiling"] != "ReferenceModelOnly":
        return result("DENY", "claim-ceiling-mismatch")
    for field in ("root_certificate_sha256", "registry_sha256", "registry_source_sha256"):
        if not valid_hash(manifest[field]):
            return result("DENY", "digest-invalid", {"field": field})
    try:
        root = decode_der(manifest["root_certificate_der_base64"])
    except ValueError as exc:
        return result("DENY", "root-certificate-invalid", {"error": str(exc)})
    if hashlib.sha256(root).hexdigest() != manifest["root_certificate_sha256"]:
        return result("DENY", "root-certificate-digest-mismatch")
    registry = manifest["registry_json"]
    if not isinstance(registry, dict):
        return result("DENY", "registry-invalid")
    if registry.get("registry_id") != REFERENCE_REGISTRY["registry_id"]:
        return result("DENY", "registry-id-mismatch")
    if registry.get("registry_version") != "0.1.0":
        return result("DENY", "registry-version-mismatch")
    if registry.get("claim_ceiling") != "ReferenceModelOnly":
        return result("DENY", "registry-claim-ceiling-mismatch")
    if hashlib.sha256(
        (json.dumps(registry, sort_keys=True, separators=(",", ":"), ensure_ascii=False) + "
").encode()
    ).hexdigest() != manifest["registry_sha256"]:
        return result("DENY", "registry-digest-mismatch")
    if manifest["registry_sha256"] != registry_digest(REFERENCE_REGISTRY):
        return result("DENY", "reference-registry-not-approved")
    if manifest["registry_source_sha256"] != REFERENCE_REGISTRY_SOURCE_SHA256:
        return result("DENY", "registry-source-not-approved")
    entries = registry.get("entries")
    if not isinstance(entries, list):
        return result("DENY", "registry-entries-invalid")
    matching = [e for e in entries if isinstance(e, dict) and e.get("anchor_id") == manifest["anchor_id"]]
    if not matching:
        return result("INDETERMINATE", "anchor-not-listed")
    entry = matching[0]
    if entry.get("authorization_state") == "INDETERMINATE":
        return result("INDETERMINATE", "registry-entry-indeterminate")
    if entry.get("authorization_state") != manifest["authorization_state"]:
        return result("DENY", "authorization-state-mismatch")
    if entry.get("authorization_state") != "PASS":
        return result("DENY", "registry-entry-not-authorized")
    if entry.get("root_certificate_sha256") != manifest["root_certificate_sha256"]:
        return result("DENY", "registry-root-digest-mismatch")
    if entry.get("anchor_id") == REFERENCE_ANCHOR_ID and entry.get("root_certificate_sha256") != REFERENCE_ROOT_SHA256:
        return result("DENY", "reference-anchor-root-mismatch")
    if manifest["anchor_id"] != REFERENCE_ANCHOR_ID:
        return result("INDETERMINATE", "non-reference-anchor-not-authorized-by-reference-model")
    return result(
        "PASS",
        "ek-trust-anchor-policy-authorized",
        {
            "anchor_id": manifest["anchor_id"],
            "root_certificate_sha256": manifest["root_certificate_sha256"],
            "registry_sha256": manifest["registry_sha256"],
            "registry_source_sha256": manifest["registry_source_sha256"],
            "authorization_class": entry.get("authorization_class"),
        },
    )

def fixture() -> dict[str, Any]:
    root = bytes.fromhex("3003020101")
    registry_text = json.dumps(REFERENCE_REGISTRY, sort_keys=True, separators=(",", ":"), ensure_ascii=False) + "
"
    rs = hashlib.sha256(root).hexdigest()
    return {
        "profile_id": "mycelix.security.tpm.ek-trust-anchor-appraisal",
        "profile_version": "0.1.0",
        "claim_ceiling": "ReferenceModelOnly",
        "anchor_id": REFERENCE_ANCHOR_ID,
        "root_certificate_der_base64": base64.b64encode(root).decode(),
        "root_certificate_sha256": rs,
        "registry_json": REFERENCE_REGISTRY,
        "registry_sha256": hashlib.sha256(registry_text.encode()).hexdigest(),
        "registry_source_sha256": REFERENCE_REGISTRY_SOURCE_SHA256,
        "authorization_state": "PASS",
    }

def self_test() -> int:
    base = fixture()
    cases = [
        ("canonical-authorized-root","PASS",lambda x:None),
        ("root-bytes-substitution","DENY",lambda x:x.update({"root_certificate_der_base64":base64.b64encode(b"changed-root").decode(),"root_certificate_sha256":hashlib.sha256(b"changed-root").hexdigest()})),
        ("root-digest-substitution","DENY",lambda x:x.update({"root_certificate_sha256":"11"*32})),
        ("anchor-id-substitution","DENY",lambda x:x.update({"anchor_id":"other-anchor"})),
        ("registry-entry-substitution","DENY",lambda x:x["registry_json"]["entries"][0].update({"root_certificate_sha256":"22"*32})),
        ("authorization-state-deny","DENY",lambda x:x.update({"authorization_state":"DENY"})),
        ("unknown-anchor","INDETERMINATE",lambda x:x.update({"anchor_id":"unknown-anchor"})),
        ("registry-source-substitution","DENY",lambda x:x.update({"registry_source_sha256":"33"*32})),
    ]
    # The reference fixture root is deliberately tiny; it is only a deterministic policy vector.
    for name, expected, mutate in cases:
        candidate = copy.deepcopy(base)
        mutate(candidate)
        observed = verify(candidate)
        if observed["state"] != expected:
            print(f"{name}: FAIL expected={expected} got={observed['state']} reason={observed['reason']}")
            return 1
    permuted = json.loads(json.dumps(base, sort_keys=True))
    if verify(permuted)["state"] != "PASS":
        print("key-order-permutation: FAIL")
        return 1
    print("EK trust-anchor appraisal semantic corpus: PASS")
    print("8 adversarial/canonical cases plus key-order control: PASS")
    return 0

def main() -> int:
    parser = argparse.ArgumentParser()
    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument("--self-test", action="store_true")
    group.add_argument("--verify", metavar="MANIFEST")
    parser.add_argument("--output")
    args = parser.parse_args()
    if args.self_test:
        return self_test()
    path = Path(args.verify).resolve()
    manifest = json.loads(path.read_text(encoding="utf-8"))
    verified = verify(manifest)
    output = {"profile_id":"mycelix.security.tpm.ek-trust-anchor-appraisal","profile_version":"0.1.0","verifier_id":VERIFIER_ID,"input_sha256":hashlib.sha256(path.read_bytes()).hexdigest(),**verified}
    output["content_sha256"] = canonical_hash({k:v for k,v in output.items() if k!="content_sha256"})
    rendered = json.dumps(output, indent=2, sort_keys=True) + "
"
    if args.output:
        Path(args.output).write_text(rendered, encoding="utf-8")
    else:
        print(rendered, end="")
    return {"PASS":0,"DENY":1,"INDETERMINATE":2}[verified["state"]]

if __name__ == "__main__":
    raise SystemExit(main())
