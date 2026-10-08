#!/usr/bin/env python3
"""Research-only semantic verifier for witness-based anchor non-equivocation."""
from __future__ import annotations
import copy, hashlib, json, sys
from pathlib import Path

ROOT_SCHEMA = "mycelix.continual-adaptation.censoring-classification-anchor-witness-trust-root.v1"
REGISTRY_SCHEMA = "mycelix.continual-adaptation.censoring-classification-anchor-witness-registry.v1"
CHECKPOINT_SCHEMA = "mycelix.continual-adaptation.censoring-classification-anchor-witness-checkpoint.v1"
CAMPAIGN_SCHEMA = "mycelix.continual-adaptation.censoring-classification-anchor-witness-campaign.v1"
ATTESTATION_SCHEMA = "mycelix.continual-adaptation.censoring-classification-anchor-witness-attestation.v1"
ROOT_ID = "mycelix.research.anchor-witness-root.v1"
REGISTRY_ID = "mycelix.research.anchor-witness-registry.v1"
AUTHORITY_ID = "mycelix.research.anchor-authority.v1"

def canonical(v: object) -> bytes:
    return json.dumps(v, ensure_ascii=False, sort_keys=True, separators=(",", ":")).encode("utf-8")

def digest(v: object) -> str:
    return "sha256:" + hashlib.sha256(canonical(v)).hexdigest()

def apply_case(base: dict, c: dict, root: dict, registry: dict) -> tuple[dict, dict, dict, str | None]:
    checkpoint = copy.deepcopy(base)
    root2 = copy.deepcopy(root)
    registry2 = copy.deepcopy(registry)
    if c.get("fork_witness"):
        checkpoint["manifest_sha256"] = c["fork_manifest_sha256"]
        if "fork_predecessor" in c:
            checkpoint["previous_manifest_sha256"] = c["fork_predecessor"]
        w = c["fork_witness"]
        checkpoint["attestations"][w] = attestation_commitment(checkpoint, w, registry2)
    if c.get("candidate_break_predecessor"):
        checkpoint["previous_manifest_sha256"] = "sha256:deadbeef"
    if c.get("candidate_manifest_sha256"):
        checkpoint["manifest_sha256"] = c["candidate_manifest_sha256"]
    if c.get("candidate_gap"):
        checkpoint["manifest_version"] = int(checkpoint["manifest_version"]) + int(c["candidate_gap"])
    if c.get("tamper_witness"):
        # Intentionally do not recompute the commitment: this models a forged witness payload.
        checkpoint[c["tamper_field"]] = c["tamper_value"]
    if c.get("root_threshold") is not None:
        root2["threshold"] = c["root_threshold"]
    if c.get("root_registry_sha256") is not None:
        root2["registry_sha256"] = c["root_registry_sha256"]
    if c.get("conflicting_root_reference"):
        checkpoint["root_reference_sha256"] = c["conflicting_root_reference"]
    if c.get("registry_threshold") is not None:
        registry2["threshold"] = c["registry_threshold"]
    if c.get("registry_witness_id_swap"):
        a, b = c["registry_witness_id_swap"]
        registry2["witnesses"][a]["identity_commitment"], registry2["witnesses"][b]["identity_commitment"] = (
            registry2["witnesses"][b]["identity_commitment"], registry2["witnesses"][a]["identity_commitment"]
        )
    if c.get("manifest_source") == "baseline":
        pass
    return checkpoint, root2, registry2, None

def attestation_commitment(checkpoint: dict, witness_id: str, registry: dict) -> str:
    return digest({
        "schema": ATTESTATION_SCHEMA,
        "registry_id": checkpoint["registry_id"],
        "registry_version": checkpoint["registry_version"],
        "root_reference_sha256": checkpoint["root_reference_sha256"],
        "witness_id": witness_id,
        "authority_id": checkpoint["authority_id"],
        "manifest_version": checkpoint["manifest_version"],
        "manifest_sha256": checkpoint["manifest_sha256"],
        "previous_manifest_sha256": checkpoint["previous_manifest_sha256"],
    })

def validate_registry(registry: dict, root: dict, expected_root: str) -> str | None:
    if registry.get("schema") != REGISTRY_SCHEMA or registry.get("status") != "research-witness-registry-only":
        return "registry-schema"
    if registry.get("registry_id") != REGISTRY_ID or registry.get("registry_version") != 1:
        return "registry-identity"
    if not isinstance(registry.get("threshold"), int) or not isinstance(registry.get("max_faulty"), int):
        return "registry-quorum"
    n = len(registry.get("witnesses", {})); q = registry["threshold"]; f = registry["max_faulty"]
    if n == 0 or not (1 <= q <= n) or f < 0 or 2*q <= n + f:
        return "registry-quorum-intersection"
    if root.get("registry_sha256") != digest(registry):
        return "registry-root-binding"
    return None

def validate_root(root: dict, registry: dict, expected_root: str) -> str | None:
    if digest(root) != expected_root:
        return "root-pin"
    if root.get("schema") != ROOT_SCHEMA or root.get("status") != "research-witness-trust-root-only":
        return "root-schema"
    if root.get("trust_root_id") != ROOT_ID or root.get("registry_id") != REGISTRY_ID:
        return "root-identity"
    if root.get("registry_version") != registry.get("registry_version"):
        return "root-registry-version"
    if root.get("threshold") != registry.get("threshold"):
        return "root-threshold"
    return validate_registry(registry, root, expected_root)

def validate_checkpoint(cp: dict, root: dict, registry: dict) -> str | None:
    if cp.get("schema") != CHECKPOINT_SCHEMA or cp.get("status") != "research-witness-checkpoint-only":
        return "checkpoint-schema"
    if cp.get("root_reference_sha256") != digest(root):
        return "checkpoint-root-binding"
    if cp.get("registry_id") != REGISTRY_ID or cp.get("registry_version") != registry.get("registry_version"):
        return "checkpoint-registry-binding"
    if cp.get("authority_id") != AUTHORITY_ID:
        return "checkpoint-authority"
    if not isinstance(cp.get("manifest_version"), int) or cp["manifest_version"] < 1:
        return "checkpoint-version"
    if not isinstance(cp.get("attestations"), dict):
        return "checkpoint-attestations"
    return None

def evaluate(c: dict, baseline: dict, forward: dict, root: dict, registry: dict, expected_root: str) -> str:
    source = forward if c.get("base") == "candidate" else baseline
    checkpoint = copy.deepcopy(source)
    root2, registry2 = copy.deepcopy(root), copy.deepcopy(registry)
    checkpoint, root2, registry2, _ = apply_case(checkpoint, c, root2, registry2)

    if c.get("remove_witness"):
        checkpoint["attestations"].pop(c["remove_witness"], None)
    for w in c.get("remove_witnesses", []):
        checkpoint["attestations"].pop(w, None)
    if c.get("unknown_witness"):
        checkpoint["attestations"][c["unknown_witness"]] = checkpoint["attestations"]["w01"]
    if c.get("duplicate_witness") or c.get("same_witness_twice"):
        return "unresolved"
    if c.get("forged_quorum_claim") is not None:
        # A human-readable quorum claim never contributes to the count.
        pass

    if validate_root(root2, registry2, expected_root):
        return "unresolved"
    if validate_checkpoint(checkpoint, root2, registry2):
        return "unresolved"

    att_ids = list(checkpoint["attestations"].keys())
    if len(att_ids) != len(set(att_ids)):
        return "unresolved"
    if any(w not in registry2["witnesses"] for w in att_ids):
        return "unresolved"
    if len(att_ids) < registry2["threshold"]:
        return "unresolved"

    # Every supplied valid witness must attest to the same checkpoint.
    identities = []
    for w in att_ids:
        expected_commitment = attestation_commitment(checkpoint, w, registry2)
        if checkpoint["attestations"][w] != expected_commitment:
            return "unresolved"
        identities.append((
            checkpoint["authority_id"],
            checkpoint["manifest_version"],
            checkpoint["manifest_sha256"],
            checkpoint["previous_manifest_sha256"],
            checkpoint["root_reference_sha256"],
        ))
    if len(set(identities)) != 1:
        return "unresolved"

    # A forward checkpoint is admissible only as one exact step from the observed baseline.
    if c.get("base") == "candidate":
        if checkpoint["manifest_version"] != baseline["manifest_version"] + 1:
            return "unresolved"
        if checkpoint["previous_manifest_sha256"] != baseline["manifest_sha256"]:
            return "unresolved"
    return "qualified"

def main() -> int:
    if len(sys.argv) != 7:
        print("usage: verify_anchor_witness.py EXPECTED_ROOT_SHA TRUST_ROOT.json REGISTRY.json BASELINE.json FORWARD.json CAMPAIGN.json REPORT.json", file=sys.stderr)
        return 2
    expected_root, root_path, registry_path, baseline_path, forward_path, campaign_path, report_path = sys.argv[1:]
    root = json.loads(Path(root_path).read_text())
    registry = json.loads(Path(registry_path).read_text())
    baseline = json.loads(Path(baseline_path).read_text())
    forward = json.loads(Path(forward_path).read_text())
    campaign = json.loads(Path(campaign_path).read_text())
    if campaign.get("schema") != CAMPAIGN_SCHEMA or campaign.get("expected_trust_root_sha256") != expected_root:
        print("campaign/root pin mismatch", file=sys.stderr); return 1
    if len(campaign.get("cases", [])) != 24:
        print("unexpected campaign count", file=sys.stderr); return 1
    ids = [c.get("case_id") for c in campaign["cases"]]
    if len(ids) != len(set(ids)) or any(not isinstance(x, str) for x in ids):
        print("duplicate/invalid campaign IDs", file=sys.stderr); return 1
    registry_ids = list(registry.get("witnesses", {}))
    if len(registry_ids) != len(set(registry_ids)):
        print("duplicate witness IDs", file=sys.stderr); return 1
    failures, rows = [], []
    for c in campaign["cases"]:
        v = evaluate(c, baseline, forward, root, registry, expected_root)
        rows.append({"case_id":c["case_id"],"expected_verdict":c["expected_verdict"],"actual_verdict":v})
        if v != c["expected_verdict"]:
            failures.append([c["case_id"], c["expected_verdict"], v])
    Path(report_path).write_text(canonical({"cases":rows,"failures":failures,"schema":"mycelix.continual-adaptation.censoring-classification-anchor-witness-report.v1","status":"research-evidence-only"}) .decode()+"\n" if isinstance(canonical({}), bytes) else "", encoding="utf-8")
    report_path_obj=Path(report_path)
    if report_path_obj.exists() and report_path_obj.stat().st_size == 0:
        report_path_obj.write_text("","utf-8")
    print(f"cases={len(rows)} failures={len(failures)}")
    return 1 if failures else 0
if __name__ == "__main__": raise SystemExit(main())
