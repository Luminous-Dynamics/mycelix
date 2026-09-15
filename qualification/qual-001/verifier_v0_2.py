#!/usr/bin/env python3
from __future__ import annotations

import argparse
import hashlib
import json
import stat
import subprocess
from pathlib import Path

BUNDLE_SCHEMA = "mycelix.qual.verifier-bundle.v0.2"
MANIFEST_SCHEMA = "mycelix.qual.gate-manifest.v0.2"
RECEIPT_SCHEMA = "mycelix.qual.independent-verifier.receipt.v0.2"
ROTATION_POLICY_SCHEMA = "mycelix.qual.rotation-policy.v0.1"
POINTER_SCHEMA = "mycelix.qual.current-verifier.v0.1"

class VerificationError(RuntimeError):
    pass

def require(condition: bool, message: str) -> None:
    if not condition:
        raise VerificationError(message)

def sha256_bytes(value: bytes) -> str:
    return hashlib.sha256(value).hexdigest()

def canonical_json(value: object) -> str:
    return json.dumps(value, sort_keys=True, separators=(",", ":"))

def git(root: Path, *args: str) -> str:
    return subprocess.check_output(["git", "-C", str(root), *args], text=True, stderr=subprocess.STDOUT).strip()

def validate_rel(value: str) -> None:
    p = Path(value)
    require(value != "", "empty path")
    require(not any(ord(ch) < 0x20 or ord(ch) == 0x7f for ch in value),
        f"control character in path rejected: {value!r}")
    require(not p.is_absolute(), f"absolute path rejected: {value}")
    require(".." not in p.parts, f"path traversal rejected: {value}")

def safe_file(root: Path, rel: str) -> Path:
    validate_rel(rel)
    p = root / rel
    require(p.exists() and not p.is_symlink() and p.is_file(), f"invalid file: {rel}")
    resolved = p.resolve()
    rr = root.resolve()
    require(rr == resolved or rr in resolved.parents, f"path escaped root: {rel}")
    return p

def load_json(path: Path) -> tuple[dict, str]:
    raw = path.read_bytes()
    data = json.loads(raw)
    require(isinstance(data, dict), f"JSON object required: {path}")
    return data, sha256_bytes(raw)

def verify_hash(root: Path, entry: dict) -> None:
    require(set(entry) == {"path","sha256"}, "component must contain only path/sha256")
    rel, digest = entry["path"], entry["sha256"]
    require(isinstance(rel, str) and isinstance(digest, str) and len(digest) == 64, "invalid component")
    p = safe_file(root, rel)
    require(sha256_bytes(p.read_bytes()) == digest, f"component digest mismatch: {rel}")

def load_bundle(root: Path, path: Path) -> tuple[dict, str]:
    data, digest = load_json(path)
    require(data.get("schema") == BUNDLE_SCHEMA, "wrong bundle schema")
    require(set(data) == {"schema","profile","predecessor_bundle_sha256","launcher","components"},
            "unexpected bundle fields")
    require(isinstance(data.get("profile"), str) and data["profile"], "invalid bundle profile")
    pred = data.get("predecessor_bundle_sha256")
    require(isinstance(pred, str) and len(pred) == 64, "invalid predecessor bundle digest")
    launcher = data.get("launcher")
    components = data.get("components")
    require(isinstance(launcher, dict) and set(launcher) == {"authoritative_workflow","dispatcher"},
            "unexpected launcher census")
    require(isinstance(components, dict) and set(components) == {"gate_manifest","verifier","rotation_policy"},
            "unexpected semantic component census")
    seen = []
    for entry in list(launcher.values()) + list(components.values()):
        verify_hash(root, entry)
        seen.append(entry["path"])
    require(len(seen) == len(set(seen)), "duplicate bundle component path")
    return data, digest

def load_gate(path: Path) -> tuple[dict, str]:
    data, digest = load_json(path)
    require(data.get("schema") == MANIFEST_SCHEMA, "wrong gate schema")
    require(data.get("receipt_schema") == RECEIPT_SCHEMA, "wrong receipt schema")
    require(data.get("subject_execution") is False, "subject execution forbidden")
    require(data.get("same_repository_subject_required") is True, "same-repository subject required")
    require(isinstance(data.get("max_changed_paths"), int) and data["max_changed_paths"] > 0, "invalid path bound")
    require(isinstance(data.get("max_changed_bytes"), int) and data["max_changed_bytes"] > 0, "invalid byte bound")
    return data, digest

def load_policy(path: Path) -> tuple[dict, str]:
    data, digest = load_json(path)
    require(data.get("schema") == ROTATION_POLICY_SCHEMA, "wrong rotation policy schema")
    require(data.get("repository_protection_required") is True, "rotation must preserve protection requirement")
    require(data.get("admin_bypass_qualifies") is False, "admin bypass must not qualify")
    require(data.get("subject_code_execution") is False, "candidate code execution forbidden")
    require(data.get("rotation_requires_predecessor_bundle") is True, "predecessor binding required")
    for key in ("max_changed_paths","max_changed_bytes"):
        require(isinstance(data.get(key), int) and data[key] > 0, f"invalid {key}")
    return data, digest

def load_pointer(path: Path) -> tuple[dict, str]:
    data, digest = load_json(path)
    require(data.get("schema") == POINTER_SCHEMA, "wrong current pointer schema")
    require(set(data) == {"schema","profile","bundle_path","bundle_sha256"}, "unexpected pointer fields")
    require(isinstance(data.get("profile"), str) and data["profile"], "invalid pointer profile")
    require(isinstance(data.get("bundle_sha256"), str) and len(data["bundle_sha256"]) == 64, "invalid pointer digest")
    validate_rel(data["bundle_path"])
    return data, digest

def changed_paths(root: Path, base: str, head: str) -> list[str]:
    raw = subprocess.check_output(
        ["git","-C",str(root),"diff","--name-only","--no-renames","-z",base,head],
        stderr=subprocess.STDOUT,
    )
    values=[]
    for item in raw.split(b"\0"):
        if not item: continue
        try:
            value=item.decode("utf-8","strict")
        except UnicodeDecodeError as exc:
            raise VerificationError("non-UTF-8 changed path rejected") from exc
        validate_rel(value)
        values.append(value)
    return values

def changed_bytes(root: Path, changed: list[str], limit: int) -> int:
    total = 0
    rr = root.resolve()
    for rel in changed:
        p = root / rel
        if not p.exists() and not p.is_symlink():
            continue
        require(not p.is_symlink(), f"subject symlink rejected: {rel}")
        resolved = p.resolve()
        require(rr == resolved or rr in resolved.parents, f"subject path escaped checkout: {rel}")
        require(stat.S_ISREG(p.stat().st_mode), f"non-regular subject path: {rel}")
        total += p.stat().st_size
        require(total <= limit, "changed-byte bound exceeded")
    return total

def ordinary_forbidden(changed: list[str], gate: dict) -> bool:
    exact = set(gate.get("ordinary_forbidden_exact_paths", []))
    prefixes = tuple(gate.get("ordinary_forbidden_prefixes", []))
    return any(x in exact or x.startswith(prefixes) for x in changed)

def ordinary_gate(changed: list[str], gate: dict) -> None:
    require(not ordinary_forbidden(changed, gate), "ordinary subject attempted to change verifier-owned path")

def allowed_rotation_path(rel: str, policy: dict) -> bool:
    if rel in set(policy.get("allowed_exact_paths", [])):
        return True
    return any(rel.startswith(prefix) for prefix in policy.get("allowed_prefixes", []))

def require_new_at_base(subject_root: Path, base: str, rel: str) -> None:
    rc = subprocess.run(["git","-C",str(subject_root),"cat-file","-e",f"{base}:{rel}"],
                        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL).returncode
    require(rc != 0, f"successor path must be new: {rel}")

def validate_successor(
    verifier_root: Path,
    subject_root: Path,
    subject_base: str,
    changed: list[str],
    current_bundle: dict,
    current_bundle_sha: str,
    policy: dict,
) -> dict:
    require(len(changed) <= policy["max_changed_paths"], "rotation path bound exceeded")
    for rel in changed:
        require(allowed_rotation_path(rel, policy), f"rotation mixed unrelated path: {rel}")
    for rel in policy["immutable_launcher_paths"]:
        require(rel not in changed, f"rotation attempted launcher mutation: {rel}")

    pointer_rel = policy["current_pointer_path"]
    require(pointer_rel in changed, "rotation must update current verifier pointer")
    pointer, pointer_sha = load_pointer(subject_root / pointer_rel)
    successor_rel = pointer["bundle_path"]
    require(successor_rel in changed, "successor bundle must be part of rotation diff")
    require_new_at_base(subject_root, subject_base, successor_rel)

    successor, successor_sha = load_bundle(subject_root, subject_root / successor_rel)
    require(pointer["bundle_sha256"] == successor_sha, "pointer/successor bundle digest mismatch")
    require(pointer["profile"] == successor["profile"], "pointer/successor profile mismatch")
    require(successor["predecessor_bundle_sha256"] == current_bundle_sha, "successor predecessor mismatch")
    require(successor["profile"] != current_bundle["profile"], "successor profile must change")
    require(successor["launcher"] == current_bundle["launcher"], "normal rotation may not change launcher")

    current_historical = {entry["path"] for entry in current_bundle["components"].values()}
    current_historical.add(pointer_rel)
    current_historical.add("qualification/qual-001/verifier-bundle-v0.2.json")
    for rel in current_historical - {pointer_rel}:
        require(rel not in changed, f"rotation attempted historical semantic mutation: {rel}")

    successor_component_paths = {entry["path"] for entry in successor["components"].values()}
    semantic_rotation_paths = {pointer_rel, successor_rel, *successor_component_paths}
    for rel in changed:
        if rel.startswith("qualification/qual-001/"):
            require(rel in semantic_rotation_paths, f"unexpected qualification path in rotation: {rel}")

    for role, entry in successor["components"].items():
        rel = entry["path"]
        require(rel in changed, f"successor component missing from rotation diff: {rel}")
        require_new_at_base(subject_root, subject_base, rel)

    gate, _ = load_json(subject_root / successor["components"]["gate_manifest"]["path"])
    require(gate.get("subject_execution") is False, "successor gate may not enable subject execution")
    require(gate.get("same_repository_subject_required") is True, "successor gate must keep same-repo boundary")

    next_policy, _ = load_json(subject_root / successor["components"]["rotation_policy"]["path"])
    require(next_policy.get("repository_protection_required") is True, "successor must preserve protection requirement")
    require(next_policy.get("admin_bypass_qualifies") is False, "successor may not qualify admin bypass")
    require(next_policy.get("subject_code_execution") is False, "successor may not execute candidate code")
    require(next_policy.get("rotation_requires_predecessor_bundle") is True, "successor must preserve predecessor binding")

    return {
        "previous_bundle_sha256": current_bundle_sha,
        "proposed_bundle_sha256": successor_sha,
        "proposed_bundle_path": successor_rel,
        "proposed_profile": successor["profile"],
        "proposed_pointer_sha256": pointer_sha,
        "candidate_verifier_code_executed": False,
        "admin_bypass_qualifies": False,
    }

def verify(
    verifier_root: Path,
    subject_root: Path,
    manifest_path: Path,
    bundle_path: Path,
    current_pointer: Path,
    expected_verifier_head: str,
    subject_base: str,
    subject_head: str,
) -> dict:
    bundle, bundle_sha = load_bundle(verifier_root, bundle_path)
    gate_path = verifier_root / bundle["components"]["gate_manifest"]["path"]
    policy_path = verifier_root / bundle["components"]["rotation_policy"]["path"]
    require(manifest_path.resolve() == gate_path.resolve(), "manifest path does not match current bundle")
    gate, gate_sha = load_gate(gate_path)
    policy, policy_sha = load_policy(policy_path)
    pointer, pointer_sha = load_pointer(current_pointer)
    require(pointer["bundle_path"] == str(bundle_path.relative_to(verifier_root)), "current pointer bundle path mismatch")
    require(pointer["bundle_sha256"] == bundle_sha, "current pointer bundle digest mismatch")
    require(pointer["profile"] == bundle["profile"], "current pointer profile mismatch")

    actual_verifier = git(verifier_root, "rev-parse", "HEAD")
    actual_subject = git(subject_root, "rev-parse", "HEAD")
    require(actual_verifier == expected_verifier_head, "verifier head mismatch")
    require(actual_subject == subject_head, "subject head mismatch")
    git(subject_root, "cat-file", "-e", f"{subject_base}^{{commit}}")

    changed = changed_paths(subject_root, subject_base, subject_head)
    require(len(changed) <= gate["max_changed_paths"], "changed-path bound exceeded")
    total = changed_bytes(subject_root, changed, gate["max_changed_bytes"])
    digest = sha256_bytes(("\n".join(changed) + ("\n" if changed else "")).encode())

    mode = "ordinary"
    extra = {}
    if ordinary_forbidden(changed, gate):
        mode = "rotation_authorization"
        extra = validate_successor(verifier_root, subject_root, subject_base, changed, bundle, bundle_sha, policy)
    else:
        ordinary_gate(changed, gate)

    receipt = {
        "schema": RECEIPT_SCHEMA,
        "qualification_pass": True,
        "mode": mode,
        "subject_head": subject_head,
        "subject_tree": git(subject_root, "rev-parse", f"{subject_head}^{{tree}}"),
        "subject_base": subject_base,
        "verifier_head": actual_verifier,
        "verifier_tree": git(verifier_root, "rev-parse", "HEAD^{tree}"),
        "current_pointer_sha256": pointer_sha,
        "verifier_bundle_schema": bundle["schema"],
        "verifier_bundle_sha256": bundle_sha,
        "gate_manifest_profile": gate["profile"],
        "gate_manifest_sha256": gate_sha,
        "rotation_policy_sha256": policy_sha,
        "changed_paths_sha256": digest,
        "changed_path_count": len(changed),
        "changed_bytes": total,
        "subject_code_executed": False,
        "candidate_local_verifier_authoritative": False,
        "os_network_sandbox_claimed": False,
        **extra,
    }
    missing = sorted(set(gate["required_receipt_fields"]) - receipt.keys())
    require(not missing, f"receipt missing required fields: {missing}")
    return receipt

def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser()
    p.add_argument("--verifier-root", type=Path, required=True)
    p.add_argument("--subject-root", type=Path, required=True)
    p.add_argument("--manifest", type=Path, required=True)
    p.add_argument("--bundle-manifest", type=Path, required=True)
    p.add_argument("--current-pointer", type=Path, required=True)
    p.add_argument("--expected-verifier-head", required=True)
    p.add_argument("--subject-base", required=True)
    p.add_argument("--subject-head", required=True)
    p.add_argument("--receipt-out", type=Path, required=True)
    return p.parse_args()

def main() -> None:
    a = parse_args()
    receipt = verify(a.verifier_root,a.subject_root,a.manifest,a.bundle_manifest,a.current_pointer,
                     a.expected_verifier_head,a.subject_base,a.subject_head)
    a.receipt_out.write_text(canonical_json(receipt))
    print(f"QUAL-001 {receipt['mode']} gates: PASS")

if __name__ == "__main__":
    main()
