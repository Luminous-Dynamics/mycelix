#!/usr/bin/env python3
from __future__ import annotations

import argparse
import hashlib
import importlib.util
import json
from pathlib import Path

CORE_REL = "qualification/qual-001/verifier_v0_2.py"
CORE_SHA256 = "a93af3422785695b601bd873f316d1119a34cd41a4c8c5bb18b93adde7ca9c5f"
NEXT_BUNDLE_PROFILE = "mycelix.qual.static-subject-independence.v0.3"

GATE_KEYS = {
    "schema",
    "profile",
    "receipt_schema",
    "subject_execution",
    "same_repository_subject_required",
    "max_changed_paths",
    "max_changed_bytes",
    "ordinary_forbidden_exact_paths",
    "ordinary_forbidden_prefixes",
    "required_receipt_fields",
    "required_rotation_receipt_fields",
}
POLICY_KEYS = {
    "schema",
    "profile",
    "repository_protection_required",
    "admin_bypass_qualifies",
    "subject_code_execution",
    "rotation_requires_predecessor_bundle",
    "max_changed_paths",
    "max_changed_bytes",
    "allowed_exact_paths",
    "allowed_prefixes",
    "current_pointer_path",
    "immutable_launcher_paths",
    "required_launcher_components",
    "required_successor_components",
}


def require(condition: bool, message: str) -> None:
    if not condition:
        raise RuntimeError(message)


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def string_list(data: dict, key: str) -> list[str]:
    value = data.get(key)
    require(
        isinstance(value, list)
        and all(isinstance(item, str) and item for item in value)
        and len(value) == len(set(value)),
        f"invalid string list: {key}",
    )
    return value


def load_core(verifier_root: Path):
    root = verifier_root.resolve()
    core_path = verifier_root / CORE_REL
    require(
        core_path.exists()
        and core_path.is_file()
        and not core_path.is_symlink()
        and root in core_path.resolve().parents,
        "invalid verifier core path",
    )
    require(sha256(core_path) == CORE_SHA256, "verifier core digest mismatch")
    spec = importlib.util.spec_from_file_location("qual001_v02_core", core_path)
    require(spec is not None and spec.loader is not None, "cannot load verifier core")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def load_strict_gate(core, path: Path) -> dict:
    data, _ = core.load_json(path)
    require(set(data) == GATE_KEYS, "unexpected gate manifest fields")
    require(data.get("schema") == core.MANIFEST_SCHEMA, "wrong gate schema")
    require(data.get("receipt_schema") == core.RECEIPT_SCHEMA, "wrong receipt schema")
    require(data.get("subject_execution") is False, "subject execution forbidden")
    require(
        data.get("same_repository_subject_required") is True,
        "same-repository subject required",
    )
    for key in ("max_changed_paths", "max_changed_bytes"):
        require(isinstance(data.get(key), int) and data[key] > 0, f"invalid {key}")
    for key in (
        "ordinary_forbidden_exact_paths",
        "ordinary_forbidden_prefixes",
        "required_receipt_fields",
        "required_rotation_receipt_fields",
    ):
        string_list(data, key)
    return data


def load_strict_policy(core, path: Path) -> dict:
    data, _ = core.load_json(path)
    require(set(data) == POLICY_KEYS, "unexpected rotation policy fields")
    require(
        data.get("schema") == core.ROTATION_POLICY_SCHEMA,
        "wrong rotation policy schema",
    )
    require(
        data.get("repository_protection_required") is True,
        "repository protection requirement weakened",
    )
    require(data.get("admin_bypass_qualifies") is False, "admin bypass may not qualify")
    require(data.get("subject_code_execution") is False, "candidate code execution forbidden")
    require(
        data.get("rotation_requires_predecessor_bundle") is True,
        "predecessor binding required",
    )
    for key in ("max_changed_paths", "max_changed_bytes"):
        require(isinstance(data.get(key), int) and data[key] > 0, f"invalid {key}")
    for key in (
        "allowed_exact_paths",
        "allowed_prefixes",
        "immutable_launcher_paths",
        "required_launcher_components",
        "required_successor_components",
    ):
        string_list(data, key)
    pointer = data.get("current_pointer_path")
    require(isinstance(pointer, str) and pointer, "invalid current pointer path")
    core.validate_rel(pointer)
    return data


def install_monotonic_guard(core) -> None:
    original = core.validate_successor

    def hardened(
        verifier_root,
        subject_root,
        subject_base,
        changed,
        current_bundle,
        current_bundle_sha,
        policy,
    ):
        result = original(
            verifier_root,
            subject_root,
            subject_base,
            changed,
            current_bundle,
            current_bundle_sha,
            policy,
        )
        require(
            result["proposed_profile"] == NEXT_BUNDLE_PROFILE,
            "successor bundle profile is not the registered next profile",
        )

        current_gate = load_strict_gate(
            core,
            verifier_root / current_bundle["components"]["gate_manifest"]["path"],
        )
        current_policy = load_strict_policy(
            core,
            verifier_root / current_bundle["components"]["rotation_policy"]["path"],
        )

        successor_path = subject_root / result["proposed_bundle_path"]
        successor, _ = core.load_bundle(subject_root, successor_path)
        next_gate = load_strict_gate(
            core,
            subject_root / successor["components"]["gate_manifest"]["path"],
        )
        next_policy = load_strict_policy(
            core,
            subject_root / successor["components"]["rotation_policy"]["path"],
        )

        require(
            next_gate["profile"] == current_gate["profile"],
            "successor changed gate profile under fixed contract",
        )
        require(
            next_gate["receipt_schema"] == current_gate["receipt_schema"],
            "successor changed receipt schema without a new transition theorem",
        )
        for key in ("max_changed_paths", "max_changed_bytes"):
            require(
                next_gate[key] == current_gate[key],
                f"successor changed gate {key} under fixed schema",
            )
        for key in (
            "ordinary_forbidden_exact_paths",
            "ordinary_forbidden_prefixes",
            "required_receipt_fields",
            "required_rotation_receipt_fields",
        ):
            require(
                set(next_gate[key]) == set(current_gate[key]),
                f"successor changed gate {key} census under fixed schema",
            )

        require(
            next_policy["profile"] == current_policy["profile"],
            "successor changed rotation policy profile under fixed contract",
        )
        for key in ("max_changed_paths", "max_changed_bytes"):
            require(
                next_policy[key] == current_policy[key],
                f"successor changed policy {key} under fixed schema",
            )
        require(
            next_policy["current_pointer_path"]
            == current_policy["current_pointer_path"],
            "successor changed current pointer path",
        )
        for key in (
            "allowed_exact_paths",
            "allowed_prefixes",
            "immutable_launcher_paths",
            "required_launcher_components",
            "required_successor_components",
        ):
            require(
                set(next_policy[key]) == set(current_policy[key]),
                f"successor changed policy {key} census under fixed schema",
            )

        result.pop("candidate_verifier_code_executed", None)
        result[
            "successor_candidate_verifier_code_executed_during_rotation_authorization"
        ] = False
        return result

    core.validate_successor = hardened


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
    core = load_core(verifier_root)
    install_monotonic_guard(core)
    receipt = core.verify(
        verifier_root,
        subject_root,
        manifest_path,
        bundle_path,
        current_pointer,
        expected_verifier_head,
        subject_base,
        subject_head,
    )
    if receipt["mode"] == "rotation_authorization":
        bundle, _ = core.load_bundle(verifier_root, bundle_path)
        gate = load_strict_gate(
            core,
            verifier_root / bundle["components"]["gate_manifest"]["path"],
        )
        missing = sorted(
            set(gate["required_rotation_receipt_fields"]) - receipt.keys()
        )
        require(not missing, f"rotation receipt missing required fields: {missing}")
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
    core = load_core(a.verifier_root)
    receipt = verify(
        a.verifier_root,
        a.subject_root,
        a.manifest,
        a.bundle_manifest,
        a.current_pointer,
        a.expected_verifier_head,
        a.subject_base,
        a.subject_head,
    )
    a.receipt_out.write_text(core.canonical_json(receipt))
    print(f"QUAL-001 {receipt['mode']} monotonic gates: PASS")


if __name__ == "__main__":
    main()
