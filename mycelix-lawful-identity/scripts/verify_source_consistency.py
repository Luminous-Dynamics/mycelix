#!/usr/bin/env python3
"""Cheap fail-closed checks for protocol/source/documentation drift.

This is intentionally not a Rust compiler. It catches stale protocol names,
missing migration fields, broken documentation links, malformed scenario
contracts, and evidence-lane drift before expensive Nix/Cargo/conductor lanes.
Python optimization must not disable any check.
"""

from __future__ import annotations

import json
import re
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
INTEGRITY = ROOT / "zomes/cross-did-zkp/integrity/src/lib.rs"
COORDINATOR = ROOT / "zomes/cross-did-zkp/coordinator/src/lib.rs"


class ConsistencyError(RuntimeError):
    pass


def require(condition: bool, message: str) -> None:
    if not condition:
        raise ConsistencyError(message)


def balanced(text: str, left: str, right: str, name: str) -> None:
    require(text.count(left) == text.count(right), f"unbalanced {left}{right} in {name}")


def local_markdown_links(path: Path) -> list[Path]:
    text = path.read_text(encoding="utf-8")
    links = []
    for target in re.findall(r"\[[^\]]+\]\(([^)]+)\)", text):
        if "://" in target or target.startswith("#"):
            continue
        links.append((path.parent / target.split("#", 1)[0]).resolve())
    return links


def main() -> int:
    try:
        integrity = INTEGRITY.read_text(encoding="utf-8")
        coordinator = COORDINATOR.read_text(encoding="utf-8")
        combined = integrity + coordinator

        required = [
            "CREDENTIAL_ANCHOR_RECEIPT_PROTOCOL_V3",
            "POLICY_DECISION_PROTOCOL_V5",
            "supersedes_receipt_action_hash",
            "supersedes_decision_action_hash",
            "CredentialAnchorRefreshFork",
            "PolicyRenewalRequired",
            "PolicyRenewalFork",
            "PolicyLineageRevoked",
            "decision_lineage_revocations",
            "MAX_CHALLENGE_PROOF_LINKS",
            "MAX_LINEAGE_LINKS",
        ]
        for token in required:
            require(token in combined, f"missing required protocol token: {token}")

        forbidden = [
            "CREDENTIAL_ANCHOR_RECEIPT_PROTOCOL_V2",
            "POLICY_DECISION_PROTOCOL_V4",
            "CredentialAnchorConflict",
            "PolicyRevocationConflict",
        ]
        for token in forbidden:
            require(token not in combined, f"stale protocol token remains: {token}")

        for path in [INTEGRITY, COORDINATOR]:
            text = path.read_text(encoding="utf-8")
            balanced(text, "(", ")", str(path))
            balanced(text, "{", "}", str(path))
            balanced(text, "[", "]", str(path))
            require("todo!" not in text, f"todo! remains in {path.relative_to(ROOT)}")
            require(
                "unimplemented!" not in text,
                f"unimplemented! remains in {path.relative_to(ROOT)}",
            )

        scenario_path = ROOT / "tests/conductor/settlement_scenarios_v2.json"
        scenarios = json.loads(scenario_path.read_text(encoding="utf-8"))
        require(
            scenarios.get("schema") == "mycelix-lawful-identity-conductor-scenarios-v2",
            "unexpected scenario contract schema",
        )
        scenario_values = scenarios.get("scenarios")
        require(isinstance(scenario_values, list) and len(scenario_values) >= 12, "scenario contract is incomplete")

        required_files = [
            "tests/conductor/evidence_manifest.py",
            "tests/conductor/real_conductor_adapter.py",
            "tests/conductor/release_evidence_index.py",
            "tests/conductor/REAL_ADAPTER_PROTOCOL.md",
            "tests/conductor/REAL_DRIVER_PROTOCOL.md",
            "tests/conductor/test_evidence_manifest.py",
            "tests/conductor/test_real_conductor_adapter.py",
            "tests/conductor/test_release_evidence_index.py",
        ]
        for relative in required_files:
            require((ROOT / relative).is_file(), f"missing evidence-lane file: {relative}")

        runner = (ROOT / "tests/conductor/run_scenarios.py").read_text(encoding="utf-8")
        model_adapter = (ROOT / "tests/conductor/model_adapter.py").read_text(encoding="utf-8")
        real_adapter = (ROOT / "tests/conductor/real_conductor_adapter.py").read_text(encoding="utf-8")
        evidence_docs = (ROOT / "tests/conductor/REAL_ADAPTER_PROTOCOL.md").read_text(encoding="utf-8")
        for source_name, source in [
            ("scenario runner", runner),
            ("model adapter", model_adapter),
            ("real adapter", real_adapter),
            ("adapter protocol", evidence_docs),
        ]:
            require(
                "mycelix-conductor-scenario-adapter-v2" in source,
                f"{source_name} is not pinned to adapter protocol v2",
            )
            require(
                "mycelix-conductor-scenario-adapter-v1" not in source,
                f"{source_name} still references adapter protocol v1",
            )

        optimization_sensitive = [
            ROOT / "tests/conductor/run_scenarios.py",
            ROOT / "tests/conductor/model_adapter.py",
            ROOT / "tests/protocol_model/settlement_conflict_model.py",
            ROOT / "tests/conductor/validate_scenarios.py",
        ]
        for path in optimization_sensitive:
            for line_number, line in enumerate(path.read_text(encoding="utf-8").splitlines(), 1):
                require(
                    not line.lstrip().startswith("assert "),
                    f"optimization-sensitive assert in {path.relative_to(ROOT)}:{line_number}",
                )

        markdown = [
            ROOT / "README.md",
            ROOT / "QUICKSTART.md",
            *sorted((ROOT / "docs").glob("*.md")),
            *sorted((ROOT / "tests/conductor").glob("*.md")),
        ]
        for path in markdown:
            for target in local_markdown_links(path):
                if not target.is_relative_to(ROOT):
                    continue
                require(
                    target.exists(),
                    f"broken local link in {path.relative_to(ROOT)}: {target}",
                )

        require(
            not (ROOT / "docs/AUTHORIZATION_LIFECYCLE_V1.md").exists(),
            "stale authorization lifecycle v1 document remains",
        )
        require(
            (ROOT / "docs/AUTHORIZATION_LIFECYCLE_V2.md").exists(),
            "authorization lifecycle v2 document is missing",
        )
        print("lawful-identity source consistency: PASS")
        return 0
    except (ConsistencyError, OSError, UnicodeDecodeError, json.JSONDecodeError) as error:
        print(f"lawful-identity source consistency: FAIL: {error}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
