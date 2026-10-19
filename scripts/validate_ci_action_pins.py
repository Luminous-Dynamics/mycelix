#!/usr/bin/env python3
"""Read-only GitHub Actions `uses:` inventory for CI-GOV-001F.

This scanner is deliberately zero-network and standard-library only. It
classifies executable workflow `uses:` references but does not resolve tags,
change files, or claim action safety.
"""

from __future__ import annotations

import argparse
import json
import re
from collections import Counter
from dataclasses import dataclass, asdict
from pathlib import Path
from typing import Iterable

POLICY_PATH = Path("docs/ci/ci_gov_001f_action_policy.json")
DOC_PATH = Path("docs/ci/CI_GOV_001F_ACTION_PINNING_POLICY_V1.md")
FULL_SHA_RE = re.compile(r"^[0-9a-fA-F]{40}$")
DOCKER_DIGEST_RE = re.compile(r"^docker://.+@sha256:[0-9a-fA-F]{64}$")
USES_KEY_RE = re.compile(
    r'''^\s*(?:-\s*)?(?:"uses"|'uses'|uses)\s*:\s*(?P<value>.+?)\s*$'''
)
FLOW_MAP_RE = re.compile(r"^\s*-\s*\{(?P<body>.*)\}\s*$")
FLOW_USES_RE = re.compile(
    r'''(?:^|,)\s*(?:"uses"|'uses'|uses)\s*:\s*(?P<value>[^,}]+)'''
)
BLOCK_SCALAR_RE = re.compile(
    r"^(?P<indent>\s*)(?:-\s*)?[^#\n][^:\n]*:\s*[|>][0-9+-]*\s*(?:#.*)?$"
)

CLASSIFICATIONS = {
    "LocalAction",
    "LocalReusableWorkflow",
    "ImmutableExternalAction",
    "MutableExternalAction",
    "ImmutableReusableWorkflow",
    "MutableReusableWorkflow",
    "ImmutableDockerImage",
    "MutableDockerImage",
    "InvalidUsesRef",
}

MUTABLE_OR_INVALID = {
    "MutableExternalAction",
    "MutableReusableWorkflow",
    "MutableDockerImage",
    "InvalidUsesRef",
}


@dataclass(frozen=True)
class Finding:
    path: str
    line: int
    value: str
    classification: str
    locator: str | None
    ref: str | None


def _strip_inline_comment(text: str) -> str:
    """Strip a YAML inline comment when `#` occurs outside quotes."""
    single = False
    double = False
    escaped = False
    for i, ch in enumerate(text):
        if double:
            if escaped:
                escaped = False
            elif ch == "\\":
                escaped = True
            elif ch == '"':
                double = False
            continue
        if single:
            if ch == "'":
                single = False
            continue
        if ch == '"':
            double = True
        elif ch == "'":
            single = True
        elif ch == "#" and (i == 0 or text[i - 1].isspace()):
            return text[:i].rstrip()
    return text.rstrip()


def _unquote(value: str) -> str:
    value = value.strip()
    if len(value) >= 2 and value[0] == value[-1] and value[0] in {"'", '"'}:
        return value[1:-1]
    return value


def _indent(raw: str) -> int:
    return len(raw) - len(raw.lstrip(" "))


def classify_uses(value: str) -> tuple[str, str | None, str | None]:
    value = value.strip()
    if not value:
        return "InvalidUsesRef", None, None

    if value.startswith("./"):
        kind = (
            "LocalReusableWorkflow"
            if value.startswith("./.github/workflows/")
            else "LocalAction"
        )
        return kind, value, None

    if value.startswith("docker://"):
        if DOCKER_DIGEST_RE.fullmatch(value):
            locator, ref = value.rsplit("@", 1)
            return "ImmutableDockerImage", locator, ref
        return "MutableDockerImage", value, None

    if "@" not in value:
        return "InvalidUsesRef", value, None

    locator, ref = value.rsplit("@", 1)
    if not locator or not ref:
        return "InvalidUsesRef", locator or None, ref or None

    reusable = "/.github/workflows/" in locator
    immutable = bool(FULL_SHA_RE.fullmatch(ref))
    if reusable:
        kind = "ImmutableReusableWorkflow" if immutable else "MutableReusableWorkflow"
    else:
        kind = "ImmutableExternalAction" if immutable else "MutableExternalAction"
    return kind, locator, ref


def _finding(path: str, lineno: int, value: str) -> Finding:
    value = _unquote(_strip_inline_comment(value))
    classification, locator, ref = classify_uses(value)
    return Finding(
        path=path,
        line=lineno,
        value=value,
        classification=classification,
        locator=locator,
        ref=ref,
    )


def extract_uses(text: str, path: str) -> list[Finding]:
    findings: list[Finding] = []
    block_indent: int | None = None

    for lineno, raw in enumerate(text.splitlines(), start=1):
        stripped = raw.strip()
        indent = _indent(raw)

        if block_indent is not None:
            if not stripped:
                continue
            if indent > block_indent:
                continue
            block_indent = None

        if raw.lstrip().startswith("#"):
            continue

        block = BLOCK_SCALAR_RE.match(raw)
        if block:
            block_indent = len(block.group("indent"))
            continue

        match = USES_KEY_RE.match(raw)
        if match:
            findings.append(_finding(path, lineno, match.group("value")))
            continue

        flow = FLOW_MAP_RE.match(raw)
        if flow:
            flow_uses = FLOW_USES_RE.search(flow.group("body"))
            if flow_uses:
                findings.append(_finding(path, lineno, flow_uses.group("value")))

    return findings


def load_policy() -> dict:
    policy = json.loads(POLICY_PATH.read_text(encoding="utf-8"))
    if policy.get("tranche") != "CI-GOV-001F" or policy.get("issue") != 1414:
        raise SystemExit("policy tranche/issue drift")
    if set(policy.get("classifications", [])) != CLASSIFICATIONS:
        raise SystemExit("policy classification set drift")
    roots = policy.get("scan_roots")
    if not isinstance(roots, list) or not roots or not all(isinstance(x, str) and x for x in roots):
        raise SystemExit("policy scan_roots must be a non-empty string list")
    suffixes = policy.get("workflow_suffixes")
    if suffixes != [".yml", ".yaml"]:
        raise SystemExit("workflow suffix profile drift")
    return policy


def workflow_files(policy: dict) -> list[Path]:
    suffixes = set(policy["workflow_suffixes"])
    files: set[Path] = set()
    for root_text in policy["scan_roots"]:
        root = Path(root_text)
        if not root.exists():
            continue
        for path in root.rglob("*"):
            if path.is_file() and path.suffix in suffixes:
                files.add(path)
    return sorted(files)


def inventory(policy: dict) -> list[Finding]:
    findings: list[Finding] = []
    for path in workflow_files(policy):
        findings.extend(extract_uses(path.read_text(encoding="utf-8"), path.as_posix()))
    return findings


def validate_document(policy: dict) -> None:
    doc = DOC_PATH.read_text(encoding="utf-8").casefold()
    required = policy.get("required_nonclaims")
    if not isinstance(required, list) or not required or len(required) != len(set(required)):
        raise SystemExit("required_nonclaims must be a non-empty unique list")
    for phrase in required:
        if not isinstance(phrase, str) or not phrase.strip():
            raise SystemExit("required_nonclaims entries must be non-empty strings")
        if phrase.casefold() not in doc:
            raise SystemExit(f"manifest-declared nonclaim missing from policy document: {phrase}")


def summarize(findings: Iterable[Finding]) -> dict:
    findings = list(findings)
    counts = Counter(f.classification for f in findings)
    return {
        "reference_count": len(findings),
        "classification_counts": {k: counts.get(k, 0) for k in sorted(CLASSIFICATIONS)},
        "mutable_or_invalid_count": sum(counts.get(k, 0) for k in MUTABLE_OR_INVALID),
        "findings": [asdict(f) for f in findings],
    }


def self_test() -> None:
    sha = "a" * 40
    digest = "b" * 64
    cases = {
        "actions/checkout@v4": "MutableExternalAction",
        f"actions/checkout@{sha}": "ImmutableExternalAction",
        "owner/repo/.github/workflows/reuse.yml@main": "MutableReusableWorkflow",
        f"owner/repo/.github/workflows/reuse.yml@{sha}": "ImmutableReusableWorkflow",
        "./.github/actions/local": "LocalAction",
        "./.github/workflows/local.yml": "LocalReusableWorkflow",
        "docker://alpine:3.20": "MutableDockerImage",
        f"docker://example.invalid/image@sha256:{digest}": "ImmutableDockerImage",
        "actions/checkout": "InvalidUsesRef",
        "actions/checkout@deadbeef": "MutableExternalAction",
    }
    for value, expected in cases.items():
        actual, _, _ = classify_uses(value)
        assert actual == expected, (value, expected, actual)

    sample = f"""
# - uses: ignored/comment@v1
jobs:
  a:
    steps:
      - uses: actions/checkout@v4 # mutable executable ref
      - name: nested
        \"uses\": owner/repo@{sha}
      - name: script text is not an action
        run: |
          echo hello
          uses: fake/action@v1
      - {{ name: flow, uses: owner/flow@{sha} }}
      - 'uses': 'owner/repo/.github/workflows/reuse.yml@main'
      - run: echo 'uses: not/a/key@v1'
"""
    found = extract_uses(sample, "fixture.yml")
    assert [f.classification for f in found] == [
        "MutableExternalAction",
        "ImmutableExternalAction",
        "ImmutableExternalAction",
        "MutableReusableWorkflow",
    ]
    assert [f.value for f in found] == [
        "actions/checkout@v4",
        f"owner/repo@{sha}",
        f"owner/flow@{sha}",
        "owner/repo/.github/workflows/reuse.yml@main",
    ]
    print("CI-GOV-001F classifier self-test PASS")


def main() -> int:
    parser = argparse.ArgumentParser()
    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument("--self-test", action="store_true")
    group.add_argument("--inventory", action="store_true")
    group.add_argument("--enforce", action="store_true")
    args = parser.parse_args()

    if args.self_test:
        self_test()
        return 0

    policy = load_policy()
    validate_document(policy)
    findings = inventory(policy)
    summary = summarize(findings)

    if args.inventory:
        output = {
            "profile_id": policy["profile_id"],
            "disposition": "InventoryOnly",
            "enforcement_enabled": policy["enforcement_enabled"],
            **summary,
        }
        print(json.dumps(output, sort_keys=True))
        return 0

    if policy.get("enforcement_enabled") is not True:
        print(
            json.dumps(
                {
                    "profile_id": policy["profile_id"],
                    "disposition": "EnforcementNotEnabled",
                    "enforcement_enabled": False,
                    **summary,
                },
                sort_keys=True,
            )
        )
        return 2

    bad = [f for f in findings if f.classification in MUTABLE_OR_INVALID]
    output = {
        "profile_id": policy["profile_id"],
        "disposition": "ActionReferencePolicyPass" if not bad else "ActionReferencePolicyFail",
        "enforcement_enabled": True,
        **summary,
    }
    print(json.dumps(output, sort_keys=True))
    return 0 if not bad else 1


if __name__ == "__main__":
    raise SystemExit(main())
