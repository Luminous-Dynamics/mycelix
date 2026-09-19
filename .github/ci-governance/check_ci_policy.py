#!/usr/bin/env python3
"""CI-GOV-001E deterministic GitHub Actions trust-boundary verifier.

Standard library only. This parser is intentionally narrow: it validates the
specific permission/action invariants in policy TOML without attempting to be a
general YAML implementation.
"""
from __future__ import annotations

import argparse
import hashlib
import re
import sys
import tomllib
from dataclasses import dataclass
from pathlib import Path

FULL_SHA = re.compile(r"^[0-9a-f]{40}$")
JOB_HEADER = re.compile(r"^  ([A-Za-z0-9_-]+):\s*$")
KEY_VALUE = re.compile(r"^([A-Za-z0-9_-]+):\s*(.*?)\s*$")
USES = re.compile(r"^\s*-\s+uses:\s*([^#\s]+)\s*(?:#.*)?$")
STEP_START = re.compile(r"^      -\s+")


class PolicyError(RuntimeError):
    pass


@dataclass(frozen=True)
class ParsedWorkflow:
    text: str
    lines: tuple[str, ...]
    jobs: dict[str, tuple[str, ...]]


def fail(message: str) -> None:
    raise PolicyError(message)


def indentation(line: str) -> int:
    return len(line) - len(line.lstrip(" "))


def parse_mapping_block(lines: tuple[str, ...], start: int, parent_indent: int) -> dict[str, str]:
    result: dict[str, str] = {}
    expected = parent_indent + 2
    for line in lines[start + 1:]:
        if not line.strip() or line.lstrip().startswith("#"):
            continue
        indent = indentation(line)
        if indent <= parent_indent:
            break
        if indent != expected:
            # Nested values are not part of a simple permission mapping.
            continue
        match = KEY_VALUE.match(line.strip())
        if not match:
            fail(f"malformed mapping line: {line!r}")
        key, value = match.groups()
        if key in result:
            fail(f"duplicate mapping key: {key}")
        result[key] = value.strip("'\"")
    return result


def parse_workflow(text: str) -> ParsedWorkflow:
    lines = tuple(text.splitlines())
    try:
        jobs_index = next(i for i, line in enumerate(lines) if line == "jobs:")
    except StopIteration:
        fail("missing top-level jobs block")

    jobs: dict[str, tuple[str, ...]] = {}
    starts: list[tuple[int, str]] = []
    for i in range(jobs_index + 1, len(lines)):
        m = JOB_HEADER.match(lines[i])
        if m:
            starts.append((i, m.group(1)))
    if not starts:
        fail("no jobs found")
    for n, (start, name) in enumerate(starts):
        if name in jobs:
            fail(f"duplicate job: {name}")
        end = starts[n + 1][0] if n + 1 < len(starts) else len(lines)
        jobs[name] = lines[start:end]
    return ParsedWorkflow(text=text, lines=lines, jobs=jobs)


def top_level_permissions(parsed: ParsedWorkflow) -> dict[str, str]:
    indexes = [
        i for i, line in enumerate(parsed.lines)
        if line == "permissions:"
    ]
    if len(indexes) != 1:
        fail(f"expected exactly one top-level permissions block, found {len(indexes)}")
    return parse_mapping_block(parsed.lines, indexes[0], 0)


def job_permissions(job_name: str, block: tuple[str, ...]) -> dict[str, str] | None:
    indexes = [i for i, line in enumerate(block) if line == "    permissions:"]
    if len(indexes) > 1:
        fail(f"{job_name}: duplicate permissions blocks")
    if not indexes:
        return None
    return parse_mapping_block(block, indexes[0], 4)


def action_refs(block: tuple[str, ...]) -> list[str]:
    refs: list[str] = []
    for line in block:
        m = USES.match(line)
        if m:
            refs.append(m.group(1))
    return refs


def is_immutable_action_ref(ref: str) -> bool:
    if ref.startswith("./"):
        return True
    if "@" not in ref:
        return False
    _, revision = ref.rsplit("@", 1)
    return bool(FULL_SHA.fullmatch(revision))


def checkout_steps(block: tuple[str, ...]) -> list[tuple[str, ...]]:
    starts = [i for i, line in enumerate(block) if STEP_START.match(line)]
    steps: list[tuple[str, ...]] = []
    for n, start in enumerate(starts):
        end = starts[n + 1] if n + 1 < len(starts) else len(block)
        step = block[start:end]
        if any("uses: actions/checkout@" in line for line in step):
            steps.append(step)
    return steps


def checkout_disables_persisted_credentials(step: tuple[str, ...]) -> bool:
    for line in step:
        if line.strip() == "persist-credentials: false":
            return True
    return False


def validate(workflow_text: str, policy: dict) -> list[str]:
    parsed = parse_workflow(workflow_text)

    expected_top = {
        key.replace("_", "-"): str(value)
        for key, value in policy["top_level_permissions"].items()
    }
    observed_top = top_level_permissions(parsed)
    if observed_top != expected_top:
        fail(f"top-level permissions mismatch: observed={observed_top!r}, expected={expected_top!r}")

    # No job may widen the token to write authority in v0.1.
    for job_name, block in parsed.jobs.items():
        perms = job_permissions(job_name, block)
        if perms:
            writes = sorted(key for key, value in perms.items() if value.lower() == "write")
            if writes:
                fail(f"{job_name}: forbidden write permissions: {', '.join(writes)}")

    critical = policy.get("critical_jobs", {})
    changes_policy = critical.get("changes")
    if changes_policy is None:
        fail("policy missing critical_jobs.changes")
    changes = parsed.jobs.get("changes")
    if changes is None:
        fail("workflow missing changes job")

    expected_changes = {
        "contents": str(changes_policy["contents"]),
        "pull-requests": str(changes_policy["pull_requests"]),
    }
    observed_changes = job_permissions("changes", changes)
    if observed_changes != expected_changes:
        fail(
            "changes: permissions mismatch: "
            f"observed={observed_changes!r}, expected={expected_changes!r}"
        )

    refs = action_refs(changes)
    if not refs:
        fail("changes: no actions found")
    mutable = [ref for ref in refs if not is_immutable_action_ref(ref)]
    if mutable:
        fail(f"changes: mutable action refs: {mutable!r}")

    expected_refs = list(changes_policy.get("immutable_actions", []))
    if refs != expected_refs:
        fail(f"changes: action sequence mismatch: observed={refs!r}, expected={expected_refs!r}")

    if bool(changes_policy.get("require_checkout_no_persist", False)):
        checkouts = checkout_steps(changes)
        if len(checkouts) != 1:
            fail(f"changes: expected exactly one checkout step, found {len(checkouts)}")
        if not checkout_disables_persisted_credentials(checkouts[0]):
            fail("changes: checkout must set persist-credentials: false")

    ci_pass_policy = critical.get("ci_pass", {})
    ci_pass = parsed.jobs.get("ci-pass")
    if ci_pass is None:
        fail("workflow missing ci-pass job")
    if bool(ci_pass_policy.get("forbid_external_actions", False)):
        refs = action_refs(ci_pass)
        if refs:
            fail(f"ci-pass: external actions forbidden, found {refs!r}")

    digest = hashlib.sha256(workflow_text.encode()).hexdigest()
    return [
        "CI_GOV_001E_POLICY_PASS",
        f"workflow_sha256={digest}",
        "top_level_token=contents:read",
        "changes_token=contents:read,pull-requests:read",
        "changes_actions=immutable",
        "changes_checkout_persist_credentials=false",
        "ci_pass_external_actions=none",
        "write_permissions=none",
    ]


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--policy",
        type=Path,
        default=Path(".github/ci-governance/ci-gov-001e-policy.toml"),
    )
    parser.add_argument("--workflow", type=Path)
    args = parser.parse_args(argv)

    policy = tomllib.loads(args.policy.read_text())
    workflow = args.workflow or Path(policy["workflow_path"])

    try:
        report = validate(workflow.read_text(), policy)
    except (OSError, KeyError, tomllib.TOMLDecodeError, PolicyError) as exc:
        print(f"CI_GOV_001E_POLICY_FAIL: {exc}", file=sys.stderr)
        return 1

    print("\n".join(report))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
