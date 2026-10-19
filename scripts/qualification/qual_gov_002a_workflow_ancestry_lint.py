#!/usr/bin/env python3
"""QUAL-GOV-002A: lint GitHub Actions ancestry requirements.

This is intentionally a small, dependency-free textual lint. It does not parse the
entire GitHub Actions YAML language and does not claim workflow correctness.
"""

from __future__ import annotations

import argparse
import pathlib
import re
import subprocess
import sys
from dataclasses import dataclass


IMMEDIATE_PARENT_PATTERNS = (
    re.compile(r"\bHEAD\^"),
    re.compile(r"\bHEAD~1\b"),
)
FULL_HISTORY_PATTERNS = (
    re.compile(r"\bgit\s+merge-base\b"),
)
FETCH_DEPTH_RE = re.compile(r"\bfetch-depth\s*:\s*['\"]?(\d+)['\"]?\s*(?:#.*)?$")
CHECKOUT_RE = re.compile(r"\buses\s*:\s*actions/checkout@")
STEP_START_RE = re.compile(r"^(\s*)-\s+(?:uses|name|run)\s*:")


@dataclass(frozen=True)
class Finding:
    path: str
    requirement: str
    checkout_depths: tuple[int | None, ...]
    reason: str


def ancestry_requirement(text: str) -> str:
    if any(p.search(text) for p in FULL_HISTORY_PATTERNS):
        return "full"
    if any(p.search(text) for p in IMMEDIATE_PARENT_PATTERNS):
        return "immediate"
    return "none"


def checkout_depths(text: str) -> tuple[int | None, ...]:
    """Return explicit fetch depth for each checkout step, or None if implicit.

    The scanner is conservative and only attributes fetch-depth values inside the
    checkout step's own indentation block.
    """

    lines = text.splitlines()
    depths: list[int | None] = []
    for i, line in enumerate(lines):
        if not CHECKOUT_RE.search(line):
            continue
        checkout_indent = len(line) - len(line.lstrip(" "))
        depth: int | None = None
        for candidate in lines[i + 1 :]:
            if candidate.strip() == "" or candidate.lstrip().startswith("#"):
                continue
            indent = len(candidate) - len(candidate.lstrip(" "))
            step = STEP_START_RE.match(candidate)
            if step and indent <= checkout_indent:
                break
            match = FETCH_DEPTH_RE.search(candidate)
            if match:
                depth = int(match.group(1))
                break
            if indent < checkout_indent:
                break
        depths.append(depth)
    return tuple(depths)


def check_workflow_text(path: str, text: str) -> Finding | None:
    requirement = ancestry_requirement(text)
    if requirement == "none":
        return None

    depths = checkout_depths(text)
    if not depths:
        return Finding(path, requirement, depths, "ancestry-sensitive workflow has no actions/checkout step")

    if requirement == "full":
        if 0 not in depths:
            return Finding(path, requirement, depths, "history-search operation requires explicit fetch-depth: 0")
        return None

    # immediate parent: full history or any explicit depth >= 2 is sufficient.
    if not any(depth == 0 or (depth is not None and depth >= 2) for depth in depths):
        return Finding(path, requirement, depths, "HEAD^/HEAD~1 requires explicit fetch-depth >= 2 or 0")
    return None


def changed_workflows(base: str, head: str) -> list[pathlib.Path]:
    proc = subprocess.run(
        ["git", "diff", "--name-only", base, head, "--", ".github/workflows"],
        check=True,
        text=True,
        stdout=subprocess.PIPE,
    )
    paths: list[pathlib.Path] = []
    for raw in proc.stdout.splitlines():
        p = pathlib.Path(raw)
        if p.suffix.lower() not in {".yml", ".yaml"}:
            continue
        if p.is_file():
            paths.append(p)
    return sorted(paths)


def run_self_tests() -> None:
    checkout = "      - uses: actions/checkout@11d5960a326750d5838078e36cf38b85af677262\n"

    cases = [
        (
            "head-caret-implicit-depth-refused",
            checkout + "      - run: git rev-parse HEAD^\n",
            True,
        ),
        (
            "head-caret-depth-one-refused",
            checkout + "        with:\n          fetch-depth: 1\n      - run: git rev-parse HEAD^\n",
            True,
        ),
        (
            "head-caret-depth-two-admitted",
            checkout + "        with:\n          fetch-depth: 2\n      - run: git rev-parse HEAD^\n",
            False,
        ),
        (
            "head-caret-full-history-admitted",
            checkout + "        with:\n          fetch-depth: 0\n      - run: git rev-parse HEAD^\n",
            False,
        ),
        (
            "head-tilde-depth-two-admitted",
            checkout + "        with:\n          fetch-depth: '2'\n      - run: git rev-parse HEAD~1\n",
            False,
        ),
        (
            "merge-base-depth-two-refused",
            checkout + "        with:\n          fetch-depth: 2\n      - run: git merge-base origin/main HEAD\n",
            True,
        ),
        (
            "merge-base-full-history-admitted",
            checkout + "        with:\n          fetch-depth: 0\n      - run: git merge-base origin/main HEAD\n",
            False,
        ),
        (
            "non-ancestry-workflow-admitted",
            checkout + "      - run: python3 -m compileall scripts\n",
            False,
        ),
    ]

    for name, text, should_refuse in cases:
        finding = check_workflow_text(name, text)
        refused = finding is not None
        if refused != should_refuse:
            raise AssertionError(f"{name}: expected refuse={should_refuse}, got {finding}")

    print(f"QUAL-GOV-002A SELF-TEST PASS: {len(cases)} closed cases")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--base")
    parser.add_argument("--head", default="HEAD")
    parser.add_argument("--self-test", action="store_true")
    args = parser.parse_args()

    if args.self_test:
        run_self_tests()

    if args.base:
        findings: list[Finding] = []
        workflows = changed_workflows(args.base, args.head)
        for path in workflows:
            text = path.read_text(encoding="utf-8")
            finding = check_workflow_text(str(path), text)
            if finding:
                findings.append(finding)

        if findings:
            for finding in findings:
                print(
                    f"QUAL-GOV-002A REFUSE: {finding.path}: {finding.reason}; "
                    f"requirement={finding.requirement}; checkout_depths={finding.checkout_depths}",
                    file=sys.stderr,
                )
            return 1
        print(f"QUAL-GOV-002A PASS: checked {len(workflows)} changed workflow(s)")

    if not args.self_test and not args.base:
        parser.error("provide --self-test and/or --base")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
