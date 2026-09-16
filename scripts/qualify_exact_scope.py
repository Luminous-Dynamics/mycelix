#!/usr/bin/env python3
import argparse
import json
import os
import re
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
DOC = ROOT / "docs/qualification/QUALIFICATION_HARNESS_V1.md"

EXPECTED_PROFILE = "mycelix-qualification-harness-v1"
EXPECTED_FROZEN_PARENT = "62d9fc51e506c2fbc0b8e08be348c02c31b5edeb"
EXPECTED_IDS = [f"QH-{i:03d}" for i in range(1, 11)]
EXPECTED_SELF_TEST_IDS = [
    "happy-path",
    "synthetic-merge-or-wrong-head",
    "wrong-parent",
    "two-commits",
    "unexpected-extra-path",
    "disallowed-delete",
    "mode-mismatch",
    "dirty-tree",
    "history-insufficient",
    "event-head-mismatch",
    "newline-path-safe",
]
EXPECTED_NONCLAIMS = [
    "domain semantic correctness",
    "code correctness",
    "cybersecurity or supply-chain security",
    "scientific validity",
    "legal validity or compliance",
    "policy legitimacy",
    "external-standard conformance",
    "production readiness",
]


class HarnessError(Exception):
    def __init__(self, code: str, message: str):
        super().__init__(message)
        self.code = code
        self.message = message


def normalize_prose(text: str) -> str:
    text = text.replace("`", "")
    return re.sub(r"\s+", " ", text.casefold()).strip()


def run_git(*args: str, check: bool = True) -> bytes:
    proc = subprocess.run(
        ["git", *args],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        check=False,
    )
    if check and proc.returncode != 0:
        raise HarnessError(
            "GitCommandFailed",
            f"git {' '.join(args)} failed with code {proc.returncode}",
        )
    return proc.stdout


def parse_raw_diff(raw: bytes):
    tokens = raw.split(b"\0")
    if tokens and tokens[-1] == b"":
        tokens.pop()
    if len(tokens) % 2 != 0:
        raise HarnessError("RawDiffMalformed", "expected raw diff header/path token pairs")

    entries = []
    for i in range(0, len(tokens), 2):
        header = tokens[i].decode("ascii", errors="strict")
        path = os.fsdecode(tokens[i + 1])
        fields = header.split()
        if len(fields) != 5 or not fields[0].startswith(":"):
            raise HarnessError("RawDiffMalformed", f"unexpected raw diff header: {header!r}")
        old_mode = fields[0][1:]
        new_mode = fields[1]
        status = fields[4]
        if len(status) != 1:
            raise HarnessError("UnsupportedChangeStatus", f"unexpected status token: {status!r}")
        entries.append({
            "path": path,
            "status": status,
            "old_mode": old_mode,
            "mode": new_mode,
        })
    return entries


def evaluate_snapshot(snapshot: dict, expected: dict) -> str:
    if snapshot.get("history_available") is not True:
        return "HistoryInsufficient"

    if snapshot.get("head") != expected.get("head"):
        return "HeadMismatch"

    if snapshot.get("event_name") == "pull_request":
        if snapshot.get("event_head") != expected.get("head"):
            return "EventHeadMismatch"

    if snapshot.get("parent") != expected.get("parent"):
        return "ParentMismatch"

    if snapshot.get("commit_count") != expected.get("commit_count"):
        return "CommitCountMismatch"

    actual_paths = sorted(item.get("path") for item in snapshot.get("changed", []))
    expected_paths = sorted(expected.get("paths", []))
    if actual_paths != expected_paths:
        return "PathSetMismatch"

    allowed = set(expected.get("allowed_statuses", []))
    for item in snapshot.get("changed", []):
        if item.get("status") not in allowed:
            return "DisallowedChangeStatus"

    expected_modes = expected.get("modes", {})
    by_path = {item.get("path"): item for item in snapshot.get("changed", [])}
    for path, mode in expected_modes.items():
        if by_path.get(path, {}).get("mode") != mode:
            return "ModeMismatch"

    if expected.get("require_clean") and snapshot.get("dirty"):
        return "WorkingTreeDirty"

    return "PASS"


def validate_profile_manifest(data: dict) -> None:
    if data.get("profile") != EXPECTED_PROFILE:
        raise HarnessError("SelfTestManifestInvalid", "unexpected profile id")
    if data.get("frozen_parent") != EXPECTED_FROZEN_PARENT:
        raise HarnessError("SelfTestManifestInvalid", "frozen parent changed")
    if data.get("governing_theorem") != "ScopeHarnessPass != domain semantic PASS":
        raise HarnessError("SelfTestManifestInvalid", "governing theorem changed")

    invariants = data.get("invariants")
    if not isinstance(invariants, list):
        raise HarnessError("SelfTestManifestInvalid", "invariants must be a list")
    ids = [item.get("id") for item in invariants]
    if ids != EXPECTED_IDS:
        raise HarnessError("SelfTestManifestInvalid", f"invariant census/order mismatch: {ids}")
    names = [item.get("name") for item in invariants]
    if len(names) != len(set(names)) or any(not isinstance(n, str) or not n for n in names):
        raise HarnessError("SelfTestManifestInvalid", "invariant names must be unique non-empty strings")

    tests = data.get("self_tests")
    if not isinstance(tests, list):
        raise HarnessError("SelfTestManifestInvalid", "self_tests must be a list")
    ids = [item.get("id") for item in tests]
    if ids != EXPECTED_SELF_TEST_IDS:
        raise HarnessError("SelfTestManifestInvalid", f"self-test census/order mismatch: {ids}")

    if data.get("mandatory_nonclaims") != EXPECTED_NONCLAIMS:
        raise HarnessError("SelfTestManifestInvalid", "mandatory nonclaim census/order changed")

    if not DOC.is_file():
        raise HarnessError("SelfTestManifestInvalid", "qualification harness document missing")
    doc = DOC.read_text(encoding="utf-8")
    doc_norm = normalize_prose(doc)
    headings = [
        line.split(" — ", 1)[0].strip("# ")
        for line in doc.splitlines()
        if line.startswith("## QH-")
    ]
    if headings != EXPECTED_IDS:
        raise HarnessError("SelfTestManifestInvalid", f"document invariant headings changed: {headings}")
    for claim in EXPECTED_NONCLAIMS:
        if normalize_prose(claim) not in doc_norm:
            raise HarnessError("SelfTestManifestInvalid", f"mandatory nonclaim absent: {claim}")


def run_self_tests(path: Path) -> dict:
    data = json.loads(path.read_text(encoding="utf-8"))
    validate_profile_manifest(data)

    observed = []
    for case in data["self_tests"]:
        actual = evaluate_snapshot(case["snapshot"], case["expected"])
        if actual != case["result"]:
            raise HarnessError(
                "SelfTestFailed",
                f"{case['id']} expected {case['result']} got {actual}",
            )
        observed.append({"id": case["id"], "result": actual})

    raw = (
        b":000000 100644 " + b"0" * 40 + b" " + b"1" * 40 + b" A\0"
        b"odd name\nsecond-line.txt\0"
    )
    parsed = parse_raw_diff(raw)
    if parsed != [{
        "path": "odd name\nsecond-line.txt",
        "status": "A",
        "old_mode": "000000",
        "mode": "100644",
    }]:
        raise HarnessError("SelfTestFailed", "raw NUL-delimited newline-path parser control failed")

    return {
        "profile": data["profile"],
        "status": "PASS",
        "self_tests": observed,
        "network_access_required": False,
        "domain_semantic_pass": False,
    }


def load_event(event_name: str | None, event_path: str | None):
    if not event_name:
        return None
    if not event_path:
        return {"event_name": event_name, "event_head": None}
    path = Path(event_path)
    if not path.is_file():
        raise HarnessError("EventDataUnavailable", f"event path not found: {event_path}")
    data = json.loads(path.read_text(encoding="utf-8"))
    event_head = None
    if event_name == "pull_request":
        try:
            event_head = data["pull_request"]["head"]["sha"]
        except (KeyError, TypeError):
            raise HarnessError("EventDataMalformed", "pull_request.head.sha missing")
    return {"event_name": event_name, "event_head": event_head}


def live_snapshot(args) -> dict:
    expected_parent = args.expected_parent

    if subprocess.run(
        ["git", "cat-file", "-e", f"{expected_parent}^{{commit}}"],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
        check=False,
    ).returncode != 0:
        return {
            "history_available": False,
            "head": None,
            "parent": None,
            "commit_count": None,
            "changed": [],
            "dirty": None,
            "event_name": args.event_name,
            "event_head": None,
        }

    head = run_git("rev-parse", "HEAD").decode().strip()
    try:
        parent = run_git("rev-parse", "HEAD^").decode().strip()
    except HarnessError:
        return {
            "history_available": False,
            "head": head,
            "parent": None,
            "commit_count": None,
            "changed": [],
            "dirty": None,
            "event_name": args.event_name,
            "event_head": None,
        }

    commit_count = int(run_git("rev-list", "--count", f"{expected_parent}..HEAD").decode().strip())
    raw = run_git("diff", "--raw", "-z", "--no-abbrev", "--no-renames", expected_parent, "HEAD", "--")
    changed = parse_raw_diff(raw)
    dirty = bool(run_git("status", "--porcelain=v1", "-z"))

    event = load_event(args.event_name, args.event_path) or {
        "event_name": None,
        "event_head": None,
    }

    return {
        "history_available": True,
        "head": head,
        "parent": parent,
        "commit_count": commit_count,
        "changed": changed,
        "dirty": dirty,
        **event,
    }


def parse_modes(items):
    result = {}
    for item in items or []:
        if "=" not in item:
            raise HarnessError("ArgumentInvalid", f"expected PATH=MODE, got {item!r}")
        path, mode = item.rsplit("=", 1)
        if not path or not re.fullmatch(r"[0-7]{6}", mode):
            raise HarnessError("ArgumentInvalid", f"invalid PATH=MODE: {item!r}")
        result[path] = mode
    return result


def emit(payload: dict, output: str | None = None, *, stream=sys.stdout):
    text = json.dumps(payload, sort_keys=True, ensure_ascii=False)
    print(text, file=stream)
    if output:
        Path(output).write_text(text + "\n", encoding="utf-8")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--self-test", type=Path)
    parser.add_argument("--expected-parent")
    parser.add_argument("--expected-head")
    parser.add_argument("--expected-commit-count", type=int, default=1)
    parser.add_argument("--expected-path", action="append", default=[])
    parser.add_argument("--allowed-status", action="append", default=[])
    parser.add_argument("--expected-mode", action="append", default=[])
    parser.add_argument("--event-name")
    parser.add_argument("--event-path")
    parser.add_argument("--require-clean-tree", action="store_true")
    parser.add_argument("--output")
    args = parser.parse_args()

    try:
        if args.self_test:
            result = run_self_tests(args.self_test)
            emit(result, args.output)
            return 0

        if not args.expected_parent or not args.expected_head:
            raise HarnessError("ArgumentInvalid", "--expected-parent and --expected-head are required")

        expected = {
            "parent": args.expected_parent,
            "head": args.expected_head,
            "commit_count": args.expected_commit_count,
            "paths": args.expected_path,
            "allowed_statuses": args.allowed_status or ["A", "M"],
            "modes": parse_modes(args.expected_mode),
            "require_clean": args.require_clean_tree,
        }

        snapshot = live_snapshot(args)
        result = evaluate_snapshot(snapshot, expected)
        if result != "PASS":
            raise HarnessError(result, f"scope verification failed with {result}")

        facts = [
            "HeadVerified",
            "ParentVerified",
            "CommitCountVerified",
            "PathSetVerified",
            "ModeSetVerified" if expected["modes"] else "ModeSetNotRequired",
            "WorkingTreeImmutable" if args.require_clean_tree else "WorkingTreeCheckNotRequired",
            "ScopeHarnessPass",
        ]
        payload = {
            "profile": EXPECTED_PROFILE,
            "status": "PASS",
            "facts": facts,
            "head": snapshot["head"],
            "parent": snapshot["parent"],
            "commit_count": snapshot["commit_count"],
            "changed": snapshot["changed"],
            "event_name": snapshot["event_name"],
            "event_head": snapshot["event_head"],
            "domain_semantic_pass": False,
            "network_access_required": False,
        }
        emit(payload, args.output)
        return 0
    except HarnessError as exc:
        payload = {
            "profile": EXPECTED_PROFILE,
            "status": "FAIL",
            "reason_code": exc.code,
            "message": exc.message,
            "domain_semantic_pass": False,
        }
        emit(payload, args.output, stream=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
