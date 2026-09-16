#!/usr/bin/env python3
from __future__ import annotations

import argparse
import hashlib
import json
import re
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Iterable

SCANNER_VERSION = "mycelix-workflow-parent-provenance-v1"
HEX40_RE = re.compile(r"\b[0-9a-f]{40}\b")
HEAD_TILDE_RE = re.compile(r"\bHEAD~(\d+)\b")
HEAD_CARET_CHAIN_RE = re.compile(r"\bHEAD(\^+)")
GIT_OP_RE = re.compile(r"\bgit\s+(rev-parse|merge-base|diff|cat-file|show|log)\b")
FETCH_RE = re.compile(r"\bgit\s+fetch\b")
CHECKOUT_RE = re.compile(r"actions/checkout@")

@dataclass
class HistoryState:
    checkout_seen: bool = False
    depth: int | None = None
    exact_objects: set[str] | None = None

    def __post_init__(self) -> None:
        if self.exact_objects is None:
            self.exact_objects = set()

    def reset_checkout(self, depth: int | None) -> None:
        self.checkout_seen = True
        self.depth = depth
        self.exact_objects = set()

    def deepen(self, amount: int) -> None:
        if self.depth == 0:
            return
        if self.depth is not None:
            self.depth += amount

    def unshallow(self) -> None:
        self.depth = 0

def canonical_bytes(value: Any) -> bytes:
    return (json.dumps(value, sort_keys=True, separators=(",", ":"), ensure_ascii=False) + "\n").encode()

def sha256_hex(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()

def _indent(line: str) -> int:
    return len(line) - len(line.lstrip(" "))

def _strip_inline_comment(value: str) -> str:
    if " #" in value:
        value = value.split(" #", 1)[0]
    return value.strip().strip("'\"")

def _parse_int_scalar(value: str) -> int | None:
    value = _strip_inline_comment(value)
    if re.fullmatch(r"\d+", value):
        return int(value)
    return None

def _step_blocks(text: str) -> list[dict[str, Any]]:
    lines = text.splitlines()
    jobs_indent = None
    for idx, line in enumerate(lines):
        if line.strip() == "jobs:":
            jobs_indent = _indent(line)
            jobs_start = idx + 1
            break
    else:
        return []

    job_starts: list[tuple[int, str, int]] = []
    for i in range(jobs_start, len(lines)):
        line = lines[i]
        s = line.strip()
        if not s or s.startswith("#"):
            continue
        ind = _indent(line)
        if ind <= jobs_indent:
            break
        m = re.match(r"^([A-Za-z0-9_.-]+):\s*(?:#.*)?$", s)
        if m and ind == jobs_indent + 2:
            job_starts.append((i, m.group(1), ind))

    out: list[dict[str, Any]] = []
    for n, (jstart, job_id, job_indent) in enumerate(job_starts):
        jend = job_starts[n+1][0] if n+1 < len(job_starts) else len(lines)
        steps_line = None
        steps_indent = None
        for i in range(jstart + 1, jend):
            if lines[i].strip() == "steps:" and _indent(lines[i]) == job_indent + 2:
                steps_line = i
                steps_indent = _indent(lines[i])
                break
        if steps_line is None:
            continue
        step_starts: list[int] = []
        step_indent = steps_indent + 2
        for i in range(steps_line + 1, jend):
            line = lines[i]
            if _indent(line) == step_indent and line.lstrip().startswith("- "):
                step_starts.append(i)
        for si, start in enumerate(step_starts):
            end = step_starts[si+1] if si+1 < len(step_starts) else jend
            block = lines[start:end]
            out.append({
                "job_id": job_id,
                "step_index": si + 1,
                "start_line": start + 1,
                "lines": block,
                "step_indent": step_indent,
            })
    return out

def _parse_step(block: dict[str, Any]) -> dict[str, Any]:
    lines = block["lines"]
    step_indent = block["step_indent"]
    name = f"step-{block['step_index']}"
    uses = None
    fetch_depth: int | None = None
    fetch_depth_present = False
    run_lines: list[tuple[int, str]] = []

    def content_after_dash(line: str) -> str:
        s = line.strip()
        return s[2:] if s.startswith("- ") else s

    for off, line in enumerate(lines):
        content = content_after_dash(line) if off == 0 else line.strip()
        if content.startswith("name:"):
            name = _strip_inline_comment(content.split(":", 1)[1])
        if content.startswith("uses:"):
            uses = _strip_inline_comment(content.split(":", 1)[1])
        if content.startswith("fetch-depth:"):
            fetch_depth_present = True
            fetch_depth = _parse_int_scalar(content.split(":", 1)[1])

    for off, line in enumerate(lines):
        lineno = block["start_line"] + off
        content = content_after_dash(line) if off == 0 else line.strip()
        if content.startswith("run:"):
            rhs = content.split(":", 1)[1].strip()
            run_indent = _indent(line)
            if rhs in {"|", "|-", "|+", ">", ">-", ">+"}:
                for off2 in range(off + 1, len(lines)):
                    ln = lines[off2]
                    if ln.strip() and _indent(ln) <= run_indent:
                        break
                    run_lines.append((block["start_line"] + off2, ln.strip()))
            elif rhs:
                run_lines.append((lineno, rhs))
            break

    return {
        **{k: block[k] for k in ("job_id", "step_index", "start_line")},
        "name": name,
        "uses": uses,
        "fetch_depth": fetch_depth,
        "fetch_depth_present": fetch_depth_present,
        "run_lines": run_lines,
    }

def _required_checkout_depth(command: str) -> int | None:
    distances: list[int] = []
    for m in HEAD_TILDE_RE.finditer(command):
        distances.append(int(m.group(1)))
    for m in HEAD_CARET_CHAIN_RE.finditer(command):
        distances.append(len(m.group(1)))
    if distances:
        return max(distances) + 1
    return None

def _is_documentation_shell_line(s: str) -> bool:
    stripped = s.strip()
    if not stripped or stripped.startswith("#"):
        return True
    return bool(re.match(r"^(echo|printf)\b", stripped))

def _is_ancestry_sensitive(command: str) -> bool:
    if _is_documentation_shell_line(command):
        return False
    m = GIT_OP_RE.search(command)
    if not m:
        return False
    op = m.group(1)
    if _required_checkout_depth(command) is not None:
        return True
    if op == "merge-base":
        return True
    if op == "diff" and ("..." in command or ".." in command):
        return True
    if op in {"cat-file", "show", "log", "rev-parse"} and HEX40_RE.search(command):
        return True
    return False

def _classify_operation(command: str, state: HistoryState) -> tuple[bool, str, int | None]:
    req_depth = _required_checkout_depth(command)
    if req_depth is not None:
        if not state.checkout_seen:
            return False, "PARENT_OBJECT_AVAILABILITY_UNPROVEN", req_depth
        if state.depth is None:
            return False, "DYNAMIC_FETCH_DEPTH_UNPROVEN", req_depth
        if state.depth == 0 or state.depth >= req_depth:
            return True, "OK", req_depth
        return False, "ANCESTRY_DEPTH_INSUFFICIENT", req_depth

    exacts = set(HEX40_RE.findall(command))
    if exacts and exacts.issubset(state.exact_objects):
        return True, "OK", None
    if state.checkout_seen and state.depth == 0:
        return True, "OK", None
    if state.checkout_seen and state.depth is None:
        return False, "DYNAMIC_FETCH_DEPTH_UNPROVEN", None
    return False, "PARENT_OBJECT_AVAILABILITY_UNPROVEN", None

def _apply_fetch(command: str, state: HistoryState) -> None:
    if not FETCH_RE.search(command) or _is_documentation_shell_line(command):
        return
    if "--unshallow" in command:
        state.unshallow()
    m = re.search(r"--deepen(?:=|\s+)(\d+)", command)
    if m:
        state.deepen(int(m.group(1)))
    for oid in HEX40_RE.findall(command):
        state.exact_objects.add(oid)

def scan_text(text: str, workflow_path: str = "<memory>") -> dict[str, Any]:
    state_by_job: dict[str, HistoryState] = {}
    findings: list[dict[str, Any]] = []
    for raw in _step_blocks(text):
        step = _parse_step(raw)
        state = state_by_job.setdefault(step["job_id"], HistoryState())
        if step["uses"] and CHECKOUT_RE.search(step["uses"]):
            depth = step["fetch_depth"] if step["fetch_depth_present"] else 1
            state.reset_checkout(depth)
            continue
        for line_no, command in step["run_lines"]:
            if not command or command.startswith("#"):
                continue
            if FETCH_RE.search(command):
                _apply_fetch(command, state)
                continue
            if not _is_ancestry_sensitive(command):
                continue
            passed, code, req_depth = _classify_operation(command, state)
            findings.append({
                "workflow_path": workflow_path,
                "job_id": step["job_id"],
                "step_index": step["step_index"],
                "step_name": step["name"],
                "line": line_no,
                "operation": command,
                "required_checkout_depth": req_depth,
                "observed_checkout_depth": state.depth,
                "exact_objects_available": sorted(state.exact_objects),
                "finding_code": code,
                "passed": passed,
            })

    findings.sort(key=lambda x: (x["workflow_path"], x["job_id"], x["step_index"], x["line"], x["operation"]))
    failed = [f for f in findings if not f["passed"]]
    return {
        "report_version": SCANNER_VERSION,
        "workflow_path": workflow_path,
        "result": "PASS" if not failed else "FAIL",
        "findings": findings,
        "nonclaims": [
            "PASS does not prove that an asserted parent is the correct policy parent",
            "PASS does not prove semantic tests pass",
            "PASS does not authenticate GitHub metadata",
            "PASS covers only ancestry-sensitive commands recognized by this frozen scanner",
        ],
    }

def scan_paths(paths: Iterable[Path], root: Path) -> dict[str, Any]:
    reports = []
    for path in sorted(paths, key=lambda p: str(p)):
        rel = str(path.relative_to(root)) if path.is_relative_to(root) else str(path)
        reports.append(scan_text(path.read_text(encoding="utf-8"), rel))
    all_findings = [f for report in reports for f in report["findings"]]
    result = "PASS" if all(f["passed"] for f in all_findings) else "FAIL"
    body = {
        "report_version": SCANNER_VERSION,
        "result": result,
        "workflow_count": len(reports),
        "finding_count": len(all_findings),
        "failed_finding_count": sum(not f["passed"] for f in all_findings),
        "workflows": reports,
        "nonclaims": [
            "PASS is static parent-availability evidence only",
            "PASS is not semantic workflow qualification",
            "PASS is not source or runner authentication",
        ],
    }
    body["report_sha256"] = sha256_hex(canonical_bytes({k:v for k,v in body.items() if k != "report_sha256"}))
    return body

def discover(root: Path) -> list[Path]:
    wf = root / ".github" / "workflows"
    return sorted(list(wf.glob("*.yml")) + list(wf.glob("*.yaml"))) if wf.exists() else []

def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("paths", nargs="*", type=Path)
    ap.add_argument("--root", type=Path, default=Path("."))
    ap.add_argument("--report-out", type=Path)
    ap.add_argument("--report-only", action="store_true")
    args = ap.parse_args()
    root = args.root.resolve()
    paths = [p.resolve() for p in args.paths] if args.paths else discover(root)
    report = scan_paths(paths, root)
    data = canonical_bytes(report)
    if args.report_out:
        args.report_out.write_bytes(data)
    else:
        print(data.decode(), end="")
    if report["result"] == "FAIL" and not args.report_only:
        return 1
    return 0

if __name__ == "__main__":
    raise SystemExit(main())
