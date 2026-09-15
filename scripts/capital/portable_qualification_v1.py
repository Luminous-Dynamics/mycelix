#!/usr/bin/env python3
from __future__ import annotations

import argparse
import hashlib
import json
import platform
import shutil
import subprocess
import sys
import tempfile
from contextlib import contextmanager
from pathlib import Path

SPEC_VERSION = "mycelix-capital-portable-qualification-spec-v1"
BUNDLE_VERSION = "mycelix-capital-portable-qualification-bundle-v1"
QUAL_VERSION = "mycelix-capital-portable-qualification-v1"
MEMBERS = {"qualification.json", "source-digests.json", "case.json", "receipt.json", "subject.txt", "NONCLAIMS.md"}
SPEC_KEYS = {"spec_version", "qualified_subject_sha", "qualification_profile", "source_files", "case_path", "receipt_path", "qualification_command", "receipt_command", "semantic_expectations", "hosted_run", "pass_scope", "nonclaims_path"}
SEM_KEYS = {"case_canonical_sha256", "profile_sha256", "event_history_sha256", "event_chain_tip_sha256"}
RUN_KEYS = {"repository", "workflow_path", "run_id", "run_attempt", "head_sha", "conclusion"}
QUAL_KEYS = {"qualification_version", "bundle_version", "qualified_subject_sha", "qualification_profile", "pass_scope", "semantic_commitments", "generated_receipt_sha256", "source_digests_sha256", "runtime", "hosted_run_discovery", "nonclaims_sha256"}


class PortableQualificationError(ValueError):
    pass


def fail(msg: str):
    raise PortableQualificationError(msg)


def sha(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def canon(value) -> bytes:
    return json.dumps(value, sort_keys=True, separators=(",", ":"), ensure_ascii=False, allow_nan=False).encode()


def read_json(path: Path):
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except Exception as exc:
        fail(f"invalid JSON {path}: {exc}")
    if not isinstance(value, dict):
        fail(f"expected JSON object: {path}")
    return value


def exact(obj, keys, ctx):
    if not isinstance(obj, dict) or set(obj) != keys:
        fail(f"{ctx}: key mismatch")
    return obj


def hexstr(value, n, ctx):
    if not isinstance(value, str) or len(value) != n or value.lower() != value:
        fail(f"{ctx}: invalid hex")
    try:
        int(value, 16)
    except ValueError:
        fail(f"{ctx}: invalid hex")
    return value


def rel(value, ctx):
    if not isinstance(value, str) or not value or len(value) > 512:
        fail(f"{ctx}: invalid path")
    p = Path(value)
    if p.is_absolute() or ".." in p.parts or value.startswith("./"):
        fail(f"{ctx}: non-normalized path")
    return value


def load_spec(path: Path):
    s = exact(read_json(path), SPEC_KEYS, "spec")
    if s["spec_version"] != SPEC_VERSION:
        fail("unsupported spec version")
    hexstr(s["qualified_subject_sha"], 40, "qualified_subject_sha")
    files = s["source_files"]
    if not isinstance(files, list) or not files or len(files) > 64:
        fail("source_files: invalid")
    files = [rel(v, "source_files") for v in files]
    if len(set(files)) != len(files):
        fail("source_files: duplicate")
    s["source_files"] = files
    for key in ("case_path", "receipt_path", "nonclaims_path"):
        s[key] = rel(s[key], key)
    if s["case_path"] not in files or s["receipt_path"] not in files:
        fail("case/receipt must be frozen source files")
    for key in ("qualification_command", "receipt_command"):
        v = s[key]
        if not isinstance(v, list) or not v or not all(isinstance(x, str) and x and len(x) <= 512 for x in v):
            fail(f"{key}: invalid argv")
    sem = exact(s["semantic_expectations"], SEM_KEYS, "semantic_expectations")
    for key in SEM_KEYS:
        hexstr(sem[key], 64, key)
    run = exact(s["hosted_run"], RUN_KEYS, "hosted_run")
    hexstr(run["head_sha"], 40, "hosted_run.head_sha")
    if run["head_sha"] != s["qualified_subject_sha"] or run["conclusion"] != "success":
        fail("hosted_run: not bound to successful qualified subject")
    if not isinstance(run["run_id"], int) or not isinstance(run["run_attempt"], int) or run["run_id"] < 1 or run["run_attempt"] < 1:
        fail("hosted_run: invalid identifiers")
    if not isinstance(s["qualification_profile"], str) or not s["qualification_profile"] or not isinstance(s["pass_scope"], str) or not s["pass_scope"]:
        fail("qualification profile/scope missing")
    return s


def git(repo: Path, *args: str) -> bytes:
    p = subprocess.run(["git", "-C", str(repo), *args], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    if p.returncode:
        fail(f"git {' '.join(args)} failed: {p.stderr.decode(errors='replace').strip()}")
    return p.stdout


def subject_bytes(repo: Path, subject: str, path: str) -> bytes:
    return git(repo, "show", f"{subject}:{path}")


def assert_subject(repo: Path, s):
    subject = s["qualified_subject_sha"]
    if git(repo, "rev-parse", f"{subject}^{{commit}}").decode().strip() != subject:
        fail("subject does not resolve exactly")
    for path in s["source_files"]:
        current = repo / path
        if not current.is_file() or current.read_bytes() != subject_bytes(repo, subject, path):
            fail(f"qualified source drift: {path}")


@contextmanager
def subject_worktree(repo: Path, subject: str):
    with tempfile.TemporaryDirectory(prefix="myc-cap-subject-") as td:
        target = Path(td) / "subject"
        p = subprocess.run(["git", "-C", str(repo), "worktree", "add", "--detach", str(target), subject], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        if p.returncode:
            fail(f"worktree add failed: {p.stderr.decode(errors='replace').strip()}")
        try:
            if git(target, "rev-parse", "HEAD").decode().strip() != subject:
                fail("exact-subject worktree mismatch")
            yield target
        finally:
            subprocess.run(["git", "-C", str(repo), "worktree", "remove", "--force", str(target)], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)


def semantics(case_bytes: bytes):
    try:
        case = json.loads(case_bytes.decode())
    except Exception as exc:
        fail(f"case invalid: {exc}")
    if not isinstance(case, dict) or set(case) != {"profile", "events"} or not isinstance(case["events"], list) or not case["events"]:
        fail("case shape invalid")
    return {
        "case_canonical_sha256": sha(canon(case)),
        "profile_sha256": sha(canon(case["profile"])),
        "event_history_sha256": sha(canon(case["events"])),
        "event_chain_tip_sha256": sha(canon(case["events"][-1])),
    }


def run(argv, cwd: Path, case: Path | None = None, receipt: Path | None = None):
    expanded = [x.replace("{case}", str(case or "")).replace("{receipt}", str(receipt or "")) for x in argv]
    p = subprocess.run(expanded, cwd=cwd)
    if p.returncode:
        fail(f"command failed ({p.returncode}): {expanded}")


def source_digests(repo: Path, s):
    subject = s["qualified_subject_sha"]
    return {p: sha(subject_bytes(repo, subject, p)) for p in sorted(s["source_files"])}


def write_json(path: Path, value):
    path.write_text(json.dumps(value, sort_keys=True, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")


def qualification_payload(s, sem, receipt_bytes, digest_bytes, nonclaims_bytes):
    return {
        "qualification_version": QUAL_VERSION,
        "bundle_version": BUNDLE_VERSION,
        "qualified_subject_sha": s["qualified_subject_sha"],
        "qualification_profile": s["qualification_profile"],
        "pass_scope": s["pass_scope"],
        "semantic_commitments": sem,
        "generated_receipt_sha256": sha(receipt_bytes),
        "source_digests_sha256": sha(digest_bytes),
        "runtime": {"implementation": platform.python_implementation(), "python_version": platform.python_version(), "platform": platform.platform()},
        "hosted_run_discovery": s["hosted_run"],
        "nonclaims_sha256": sha(nonclaims_bytes),
    }


def build_bundle(spec_path: Path, repo: Path, out: Path):
    s = load_spec(spec_path)
    repo = repo.resolve()
    assert_subject(repo, s)
    subject = s["qualified_subject_sha"]
    case_bytes = subject_bytes(repo, subject, s["case_path"])
    checked_receipt = subject_bytes(repo, subject, s["receipt_path"])
    sem = semantics(case_bytes)
    if sem != s["semantic_expectations"]:
        fail("semantic commitments mismatch")
    with subject_worktree(repo, subject) as wrk:
        run(s["qualification_command"], wrk)
        with tempfile.TemporaryDirectory(prefix="myc-cap-receipt-") as td:
            case = Path(td) / "case.json"
            receipt = Path(td) / "receipt.json"
            case.write_bytes(case_bytes)
            run(s["receipt_command"], wrk, case, receipt)
            receipt_bytes = receipt.read_bytes()
    if receipt_bytes != checked_receipt:
        fail("regenerated receipt differs from qualified subject")
    digests = source_digests(repo, s)
    digest_bytes = (json.dumps(digests, sort_keys=True, indent=2) + "\n").encode()
    nonclaims = (repo / s["nonclaims_path"]).read_bytes()
    if b"not self-authenticating PASS" not in nonclaims:
        fail("nonclaims missing authority boundary")
    if out.exists():
        shutil.rmtree(out)
    out.mkdir(parents=True)
    (out / "case.json").write_bytes(case_bytes)
    (out / "receipt.json").write_bytes(receipt_bytes)
    (out / "source-digests.json").write_bytes(digest_bytes)
    (out / "subject.txt").write_text(subject + "\n")
    (out / "NONCLAIMS.md").write_bytes(nonclaims)
    write_json(out / "qualification.json", qualification_payload(s, sem, receipt_bytes, digest_bytes, nonclaims))


def verify_bundle(spec_path: Path, repo: Path, bundle: Path, run_tests=True):
    s = load_spec(spec_path)
    repo = repo.resolve()
    assert_subject(repo, s)
    if not bundle.is_dir() or {p.name for p in bundle.iterdir()} != MEMBERS or any(not p.is_file() for p in bundle.iterdir()):
        fail("bundle member mismatch")
    subject = s["qualified_subject_sha"]
    if (bundle / "subject.txt").read_text() != subject + "\n":
        fail("subject mismatch")
    case_bytes = (bundle / "case.json").read_bytes()
    if case_bytes != subject_bytes(repo, subject, s["case_path"]):
        fail("case mismatch")
    sem = semantics(case_bytes)
    if sem != s["semantic_expectations"]:
        fail("semantic commitments mismatch")
    expected_digests = source_digests(repo, s)
    actual_digests = read_json(bundle / "source-digests.json")
    if actual_digests != expected_digests:
        fail("source digest mismatch")
    digest_bytes = (bundle / "source-digests.json").read_bytes()
    receipt_bytes = (bundle / "receipt.json").read_bytes()
    if receipt_bytes != subject_bytes(repo, subject, s["receipt_path"]):
        fail("receipt mismatch")
    nonclaims = (bundle / "NONCLAIMS.md").read_bytes()
    if nonclaims != (repo / s["nonclaims_path"]).read_bytes():
        fail("nonclaims mismatch")
    q = exact(read_json(bundle / "qualification.json"), QUAL_KEYS, "qualification")
    expected_q = qualification_payload(s, sem, receipt_bytes, digest_bytes, nonclaims)
    for key in QUAL_KEYS - {"runtime"}:
        if q[key] != expected_q[key]:
            fail(f"qualification mismatch: {key}")
    if not isinstance(q["runtime"], dict) or set(q["runtime"]) != {"implementation", "python_version", "platform"}:
        fail("runtime provenance invalid")
    with subject_worktree(repo, subject) as wrk:
        if run_tests:
            run(s["qualification_command"], wrk)
        with tempfile.TemporaryDirectory(prefix="myc-cap-replay-") as td:
            regenerated = Path(td) / "receipt.json"
            run(s["receipt_command"], wrk, bundle / "case.json", regenerated)
            if regenerated.read_bytes() != receipt_bytes:
                fail("independent receipt replay mismatch")


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--spec", type=Path, required=True)
    p.add_argument("--repo", type=Path, default=Path("."))
    g = p.add_mutually_exclusive_group(required=True)
    g.add_argument("--build", type=Path)
    g.add_argument("--verify", type=Path)
    p.add_argument("--skip-tests", action="store_true")
    a = p.parse_args()
    try:
        if a.build:
            build_bundle(a.spec, a.repo, a.build)
        else:
            verify_bundle(a.spec, a.repo, a.verify, not a.skip_tests)
    except PortableQualificationError as exc:
        print(f"FAIL: {exc}", file=sys.stderr)
        return 2
    print("PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
