#!/usr/bin/env python3
"""Fast stdlib-only source preflight; deliberately not a Rust compiler substitute."""

from __future__ import annotations

import hashlib
import json
import os
import re
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[4]
CRATE = ROOT / "mycelix-workspace/simulations/civ-econ-transition-validator"
JOURNAL_PATH = CRATE / "src/durable_journal.rs"
SCALE_PATH = CRATE / "src/journal_replay_scale.rs"
MANIFEST_PATH = CRATE / "Cargo.toml"
LOCKFILE_PATH = CRATE / "Cargo.lock"
README_PATH = CRATE / "README.md"
DESIGN_PATH = CRATE / "JOURNAL_CHECKPOINT_COMPACTION_DESIGN.md"
WORKFLOW_PATH = ROOT / ".github/workflows/civ-econ-transition-validator.yml"
SCRIPT_PATH = Path(__file__).resolve()

failures: list[str] = []
passed: list[str] = []


def check(name: str, condition: bool) -> None:
    (passed if condition else failures).append(name)


def read(path: Path) -> str:
    if not path.is_file():
        failures.append(f"required file exists: {path.relative_to(ROOT)}")
        return ""
    return path.read_text(encoding="utf-8")


journal = read(JOURNAL_PATH)
scale = read(SCALE_PATH)
manifest = read(MANIFEST_PATH)
lockfile = read(LOCKFILE_PATH)
readme = read(README_PATH)
design = read(DESIGN_PATH)
workflow = read(WORKFLOW_PATH)
audit_script = read(SCRIPT_PATH)

# Replay resource bounds and parser structure.
check("record limit is 4096 bytes", "MAX_JOURNAL_RECORD_BYTES: usize = 4096;" in journal)
check("effect ID limit is 1024 bytes", "MAX_EFFECT_ID_BYTES: usize = 1024;" in journal)
check("replay uses the bounded record reader", "read_bounded_record(reader, &mut record, line_no + 1)?" in journal)
check("bounded reader uses fill_buf/consume", re.search(r"reader\s*\.fill_buf\(\)", journal) is not None and "reader.consume(consumed)" in journal)
check("no full-file read_to_end replay path", "read_to_end(" not in journal)
check("record fields are borrowed rather than collected per line", "let mut fields = line.split('\\t');" in journal and "let fields: Vec<&str> = line.split" not in journal)
match_start = journal.find("let (provider_profile_digest, receipt_digest, source_evidence_digest) =")
match_end = journal.find("if encoded_id.len() > MAX_EFFECT_ID_BYTES * 2", match_start)
check(
    "record-shape match closes exactly once",
    match_start >= 0 and match_end > match_start and journal[match_start:match_end].count("};") == 1,
)
check(
    "encoded effect ID limit is checked before decoding",
    journal.find("if encoded_id.len() > MAX_EFFECT_ID_BYTES * 2")
    < journal.find("let id_bytes = decode_hex(encoded_id, line_no)?"),
)
for test_name in (
    "streaming_replay_handles_history_larger_than_reader_buffer",
    "streaming_replay_rejects_truncated_final_record_after_valid_history",
    "replay_rejects_extra_record_fields_without_ignoring_them",
    "effect_count_includes_all_states_and_survives_replay",
):
    annotated_test = re.search(r"#\[test\]\s*fn\s+" + re.escape(test_name) + r"\(", journal)
    check(f"regression test is annotated: {test_name}", annotated_test is not None)

# Scale probe and build registration must describe the same four lifecycle paths.
scenarios = ("pending", "indeterminate", "acknowledged", "reconciled")
check("scale probe has the one-million-effect ceiling", "const MAX_EFFECTS: usize = 1_000_000;" in scale)
check("scale probe verifies total count and unresolved count", "journal.effect_count()" in scale and "journal.unresolved_effect_count()" in scale)
check("scale probe verifies first and last states", "first_status_matches" in scale and "last_status_matches" in scale)
check("scale probe writes and syncs its generated journal before replay", "writer.flush()?;" in scale and "writer.get_ref().sync_all()?;" in scale)
for scenario in scenarios:
    check(f"scale probe scenario is implemented: {scenario}", f'"{scenario}"' in scale)
    check(f"workflow smoke includes: {scenario}", f"{scenario}" in workflow)
check("scale binary is registered in Cargo manifest", 'name = "journal-replay-scale"' in manifest and 'path = "src/journal_replay_scale.rs"' in manifest)
check("crate remains dependency-free", "[dependencies]" not in manifest)
check("Cargo.lock resolves the local validator package", 'name = "civ-econ-transition-validator"' in lockfile)
check("locked Cargo input is included in report hash set", "MANIFEST_PATH, LOCKFILE_PATH, README_PATH" in audit_script)
check("README states measurements are not established yet", "has not itself established performance numbers" in " ".join(readme.split()))
check("README documents how to run source preflight", "python3 scripts/audit_journal_source_contract.py" in readme)
check("audit fails closed when Git subject is absent", 'check("Git commit identity is available"' in audit_script)
check("audit compares checked-out subject to QUALIFIED_SHA", 'if expected_subject:' in audit_script and "commit == expected_subject" in audit_script)
check("audit report records expected subject comparison", '"expected_subject": expected_subject' in audit_script and '"subject_matches_expected"' in audit_script)
check("compaction remains design-only", "design only; not implemented or qualified" in design)
check("compaction preserves acknowledged identity tombstones", "Acknowledged identities must survive" in " ".join(design.split()) and "No unresolved effect disappears" in design)

# Qualification process: exact subject, pinned toolchain, locked build and measured smoke.
check("workflow asserts checkout matches exact PR head", 'test "$(git rev-parse HEAD)" = "${QUALIFIED_SHA}"' in workflow)
check("workflow pins Rust 1.96.0 identity", "rustc +1.96.0 --version" in workflow and "rustc 1.96.0 (ac68faa20 2026-05-25)" in workflow)
check("workflow runs locked all-target tests", "cargo +1.96.0 test --manifest-path" in workflow and "--locked --all-targets" in workflow)
check("workflow runs strict Clippy", "clippy --manifest-path" in workflow and "-- -D warnings" in workflow)
check("workflow checks formatting", "cargo +1.96.0 fmt" in workflow and "-- --check" in workflow)
check("workflow records host and RSS for smoke", "/usr/bin/time -v" in workflow and "uname -a" in workflow and "df -T" in workflow)
check("workflow invokes a 10k smoke per lifecycle scenario", 'echo "runner_image=' in workflow and ' "$bin" 10000 "$scenario"' in workflow)
check("workflow writes the JSON source report to runner temp", "JOURNAL_AUDIT_REPORT_PATH:" in workflow and "journal-source-contract-report.json" in workflow)
check("workflow uploads the report even when prior steps fail", "Upload journal qualification evidence" in workflow and "if: always()" in workflow)
check("artifact upload action is pinned to reviewed v7.0.2", "actions/upload-artifact@cf430e030ddbb5b0abf93d22962f4752f3646cd9" in workflow and "# v7.0.2" in workflow)
check("artifact name includes run ID and exact subject", "github.run_id" in workflow and "env.QUALIFIED_SHA" in workflow)
check("audit writes a newline-terminated JSON report", "JOURNAL_AUDIT_REPORT_PATH" in audit_script and "destination.write_text(report_json" in audit_script and "encoding=\"utf-8\"" in audit_script)
check("workflow validates persisted JSON before any Cargo build", workflow.index("Validate persisted journal source-contract report JSON") < workflow.index("Run dependency-free validator tests"))
check("workflow captures replay smoke output in a durable log", 'report="$RUNNER_TEMP/journal-replay-smoke.log"' in workflow and '2>&1 | tee -a "$report"' in workflow)
check("smoke records the qualification subject and run ID", 'echo "qualified_sha=$QUALIFIED_SHA" | tee -a "$report"' in workflow and 'echo "github_run_id=${GITHUB_RUN_ID:-unknown}" | tee -a "$report"' in workflow)
check("smoke records host metadata before attempting the build", workflow.index('echo "runner_image=') < workflow.index("cargo +1.96.0 build --release"))
check("one artifact includes source report and runtime smoke log", "journal-source-contract-report.json" in workflow and "journal-replay-smoke.log" in workflow and "Upload journal qualification evidence" in workflow)
check("combined evidence upload remains failure-tolerant and 14-day", "Upload journal qualification evidence\n        if: always()" in workflow and "retention-days: 14" in workflow)

files = (JOURNAL_PATH, SCALE_PATH, MANIFEST_PATH, LOCKFILE_PATH, README_PATH, DESIGN_PATH, WORKFLOW_PATH, SCRIPT_PATH)
try:
    commit = subprocess.run(
        ["git", "rev-parse", "HEAD"],
        cwd=ROOT,
        check=True,
        capture_output=True,
        text=True,
    ).stdout.strip()
except (OSError, subprocess.CalledProcessError):
    commit = "unavailable"

expected_subject = os.environ.get("QUALIFIED_SHA")
check("Git commit identity is available", re.fullmatch(r"[0-9a-f]{40}", commit) is not None)
if expected_subject:
    check("audit subject matches QUALIFIED_SHA", commit == expected_subject)

report = {
    "audit": "journal-source-contract-v1",
    "commit": commit,
    "expected_subject": expected_subject,
    "github_run_id": os.environ.get("GITHUB_RUN_ID"),
    "subject_matches_expected": commit == expected_subject if expected_subject else None,
    "passed": not failures,
    "checks_passed": len(passed),
    "checks_failed": len(failures),
    "checks": passed,
    "failures": failures,
    "sha256": {
        path.relative_to(ROOT).as_posix(): hashlib.sha256(path.read_bytes()).hexdigest()
        for path in files
        if path.is_file()
    },
    "scope": (
        "Static source-contract preflight only. It does not compile Rust, execute tests, "
        "run Clippy/rustfmt, authenticate provider evidence, or establish benchmark/capacity claims."
    ),
}

report_json = json.dumps(report, indent=2, sort_keys=True)
report_destination = os.environ.get("JOURNAL_AUDIT_REPORT_PATH")
if report_destination:
    destination = Path(report_destination)
    destination.parent.mkdir(parents=True, exist_ok=True)
    destination.write_text(report_json + "\n", encoding="utf-8")

print(report_json)
sys.exit(1 if failures else 0)
