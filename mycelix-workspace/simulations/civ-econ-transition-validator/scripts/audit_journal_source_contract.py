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
TRANSITION_PATH = CRATE / "src/lib.rs"
RECOVERY_POLICY_PATH = CRATE / "src/external_effect_policy.rs"
CRASH_TEST_PATH = CRATE / "tests/crash_recovery.rs"
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


def ordered_tokens(source: str, *tokens: str) -> bool:
    """Require every token to exist and appear in the requested strict order."""
    positions = [source.find(token) for token in tokens]
    return all(position >= 0 for position in positions) and all(
        left < right for left, right in zip(positions, positions[1:])
    )


def read(path: Path) -> str:
    if not path.is_file():
        failures.append(f"required file exists: {path.relative_to(ROOT)}")
        return ""
    return path.read_text(encoding="utf-8")


journal = read(JOURNAL_PATH)
scale = read(SCALE_PATH)
transition = read(TRANSITION_PATH)
recovery_policy = read(RECOVERY_POLICY_PATH)
crash_test = read(CRASH_TEST_PATH)
manifest = read(MANIFEST_PATH)
lockfile = read(LOCKFILE_PATH)
readme = read(README_PATH)
design = read(DESIGN_PATH)
workflow = read(WORKFLOW_PATH)
audit_script = read(SCRIPT_PATH)

# Replay resource bounds and parser structure.
check("record limit is 4096 bytes", "MAX_JOURNAL_RECORD_BYTES: usize = 4096;" in journal)
check("effect ID limit is 1024 bytes", "MAX_EFFECT_ID_BYTES: usize = 1024;" in journal)
check("idempotency key limit is 768 bytes", "MAX_IDEMPOTENCY_KEY_BYTES: usize = 768;" in journal)
check(
    "journal parent directory rejects group/other write access on Unix",
    "ensure_secure_parent_directory(&parent)?;" in journal
    and "metadata.permissions().mode() & 0o022 != 0" in journal
    and "InsecureDirectory" in journal,
)
check(
    "journal parent directory gate precedes lock creation",
    ordered_tokens(
        journal,
        "let parent = fs::canonicalize(requested_parent)",
        "ensure_secure_parent_directory(&parent)?;",
        "let lock_path = parent.join(",
        "let mut lock_file = lock_options.open(&lock_path)",
    ),
)
check(
    "public journal API documents key confidentiality and native ACL boundary",
    "The idempotency_key parameter is the exact original key" in journal
    and "recoverable (hex-encoded, not encrypted) form" in journal
    and "Non-Unix native ACLs are not inspected by this crate." in journal,
)
check(
    "J4 begin intent is synced before Started is returned",
    ordered_tokens(
        journal,
        '"J4\\tB\\t{}\\t{}\\t{}\\t{}\\n"',
        "self.append_and_sync(&record)?;",
        "self.entries.insert(id.to_owned(), JournalEntry {",
        "Ok(BeginResult::Started)",
    ),
)
check(
    "acknowledged recovery results carry the original-key binding",
    journal.count("idempotency_key_binding: IdempotencyKeyBinding,") == 3
    and journal.count("idempotency_key_binding: existing.idempotency_key_binding.clone()") == 2
    and "already_acknowledged_indeterminate_result_returns_original_key_binding" in journal,
)
check(
    "journal writes explicit J4 original-key bindings",
    '"J4\\tB' in journal
    and '"J4\\tI' in journal
    and '"J4\\tA' in journal
    and "pub fn idempotency_key_binding" in journal
    and "IdempotencyKeyBinding::NotUsed" in journal,
)
check("replay uses the bounded record reader", "read_bounded_record(reader, &mut record, line_no + 1)?" in journal)
check("bounded reader uses fill_buf/consume", re.search(r"reader\s*\.fill_buf\(\)", journal) is not None and "reader.consume(consumed)" in journal)
check("no full-file read_to_end replay path", "read_to_end(" not in journal)
check("hex parser uses pinned-toolchain is_multiple_of idiom", "!value.len().is_multiple_of(2)" in journal)
check("record fields are borrowed rather than collected per line", "let mut fields = line.split('\\t');" in journal and "let fields: Vec<&str> = line.split" not in journal)
match_start = journal.find("let (provider_profile_digest, key_binding_field, receipt_digest, source_evidence_digest) =")
match_end = journal.find("let idempotency_key_binding = match key_binding_field", match_start)
check(
    "record-shape match closes exactly once",
    match_start >= 0 and match_end > match_start and journal[match_start:match_end].count("};") == 1,
)
check(
    "J4 replay rejects key-binding changes in both non-begin transitions",
    journal.count("entry.idempotency_key_binding != idempotency_key_binding") == 2
    and "new begin record cannot leave idempotency-key binding unknown" in journal,
)
check(
    "J4 parser rejects unknown, malformed, and oversized new-key records",
    all(
        name in journal
        for name in (
            "j4_begin_rejects_legacy_unbound_key_marker",
            "j4_replay_rejects_malformed_idempotency_key_encoding",
            "j4_replay_rejects_oversized_encoded_key_before_decoding",
            "idempotency key is not canonical lowercase hex",
            "idempotency key exceeds maximum size",
        )
    ),
)
check(
    "legacy J3 and J4 transition fixtures use real newline delimiters",
    r'format!("J3\tB\t{}\t{}\t{}\n", encoded_id, REQUEST_A, PROVIDER_A)' in journal
    and r'"J4\tB\t{}\t{}\t{}\t{}\nJ4\tA\t{}\t{}\t{}\t{}\t{}\t{}\n",' in journal
    and r'format!("J3\tB\t{}\t{}\t{}\\n", encoded_id, REQUEST_A, PROVIDER_A)' not in journal
    and r'"J4\tB\t{}\t{}\t{}\t{}\\nJ4\tA' not in journal,
)
check(
    "J4 maximum ID/key acknowledgement stays within the record ceiling",
    "maximum_effect_id_and_idempotency_key_fit_bounded_ack_record" in journal
    and "record.len() <= MAX_JOURNAL_RECORD_BYTES" in journal
    and "MAX_IDEMPOTENCY_KEY_BYTES: usize = 768;" in journal,
)
check(
    "explicit no-key marker cannot be silently changed to a key",
    "explicit_no_key_binding_survives_restart_and_rejects_later_key" in journal
    and "Some(IdempotencyKeyBinding::NotUsed)" in journal
    and "Err(JournalError::IdempotencyKeyConflict)" in journal,
)
check(
    "encoded effect ID limit is checked before decoding",
    ordered_tokens(
        journal,
        "if encoded_id.len() > MAX_EFFECT_ID_BYTES * 2",
        "let id_bytes = decode_hex(encoded_id, line_no)?",
    ),
)
for test_name in (
    "streaming_replay_handles_history_larger_than_reader_buffer",
    "streaming_replay_rejects_truncated_final_record_after_valid_history",
    "replay_rejects_extra_record_fields_without_ignoring_them",
    "effect_count_includes_all_states_and_survives_replay",
    "original_idempotency_key_binding_survives_restart_and_is_immutable",
    "maximum_effect_id_and_idempotency_key_fit_bounded_ack_record",
    "legacy_j3_pending_effect_keeps_original_key_unbound",
    "j4_replay_rejects_key_change_between_begin_and_ack",
    "explicit_no_key_binding_survives_restart_and_rejects_later_key",
    "j4_replay_rejects_key_change_between_begin_and_indeterminate",
    "invalid_original_idempotency_key_is_rejected_before_append",
    "group_or_other_writable_parent_directory_is_rejected_before_lock_creation",
    "already_acknowledged_indeterminate_result_returns_original_key_binding",
    "j4_begin_rejects_legacy_unbound_key_marker",
    "j4_replay_rejects_malformed_idempotency_key_encoding",
    "j4_replay_rejects_oversized_encoded_key_before_decoding",
):
    annotated_test = re.search(r"#\[test\]\s*fn\s+" + re.escape(test_name) + r"\(", journal)
    check(f"regression test is annotated: {test_name}", annotated_test is not None)

# Cross-check that the state machine and external recovery policy remain in the audit set.
check(
    "transition safety facts are monotonic before recovery gating",
    ordered_tokens(
        transition,
        "if ev.cutover_effects_started",
        "self.effects_may_have_started = true;",
        "let effect_recovery_transition = matches!",
        "if self.effects_may_have_started && !self.effect_frontier_resolved",
    ),
)
check(
    "unknown external outcome remains indeterminate without a live key",
    "Ok(None) => RecoveryAction::RemainIndeterminate" in recovery_policy
    and "outcome is unknown and no live idempotency guarantee permits a retry" in recovery_policy,
)
check(
    "retry policy binds exact provider and request and respects idempotency deadline",
    "provider_profile_digest != expected_provider_profile_digest" in recovery_policy
    and "request_digest != expected_request_digest" in recovery_policy
    and "now_unix_seconds >= *guaranteed_until_unix_seconds" in recovery_policy,
)
check(
    "same-key recovery retry must match original dispatch key",
    "expected_idempotency_key: Option<&str>" in recovery_policy
    and "Some(original_key) if original_key == idempotency_key" in recovery_policy
    and "provider idempotency key differs from original attempt" in recovery_policy
    and "original attempt has no idempotency-key binding" in recovery_policy
    and "authoritative durable dispatch record" in recovery_policy,
)
for test_name in (
    "unknown_outcome_rejects_a_different_key_even_for_the_same_request",
    "unknown_outcome_cannot_retry_when_original_key_was_not_recorded",
):
    annotated_test = re.search(
        r"#\[test\]\s*fn\s+" + re.escape(test_name) + r"\(",
        recovery_policy,
    )
    check(f"original-key recovery regression test is annotated: {test_name}", annotated_test is not None)
check(
    "no-effect retry requires no in-flight attempt and a fresh observation",
    "if !no_in_flight_attempt" in recovery_policy
    and "now_unix_seconds - *observed_at_unix_seconds" in recovery_policy
    and "max_no_effect_observation_age_seconds" in recovery_policy,
)
check(
    "crash recovery integration test remains present and exercises process exit",
    "Command::new" in crash_test
    and "process::exit" in crash_test
    and "crash_between_dispatch_and_acknowledgement_reconciles_without_duplicate_dispatch" in crash_test
    and "acknowledged_receipt_survives_process_exit_before_caller_observes_success" in crash_test,
)

# Scale probe and build registration must describe the same four lifecycle paths.
scenarios = ("pending", "indeterminate", "acknowledged", "reconciled")
check("scale probe has the one-million-effect ceiling", "const MAX_EFFECTS: usize = 1_000_000;" in scale)
check(
    "scale probe generates current J4 records with explicit no-key binding",
    '"J4\\tB' in scale and '"J4\\tI' in scale and '"J4\\tA' in scale
    and "synthetic J4 journal" in scale,
)
check("scale probe verifies total count and unresolved count", "journal.effect_count()" in scale and "journal.unresolved_effect_count()" in scale)
check("scale probe verifies first and last states", "first_status_matches" in scale and "last_status_matches" in scale)
check("scale probe writes and syncs its generated journal before replay", "writer.flush()?;" in scale and "writer.get_ref().sync_all()?;" in scale)
for scenario in scenarios:
    check(f"scale probe scenario is implemented: {scenario}", f'"{scenario}"' in scale)
    check(f"workflow smoke includes: {scenario}", f"{scenario}" in workflow)
check("scale binary is registered in Cargo manifest", 'name = "journal-replay-scale"' in manifest and 'path = "src/journal_replay_scale.rs"' in manifest)
check("crate remains dependency-free", "[dependencies]" not in manifest)
check("Cargo.lock resolves the local validator package", 'name = "civ-econ-transition-validator"' in lockfile)
hash_list_start = audit_script.find("\nfiles = (\n    JOURNAL_PATH,")
hash_list_end = audit_script.find("\n)\n", hash_list_start)
hash_list_source = audit_script[hash_list_start:hash_list_end + 3] if hash_list_start >= 0 and hash_list_end > hash_list_start else ""
check(
    "report hash set includes manifest, lockfile, and documentation",
    all(name in hash_list_source for name in ("MANIFEST_PATH", "LOCKFILE_PATH", "README_PATH", "DESIGN_PATH")),
)
check("README states measurements are not established yet", "has not itself established performance numbers" in " ".join(readme.split()))
check(
    "README documents J4 writes, key security, and legacy binding uncertainty",
    "writes J4 records" in " ".join(readme.split())
    and "legacy J1/J2/J3 entries replay with no original-key binding" in " ".join(readme.split())
    and "768 idempotency-key bytes" in " ".join(readme.split())
    and "not encrypted" in " ".join(readme.split())
    and "parent directory must not be group/other-writable" in " ".join(readme.split())
    and "Both AlreadyAcknowledged results expose the original-key binding" in " ".join(readme.split())
)
check("README documents how to run source preflight", "python3 scripts/audit_journal_source_contract.py" in readme)
check(
    "audit ordering helper rejects missing or reversed markers",
    ordered_tokens("first ... second", "first", "second")
    and not ordered_tokens("present", "absent", "present")
    and not ordered_tokens("second first", "first", "second"),
)
check("README documents combined report/smoke artifact and limits", "JSON audit report and replay-smoke log are bundled" in " ".join(readme.split()) and "not immutable long-term archival" in " ".join(readme.split()) and "smoke figures are not capacity benchmarks" in " ".join(readme.split()))
check(
    "README documents J4 original-key persistence and remaining adapter boundary",
    "J4 records now persist the original key" in " ".join(readme.split())
    and "not encrypted" in " ".join(readme.split())
    and "must read the J4 key binding" in " ".join(readme.split())
    and "provider observations/contracts still require independent authentication" in " ".join(readme.split()),
)
audit_exec_start = audit_script.find("\ntry:\n    commit = subprocess.run", hash_list_start)
audit_identity_start = audit_script.find('expected_subject = os.environ.get("QUALIFIED_SHA")', audit_exec_start)
audit_report_start = audit_script.find("\nreport = {", audit_identity_start)
audit_report_json_start = audit_script.find("report_json = json.dumps(report", audit_report_start)
audit_identity_source = audit_script[audit_exec_start:audit_report_start]
audit_report_source = audit_script[audit_report_start:audit_report_json_start]
check(
    "audit fails closed when Git subject is absent",
    'check("Git commit identity is available"' in audit_identity_source,
)
check(
    "audit compares checked-out subject to QUALIFIED_SHA",
    "if expected_subject:" in audit_identity_source
    and "commit == expected_subject" in audit_identity_source,
)
check(
    "audit report records expected subject comparison",
    '"expected_subject": expected_subject' in audit_report_source
    and '"subject_matches_expected": commit == expected_subject if expected_subject else None' in audit_report_source,
)
check("compaction remains design-only", "design only; not implemented or qualified" in design)
check(
    "compaction preserves identity tombstones and original-key states",
    "Acknowledged identities must survive" in " ".join(design.split())
    and "No unresolved effect disappears" in design
    and "Bound(key), explicit NotUsed, or LegacyUnbound" in " ".join(design.split())
    and "original idempotency-key binding" in " ".join(design.split())
    and "hex encoding is not encryption" in " ".join(design.split()),
)

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
audit_output_source = audit_script[audit_report_json_start:]
check(
    "audit writes a newline-terminated JSON report",
    'report_destination = os.environ.get("JOURNAL_AUDIT_REPORT_PATH")' in audit_output_source
    and 'destination.write_text(report_json + "\\n", encoding="utf-8")' in audit_output_source,
)
check(
    "workflow validates persisted JSON before any Cargo build",
    ordered_tokens(
        workflow,
        "name: Validate persisted journal source-contract report JSON",
        "name: Run dependency-free validator tests",
    ),
)
check(
    "workflow independently requires a passing report bound to the exact subject",
    all(
        marker in workflow
        for marker in (
            'assert report.get("passed") is True',
            'assert report.get("checks_failed") == 0',
            'assert report.get("commit") == expected_sha',
            'assert report.get("expected_subject") == expected_sha',
            'assert report.get("subject_matches_expected") is True',
            'assert report.get("checks_passed") == len(report.get("checks", []))',
            'assert report.get("checks_failed") == len(report.get("failures", []))',
            'assert isinstance(report.get("sha256"), dict) and report["sha256"]',
        )
    ),
)
check(
    "workflow binds report type and run ID to the current invocation",
    'assert report.get("audit") == "journal-source-contract-v1"' in workflow
    and 'assert report.get("github_run_id") == expected_run_id' in workflow,
)
check(
    "workflow requires valid SHA-256 fingerprints for every core implementation file",
    all(
        marker in workflow
        for marker in (
            're.fullmatch(r"[0-9a-f]{64}", digest)',
            'assert required_paths <= fingerprints.keys()',
            '"mycelix-workspace/simulations/civ-econ-transition-validator/src/lib.rs"',
            '"mycelix-workspace/simulations/civ-econ-transition-validator/src/external_effect_policy.rs"',
            '"mycelix-workspace/simulations/civ-econ-transition-validator/tests/crash_recovery.rs"',
        )
    ),
)
check("workflow captures replay smoke output in a durable log", 'report="$RUNNER_TEMP/journal-replay-smoke.log"' in workflow and '2>&1 | tee -a "$report"' in workflow)
check("smoke records the qualification subject and run ID", 'echo "qualified_sha=$QUALIFIED_SHA" | tee -a "$report"' in workflow and 'echo "github_run_id=${GITHUB_RUN_ID:-unknown}" | tee -a "$report"' in workflow)
check(
    "smoke records host metadata before attempting the build",
    ordered_tokens(workflow, 'echo "runner_image=', "cargo +1.96.0 build --release"),
)
check("smoke pipeline fails on command failures", "set -euo pipefail" in workflow)
check("smoke log captures non-zero probe status through tee", "/usr/bin/time -v \"$bin\" 10000 \"$scenario\" 2>&1 | tee -a \"$report\"" in workflow and "set -euo pipefail" in workflow)
check("README documents why smoke uses pipefail", "`set -euo pipefail`" in readme and "cannot be hidden by a successful `tee` process" in " ".join(readme.split()))
check("one artifact includes source report and runtime smoke log", "journal-source-contract-report.json" in workflow and "journal-replay-smoke.log" in workflow and "Upload journal qualification evidence" in workflow)
check("combined evidence upload remains failure-tolerant and 14-day", "Upload journal qualification evidence\n        if: always()" in workflow and "retention-days: 14" in workflow)

files = (
    JOURNAL_PATH,
    SCALE_PATH,
    TRANSITION_PATH,
    RECOVERY_POLICY_PATH,
    CRASH_TEST_PATH,
    MANIFEST_PATH,
    LOCKFILE_PATH,
    README_PATH,
    DESIGN_PATH,
    WORKFLOW_PATH,
    SCRIPT_PATH,
)
check(
    "report hashes include transition, recovery policy, and crash integration source",
    all(
        name in hash_list_source
        for name in (
            "TRANSITION_PATH,",
            "RECOVERY_POLICY_PATH,",
            "CRASH_TEST_PATH,",
        )
    ),
)
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
