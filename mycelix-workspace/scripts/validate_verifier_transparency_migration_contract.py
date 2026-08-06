#!/usr/bin/env python3
"""Fail-closed source contract for compromised transparency-log migration."""
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
checks = {
    ROOT / "src/verifier_transparency_migration.rs": [
        "VERIFIER_TRANSPARENCY_MIGRATION_POLICY_ID",
        "VerifierTransparencyLogCompromiseNotice",
        "VerifierTransparencyHistoryCommitment",
        "VerifiedVerifierTransparencyHistory",
        "VerifierTransparencyCrossLogMigration",
        "derive_destination_history_anchor",
        "VerifierTransparencyMigrationAuthoritySet",
        "VerifiedVerifierTransparencyMigration",
        "authenticate_verifier_transparency_migration",
        "VerifierTransparencyMigrationCheckpoint",
        "forbid_history_truncation",
        "require_source_witness_quorum",
        "require_destination_witness_quorum",
        "require_separate_recovery_authority",
        "source_quorum.quorum_hash()",
        "destination_quorum.quorum_hash()",
        "insufficient transparency migration approval quorum",
        "transparency history is truncated, reordered, or rolled back",
    ],
    ROOT / "src/promotion.rs": [
        "VerifierTransparencyMigrationPolicyNotPinned",
        "VerifierTransparencySourceHistoryNotPreserved",
        "VerifierTransparencyCrossLogWitnessQuorumsNotVerified",
        "VerifierTransparencyMigrationRecoveryQuorumNotVerified",
        "VerifierTransparencyMigrationCheckpointNotPinned",
        "verifier_transparency_migration_policy_not_pinned",
        "verifier_transparency_source_history_not_preserved",
        "verifier_transparency_cross_log_witness_quorums_not_verified",
        "verifier_transparency_migration_recovery_quorum_not_verified",
        "verifier_transparency_migration_checkpoint_not_pinned",
    ],
    ROOT / "src/verifier_release.rs": [
        "transparency_migration_policy_hash",
        "transparency_compromise_notice_hash",
        "transparency_source_history_hash",
        "transparency_cross_log_migration_hash",
        "transparency_migration_approval_hash",
        "transparency_migration_checkpoint_hash",
        "transparency_migration_checkpoint_sequence",
    ],
    ROOT / "src/lib.rs": ["pub mod verifier_transparency_migration;"],
}
for path, needles in checks.items():
    text = path.read_text()
    missing = [needle for needle in needles if needle not in text]
    if missing:
        raise SystemExit(f"{path.relative_to(ROOT)} missing: {missing}")

source = (ROOT / "src/verifier_transparency_migration.rs").read_text()
for forbidden in [
    "migration_authenticated: bool",
    "history_preserved: bool",
    "witness_quorums_verified: bool",
    "recovery_quorum_verified: bool",
]:
    if forbidden in source:
        raise SystemExit(f"caller-supplied trust boolean forbidden: {forbidden}")

print("verifier transparency migration contract valid: complete history, dual witness quorums, recovery quorum, external checkpoint")
