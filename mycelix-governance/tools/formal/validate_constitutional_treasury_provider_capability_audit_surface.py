#!/usr/bin/env python3
"""Audit the F0C public retry/audit surface independently of the profile validator."""

from __future__ import annotations

import argparse
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
SOURCE_PATH = ROOT / "mycelix-governance/crates/constitutional-treasury-effect-provider/src/capability_public.rs"
MANIFEST_PATH = ROOT / "mycelix-governance/crates/constitutional-treasury-effect-provider/Cargo.toml"


def fail(message: str) -> None:
    raise ValueError(message)


def validate(source: str, manifest: str) -> None:
    if 'path = "src/capability_public.rs"' not in manifest:
        fail("capability facade is not the crate entrypoint")

    use_start = source.find("pub use f0::{")
    use_end = source.find("};", use_start)
    if use_start < 0 or use_end < 0:
        fail("F0 re-export block missing")
    use_block = source[use_start:use_end]
    if "RetryBasis" in use_block:
        fail("raw F0 RetryBasis leaked through public re-export")
    if "EffectAttempt" in use_block:
        fail("raw F0 EffectAttempt leaked through public re-export")

    if "pub enum AttemptRetryEvidence {" not in source:
        fail("descriptive attempt retry evidence wrapper missing")
    if "pub struct EffectAttempt {" not in source:
        fail("public attempt snapshot wrapper missing")
    attempt_start = source.find("pub struct EffectAttempt {")
    attempt_end = source.find("}\n\nimpl From<f0::EffectAttempt>", attempt_start)
    if attempt_end < 0:
        fail("attempt snapshot conversion boundary missing")
    attempt_struct = source[attempt_start:attempt_end]
    if "retry_evidence: AttemptRetryEvidence" not in attempt_struct:
        fail("attempt snapshot does not expose descriptive retry evidence")
    if "retry_basis" in attempt_struct or "f0::RetryBasis" in attempt_struct:
        fail("private retry authority type leaked into attempt snapshot")

    conversion_start = source.find("impl From<f0::EffectAttempt> for EffectAttempt")
    conversion_end = source.find("/// Capability-gated effect record.", conversion_start)
    if conversion_start < 0 or conversion_end < 0:
        fail("attempt conversion implementation missing")
    conversion = source[conversion_start:conversion_end]
    for required in (
        "f0::RetryBasis::Initial",
        "f0::RetryBasis::ProviderReplayQualified",
        "f0::RetryBasis::ReconciledNoEffect",
        "AttemptRetryEvidence::Initial",
        "AttemptRetryEvidence::ProviderReplayQualified",
        "AttemptRetryEvidence::ReconciledNoEffect",
    ):
        if required not in conversion:
            fail(f"attempt retry evidence conversion incomplete: {required}")

    for signature in (
        "retry_basis: RetryBasis,",
        "let inner_basis = retry_basis.into_f0(&self.inner)?;",
        "let inner_basis = retry_basis.into_f0(&snapshot)?;",
    ):
        if signature not in source:
            fail(f"capability retry gate bypassed: {signature}")

    if "retry_basis: AttemptRetryEvidence" in source:
        fail("descriptive retry evidence became an authorization input")
    if "pub fn record_mut(" in source:
        fail("mutable raw record escape hatch exposed")


def expect_rejected(name: str, source: str, manifest: str) -> None:
    try:
        validate(source, manifest)
    except ValueError:
        return
    raise AssertionError(f"audit-surface mutation survived: {name}")


def self_test(source: str, manifest: str) -> None:
    expect_rejected(
        "raw-retry-reexport",
        source.replace("REQUEST_COMMITMENT_PREFIX, TREASURY_EFFECT_SCHEMA_VERSION, EffectIntegrityFault,", "REQUEST_COMMITMENT_PREFIX, TREASURY_EFFECT_SCHEMA_VERSION, RetryBasis, EffectIntegrityFault,", 1),
        manifest,
    )
    expect_rejected(
        "raw-attempt-reexport",
        source.replace("REQUEST_COMMITMENT_PREFIX, TREASURY_EFFECT_SCHEMA_VERSION, EffectIntegrityFault,", "REQUEST_COMMITMENT_PREFIX, TREASURY_EFFECT_SCHEMA_VERSION, EffectAttempt, EffectIntegrityFault,", 1),
        manifest,
    )
    expect_rejected(
        "retry-authority-leak",
        source.replace("pub retry_evidence: AttemptRetryEvidence,", "pub retry_basis: f0::RetryBasis,", 1),
        manifest,
    )
    expect_rejected(
        "audit-evidence-as-input",
        source.replace("retry_basis: RetryBasis,", "retry_basis: AttemptRetryEvidence,", 1),
        manifest,
    )
    expect_rejected(
        "entrypoint-bypass",
        source,
        manifest.replace('path = "src/capability_public.rs"', 'path = "src/public.rs"', 1),
    )


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--self-test", action="store_true")
    args = parser.parse_args()

    source = SOURCE_PATH.read_text()
    manifest = MANIFEST_PATH.read_text()
    validate(source, manifest)
    if args.self_test:
        self_test(source, manifest)
    print("validated treasury provider capability audit surface")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
