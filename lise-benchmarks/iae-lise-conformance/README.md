# IAE -> LISE temporal conformance adapter

**Status:** synthetic cross-owner conformance evaluator, stacked on the LISE reference-model branch. It is not a production authorizer or provider adapter.

This closes a specific gap in the IAE v0.2 mapping: IAE owns claim/evidence selection and bitemporal reconstruction; LISE owns exact-operation pre-effect correspondence and preservation of unknown outcomes. The mapping had been documented but not executable.

## Qualification and comparison

See [QUALIFICATION_PROTOCOL.md](./QUALIFICATION_PROTOCOL.md) for the current research baseline, standards boundary, benchmark tiers, scorecard, prospective targets and matched SOTA comparison protocol.

## What this crate does

It consumes an IAE fixture and submission, maps the approved and pending operation projections into the existing ipb-finality-reference FinalityInput, derives authority and prior-effect states from their effective-time **and** knowledge-time histories, and calls the LISE reference relation.

It evaluates three views independently:

| View | Effective/action time | Knowledge cutoff | Expected LISE relation |
|---|---|---|---|
| PRE_DISPATCH | original action time | pre-dispatch knowledge | Permit (reference relation only) |
| POST_TIMEOUT | timeout occurrence | timeout observation | Hold(PriorOutcomeUnknown) |
| REATTEMPT_AFTER_REVOCATION | reattempt time | reattempt knowledge | Deny(AuthorityRevoked) |

The distinction matters: a later denial of a reattempt does not resolve whether the original external operation happened. The original timeout remains an UNKNOWN reconciliation obligation.

The adapter checks the write trace against the owner-supplied write boundary, including operation reference, operation/action digests, idempotency key, action, purpose, actor, target, field set, authorization reference and possible-side-effect flag. It also compares the submission finality/retry state with the source operation history.

## Run

    cargo fmt --manifest-path lise-benchmarks/iae-lise-conformance/Cargo.toml --check
    cargo check --all-targets --manifest-path lise-benchmarks/iae-lise-conformance/Cargo.toml
    cargo test --all-targets --manifest-path lise-benchmarks/iae-lise-conformance/Cargo.toml
    cargo run --manifest-path lise-benchmarks/iae-lise-conformance/Cargo.toml -- \
      lise-benchmarks/iae-lise-conformance/fixtures/iae-v0.2-fixture.json \
      lise-benchmarks/iae-lise-conformance/fixtures/iae-v0.2-reference-submission.json \
      /tmp/iae-lise-report.json

The source fixture and reference submission are copied byte-for-byte from IAE v0.2 PR #6169 at head f559026bc25bb99ba632bd04f13e8782d0f21eea. The LISE reference dependency is the local ../ipb-finality-reference crate from PR #6163 at head 3342ad472ada05175e622f873a6936d5e596d30b. Updating either input requires an intentional fixture refresh and review of the adapter mapping.

## Fail-closed properties

- Intent-baseline acceptance (accepted) is not operation approval (authorized). Unknown approval values map to ApprovalState::Unknown.
- Action, principal, target, purpose, policy, constitution and idempotency remain separate bindings.
- Authority is reconstructed from scoped status events using both effective and observed time. A revocation not yet known at the query cutoff is not silently backdated into historical knowledge.
- Submitted/timeout events are not evidence of no effect. Missing, ambiguous or incompatible history is held as unknown.
- A write outside the source write boundary fails conformance.
- The report always emits dispatch_eligible: false for this fixture.

## Critical claim boundary

The LISE Permit in the pre-dispatch synthetic case means only that the supplied fields satisfy the reference relation. The fixture uses placeholder canonical/action digests (synthetic_placeholder_not_cryptographically_verified) and synthetic authority, policy and constitution evidence. The fixture environment explicitly disables network, external I/O and mutation. Therefore this result cannot authorize a live dispatch.

A real provider integration still needs owner-native authority resolution, cryptographically verified exact-operation canonicalization, a mediated write boundary, provider contract and idempotency semantics, post-effect observation, and independent reconciliation. A digest verifies byte equality only when its source and construction are trusted; it does not authenticate the truth of a claim.

## Validation status

The crate includes adversarial unit tests. The GitHub Actions workflow checks formatting, compilation, tests and strict Clippy at the exact pull-request head. No local Rust execution is claimed in environments without Cargo/rustc; queued or skipped CI is not a pass. The dependency graph still needs a committed lockfile before release qualification.
