# FSRE service-universe audit v0.1

**Status:** Deterministic, read-only reconciliation of one observed frame against one declared reference frame. It does not establish that the reference frame is exhaustive, authenticate source owners or estimate population coverage.

## Run

    cargo test --locked --offline --manifest-path tools/fsre-service-universe-audit/Cargo.toml
    cargo run --quiet --locked --offline --manifest-path tools/fsre-service-universe-audit/Cargo.toml -- tools/fsre-service-universe-audit/examples/three-frame-reference.psv

The CLI exits 0 only for an exact reconciliation to the declared enumerated reference frame. It exits 1 for a valid but incomplete, conflicted or non-estimable audit, and 2 for invalid input or invocation.

## Input contract

The pipe-delimited input begins with strict #key=value metadata and this exact table header:

    record_type|record_id|scope_status|reference_record_id|decision|evidence_owner|evidence_ref|valid_from_seconds|valid_until_seconds

Rows:
- REFERENCE: one expected unit in the declared reference frame.
- OBSERVED: one unit in the comparison frame, with IN_SCOPE, OUT_OF_SCOPE or UNKNOWN scope status.
- LINK: an evidence-backed disposition for one observed record.

Decisions:
- MATCH: one in-scope observed record maps to one reference record.
- UNRESOLVED: identity is not established; an optional reference ID may identify a candidate.
- NO_MATCH_IN_SCOPE: the adjudicator says the observed record is in scope but is not represented by the reference frame.
- OUT_OF_SCOPE: the adjudicator explicitly excludes the observed record from the target scope.

Every observed record requires an explicit link disposition. Multiple unresolved candidate links are allowed, but may not be mixed with resolved dispositions.

All frame, record and crosswalk evidence intervals use half-open [from, until) semantics and must cover the full study horizon. The reference basis must be DECLARED_ENUMERATED_REGISTER for an exact reconciliation result. SAMPLE and UNKNOWN are explicitly non-estimable in v0.1.

## Output boundary

RECONCILED_TO_DECLARED_REFERENCE_FRAME means the observed records reconcile one-to-one to the supplied reference register. The report always retains:
- reference_population_completeness=NOT_ESTABLISHED
- population_coverage_estimate=NOT_COMPUTED
- source_authentication=ASSERTED_NOT_AUTHENTICATED

This is a structural reference, not a legal authorization, independent source verifier, field audit, statistical estimator or operational decision engine. See research/fsre/SERVICE_UNIVERSE_COVERAGE_AUDIT_BENCHMARK_V0_1.md for the benchmark and limits.
