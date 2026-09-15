# AC-002 — Capture Observation Contract

Status: draft implementation contract

Parent: AC-001 constitutional anti-capture kernel

## Purpose

AC-002 defines the epistemic boundary between measurements of institutional concentration and any later human/legal finding. It is intentionally non-adjudicative.

The permitted chain is:

`measurement -> observation -> hypothesis -> signal -> human review`

AC-002 does not define punishment, guilt, rights removal, or sovereign authority.

## Design rules

1. **Measure institutions and processes, not people.** `CaptureSubject` has no person-level variant.
2. **Require reproducibility.** Every observation names a metric, deterministic value, unit, method reference, and source provenance.
3. **Make uncertainty explicit.** Every observation carries a confidence assessment and at least one limitation.
4. **Require competing explanations.** A capture hypothesis must cite measured observations and at least one alternative explanation with evidence needed to test it.
5. **Keep signals non-adjudicative.** A `CaptureSignal` is review-priority evidence only. `authorizes_consequence=true` is invalid.
6. **Make review provenance visible.** Reviewed signals require reviewer references; terminal review states require rationale.
7. **Preserve AC-001 semantics.** Observations map to `ConsequenceBasis::Observation`; signals map to `ConsequenceBasis::CaptureSignal`. Neither becomes `AdjudicatedFinding` in this module.

## Why this separation exists

Anti-corruption indicators are useful precisely because they can identify anomalous or high-risk patterns before wrongdoing is proven. The same property makes them dangerous if a system silently turns them into guilt scores. AC-002 therefore treats red flags as prompts for review rather than conclusions.

## Initial metric vocabulary

- authority concentration
- delegation concentration
- influence opacity
- ownership concentration
- procurement supplier concentration
- procurement single-bid share
- evidence deficit
- contestability deficit
- exit deficit
- explicitly named custom metrics

The vocabulary is intentionally extensible, but anonymous custom metrics are rejected.

## Deterministic measurements

`MetricValue` uses an integer numerator and denominator instead of floating point so evidence can be serialized and reproduced exactly. Example:

`2350 / 10000, unit=share`

represents 23.50% without binary floating-point ambiguity.

## Review states

AC-002 supports only non-adjudicative review states:

- `Unreviewed`
- `UnderReview`
- `SupportedForFurtherInquiry`
- `NeedsMoreEvidence`
- `Rejected`

There is intentionally no `Guilty`, `Corrupt`, `Sanctioned`, or `AdjudicatedFinding` state.

## Fail-closed boundaries

Validation rejects at least:

- missing IDs or subjects;
- zero measurement denominators;
- unnamed metrics or units;
- missing method references;
- missing/invalid provenance;
- empirical confidence outside 0..=10,000 basis points;
- uncertainty with no limitations;
- hypotheses without supporting observations;
- hypotheses without alternative explanations;
- signals with no measured observations;
- signals that claim authority to impose consequences;
- reviewed signals without reviewer provenance;
- completed review states without rationale.

## Non-goals

AC-002 does not:

- determine whether a person or organization is corrupt;
- create a universal trust/reputation score;
- define procurement red-flag formulas;
- define legal burdens of proof;
- define sanctions;
- choose policy;
- authorize AI to adjudicate rights;
- replace auditors, courts, regulators, or democratic institutions.

## Qualification gate

Before AC-002 can be treated as qualified infrastructure:

1. exact-subject `cargo test -p civic-types` must pass;
2. warnings-denied Clippy for the crate must pass;
3. serialization fixtures should be frozen for the public AC-002 types;
4. mutation tests should prove that removing provenance, alternatives, uncertainty, or consequence separation causes failures;
5. an independent review should verify that no AC-002 API path can manufacture `ConsequenceBasis::AdjudicatedFinding`.

## Next tranche

AC-003 should introduce the institutional relationship graph: authority, delegation, ownership, influence, procurement, disclosure, appeal, and audit edges. AC-003 should consume AC-002 observations/signals without increasing their epistemic authority.
