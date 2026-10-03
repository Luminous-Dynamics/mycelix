# SYM-CIVIC-003 — immutable study-execution evidence contract v1

Status: synthetic research provenance contract only

Parent: SYM-CIVIC-002 evidence capsule / `8b54602821d10e9b9cab46ece33d80e04045e90e`

Tracking issue: #3839

## Purpose

Make a concrete research execution auditable without confusing execution provenance with scientific truth.

A study-execution receipt answers **what was bound and executed**. It does not answer whether the hypothesis is true, whether an intervention is effective, or whether any authority should act.

## Required execution binding

A valid execution evidence record binds distinct dimensions:

`study_id + input_snapshot + upstream_evidence_cut + estimand + model_identity + analysis_commit + environment_identity + execution_identity + configuration_digest + randomness + temporal_evidence + uncertainty + missingness_policy + identification_assumptions + diagnostics + sensitivity_analyses + spillover_displacement_checks + alternative_explanations + output_disposition + result_artifact`

## Invariants

- Mutable `latest`, branch-only, or floating references cannot establish scientific provenance.
- Input identity, analysis code, model identity, environment identity, configuration identity, and execution identity are separately bound.
- Canonical serialization is required before cryptographic hashing.
- Missing or unavailable provenance is explicit and never silently defaulted.
- Deterministic and stochastic execution modes are distinguished.
- A claimed replication must state whether identical bytes, identical seed, or a stochastic distribution is being reproduced.
- Result artifacts are bound by digest; a changed artifact invalidates the original result binding.
- Receipt metadata does not copy protected research data; it binds approved artifacts and describes their access/protection status.
- Output disposition is an analysis result field, not evidence that an intervention is authorized or effective.

## Adversarial qualification corpus

SE-01 stale input substitution
SE-02 analysis-code drift
SE-03 model-version drift
SE-04 environment drift
SE-05 omitted randomness policy
SE-06 hidden configuration
SE-07 result-artifact digest mismatch
SE-08 nondeterministic execution presented as deterministic
SE-09 oracle/verdict embedded in input evidence
SE-10 mutable branch-only provenance
SE-11 protected-data leakage into receipt
SE-12 correctly declared stochastic replication
SE-13 canonicalization order invariance
SE-14 complete deterministic execution binding

The fixture corpus contains candidate execution-evidence records and reference conditions, but no embedded expected disposition. The qualifier derives acceptance/rejection independently.

## Qualification ceiling

PASS establishes only that this synthetic corpus detects the declared provenance failures and preserves research-only semantics.

It does not establish reproducibility of a real scientific study, causal effectiveness, clinical validity, suicide prediction/prevention, violence prediction/reduction, public safety, municipal legitimacy, or deployment readiness.
