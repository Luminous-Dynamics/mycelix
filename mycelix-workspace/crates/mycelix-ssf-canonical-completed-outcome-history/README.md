# SSF Canonical Completed Outcome History v0.1

Unified append-only durable history for `CanonicalCompletedEffectEvidenceV1`, independent of the older canonical outcome-history format.

The history is keyed by the exact completed invocation record domain: `Initial`, `LegacyReplay`, or `CanonicalReplay`. Raw actuator reports never determine terminality; only the canonical qualified disposition may make a history terminal.

`OutcomeUnknown` remains non-terminal and may later refine to `Confirmed` or `ProvenNotApplied`. Terminal states are irreversible and cannot be appended through.

Historical evidence may outlive the effect authority that produced it. Recording and head-read operations still require a fresh exact store generation and trusted time, and recording/read time may not regress behind the effect or latest archived observation.

Head reads return an explicit paired latest entry `{record, manifest}` and require that record to equal the durable head, preventing sibling-manifest ambiguity by construction.

This crate creates no replay authority. Same-stable-identity store/key rotation is intentionally deferred to a separate historical reconciliation boundary rather than implicitly weakening exact-generation reads.
