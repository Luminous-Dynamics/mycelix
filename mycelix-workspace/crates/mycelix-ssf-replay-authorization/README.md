# SSF Replay Authorization v0.1

Replay is an authority-bearing event, not an automatic consequence of uncertainty or actuator idempotency.

This crate now consumes `QualifiedExactEffectOutcomeHeadEntryV1`, so replay policy cannot be reached from a merely compatible outcome-history head/manifest pair. The exact durable head record, exact paired manifest, exact paired-read receipt, and exact trusted decision time are all part of the replay subject.

`Confirmed` remains a structural hard stop. `OutcomeUnknown` can become a replay candidate only for an idempotent claim-key actuator; transactional ambiguity remains reconcile-only and non-idempotent ambiguity remains non-replayable. `ProvenNotApplied` may become a candidate under explicit local policy.

A successful token authorizes at most one future exact-same-effect attempt, requires fresh pre-invocation qualification, and still contains no effect authority.