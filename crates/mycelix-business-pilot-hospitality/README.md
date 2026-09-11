# mycelix-business-pilot-hospitality

Preregistered, read-only field-pilot composition for hospitality demand forecasting.

This crate binds the generic Business Shadow, Field Qualification, Ingress, and Hospitality Profile contracts into one reproducible pilot registration. It does not execute orders, schedules, refunds, payments, or any other business mutation.

## Conservative manual-export policy v1

The built-in starting policy requires:

- at least 24 hours between registration and evaluation start;
- at least 28 days of holdout evaluation;
- at least 56 forecast cases overall;
- at least 10 cases in every preregistered slice;
- no more than 10% candidate abstention;
- no more than 1% missing sales observations;
- zero conflicting sales observations;
- no more than 2% stale sales observations;
- no more than 36 hours maximum observed ingest delay for the manual/daily-export lane;
- candidate forecast not worse than the pinned same-weekday/daypart seasonal-naive baseline.

These are versioned pilot defaults, not universal hospitality rules. Different thresholds must be preregistered before evidence collection.

## Default slices

The default v1 slice set is timezone-bound and includes breakfast (05:00–11:00), lunch (11:00–15:00), evening (15:00–23:00), and weekend. Each deterministic slice predicate receives its own digest; changing timezone or daypart semantics changes the qualification lineage.

## Connector lineage

The registration binds:

- source-system identity;
- adapter semantic identity;
- adapter release/build digest;
- mapping-semantics digest;
- source-schema digest.

The source-schema digest is included in the stronger registration/field-plan digest even though the generic field connector projection is narrower. A changed export shape therefore cannot silently reuse previous field evidence.

Passing this pilot remains A0–A2 evidence for one scope and one forecast lineage. It grants no institutional authority and establishes no causal claim about savings, profit, staffing, or purchasing outcomes.
