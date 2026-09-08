# SSF No-Third-Attempt Provenance Gate v0.1

Pure structural replay-provenance gate over audited canonical completed-outcome history.

v0.1 allows replay consideration only for an `Initial` execution lineage. `LegacyReplay` and `CanonicalReplay` are already second-attempt provenance and are therefore structural hard stops before any evidence qualifier or replay-policy component is invoked.

Within `Initial` provenance, canonical `Confirmed` is non-replayable, transactional `OutcomeUnknown` is reconcile-only, and non-idempotent `OutcomeUnknown` is non-replayable. Only canonical `ProvenNotApplied` or idempotent `OutcomeUnknown` may become a non-authoritative replay-evidence candidate.

The candidate preserves the exact prior attempt, effect subject, stable effect identity, replay basis, and inherited audit validity ceiling. It creates no replay authority, no effect authority, and no third-attempt permission.
