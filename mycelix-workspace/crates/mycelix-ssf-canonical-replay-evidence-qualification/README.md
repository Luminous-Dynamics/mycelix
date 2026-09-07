# SSF Canonical Replay Evidence Qualification v0.1

Freshly reading old canonical outcome evidence does not make that evidence fresh enough to create new authority.

This crate structurally determines whether the exact canonical history head may even be considered for replay, then requires an independently expected replay-evidence qualifier to attest the exact paired head/entry under a current policy generation.

Structural stops happen before the qualifier is called:

- canonical `Confirmed` cannot replay;
- transactional `OutcomeUnknown` is reconcile-only;
- non-idempotent `OutcomeUnknown` cannot replay;
- only canonical `ProvenNotApplied` or idempotent `OutcomeUnknown` may become candidates.

The qualification subject binds the exact canonical history head and receipt, exact latest `{record, manifest}` entry, history-read trusted-time evidence, a newer qualification-time receipt, exact prior attempt subject, stable effect identity, and replay basis.

The qualifier may reject or defer. A `Qualified` result must supply an explicit bounded `evidence_valid_until` that cannot outlive the qualifier receipt, history-read evidence, qualification-time evidence, or verifier generation.

The resulting token is evidence qualification only. It contains no replay or execution authority.