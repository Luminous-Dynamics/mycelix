# SSF Canonical Effect Outcome History v0.1

Persists the richer canonical post-effect evidence envelope in an append-only history keyed by the exact durable invocation lineage.

Historical evidence may outlive the authority that produced it. Recording and head-read operations therefore require fresh store/time evidence, while previously qualified effect evidence is preserved as historical fact rather than rejected merely because its old authority lifetime has expired.

Terminality is derived only from the canonical qualified disposition. A raw provider `Confirmed` report paired with canonical `OutcomeUnknown` remains non-terminal for replay purposes, while the original provider report remains embedded verbatim in the evidence envelope.

The history is append-only. Canonical `OutcomeUnknown` may later refine to a terminal canonical result, but `Confirmed` and `ProvenNotApplied` heads are terminal and cannot be overwritten.

Head reads return an explicit latest `{record, manifest}` pair from the same concrete store and validate that the record equals the durable head plus exact predecessor/generation/terminal consistency. This avoids the sibling-manifest ambiguity of a loose head/manifest API.

This crate creates no replay or execution authority.