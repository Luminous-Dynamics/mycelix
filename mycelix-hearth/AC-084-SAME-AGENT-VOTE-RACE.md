# AC-084 — Same-Agent Concurrent Vote Race

## Purpose

Characterize the duplicate-vote boundary when two `cast_vote` calls from the same Hearth cell and same agent execute concurrently.

The test is intentionally stronger than a sequential duplicate check: both calls use the same `SweetConductor`, same cell, same source chain, and are awaited with `tokio::join!`.

## Acceptance property

At most one concurrent call may successfully create an accepted Vote for the Decision.

After the race settles:

- `get_decision_votes` must expose at most one current Vote.
- `get_vote_history` must contain at most one accepted Vote from the race.
- When both speculative calls lose before a source-chain position is committed, a sequential recovery vote proves the cell remains usable.

Both call results are retained for diagnosis rather than collapsed into a boolean pass/fail.

## Platform boundary

Current Holochain source-chain semantics document that concurrent zome calls may observe the same execution-start snapshot, while simultaneous writes to one agent source chain cannot both commit: the first completed write advances the chain top and the competing write fails unless the caller retries.

This test therefore qualifies a concrete same-agent source-chain race. It does **not** establish:

- DHT-wide vote completeness;
- distributed exactly-once finalization;
- absence of malicious source-chain forks;
- real-world identity;
- substantive legitimacy of a decision rule.

Those require separate evidence layers.

## Qualification topology

The current dedicated Hearth exact-head workflow is introduced by AC-079. This PR is deliberately based directly on `main` and does not duplicate that workflow, so this PR remains a single-purpose test change.

Once the exact-head qualifier workflow is present on `main`, this test should run under that exact-head boundary rather than relying on generic CI admission.

## References

- Holochain source-chain concurrency: https://developer.holochain.org/concepts/3_source_chain/
- Holochain deterministic validation: https://developer.holochain.org/build/validation/
- Holochain deterministic `must_get_*` dependencies: https://developer.holochain.org/build/must-get-host-functions/
