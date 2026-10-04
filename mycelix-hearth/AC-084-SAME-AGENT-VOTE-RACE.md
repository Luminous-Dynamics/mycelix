# AC-084 — Same-Agent Concurrent Vote Race

## Purpose

Characterize the duplicate-vote boundary when two `cast_vote` calls from the same Hearth cell and same agent execute concurrently.

The test deliberately uses the same `SweetConductor`, the same cell, the same source chain, and `tokio::join!`.

## Acceptance property

At most one concurrent call may successfully create an accepted Vote for the Decision.

After the race settles:

- `get_decision_votes` exposes at most one current Vote;
- `get_vote_history` contains at most one accepted Vote from the race;
- when both speculative calls fail before a source-chain position is committed, a sequential recovery vote proves the cell remains usable.

Both call results are retained for diagnosis rather than collapsed into a boolean.

## Platform boundary

Current Holochain source-chain semantics describe concurrent zome calls sharing an execution-start snapshot while simultaneous writes to one agent source chain cannot both commit; a competing write can fail when the chain top has advanced.

This qualifies a concrete same-agent source-chain race. It does not establish DHT-wide vote completeness, distributed exactly-once finalization, absence of malicious source-chain forks, real-world identity, or substantive governance legitimacy.

## Qualification topology

AC-084 is a single-purpose characterization test. The exact-head qualification infrastructure is separately promoted by AC-088.

## References

- Holochain source-chain concurrency: https://developer.holochain.org/concepts/3_source_chain/
- Holochain deterministic validation: https://developer.holochain.org/build/validation/
- Holochain deterministic `must_get_*` dependencies: https://developer.holochain.org/build/must-get-host-functions/