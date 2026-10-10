# AC-071: Deterministic Concurrent Outcome Conflict Handling

## Purpose

Hearth decision finalization is a multi-step distributed operation. Multiple agents may observe the same open decision and independently attempt resolution.

The application must therefore define a conflict rule rather than assuming a global database-style lock.

## Canonical rule

When more than one `DecisionOutcome` candidate is linked to the same decision:

1. preserve every candidate in the DHT;
2. validate that each candidate references the requested decision;
3. choose the candidate whose creation action has the earliest timestamp;
4. when timestamps are identical, choose the lexicographically smallest raw action hash;
5. never use DHT link iteration order as a semantic rule.

Formally:

```
min(action_timestamp, action_hash)
```

defines the canonical read candidate.

## Why candidates remain

Discarding losing candidates would destroy evidence about concurrent execution.

The system therefore distinguishes:

```
candidate history != canonical outcome view
```

The canonical view is deterministic while the underlying history remains auditable.

## Limits

This does not claim:

- exactly-once distributed finalization;
- a universal global lock;
- prevention of concurrent writes;
- that the earliest candidate is morally or politically superior.

It is only a deterministic application-level conflict-resolution rule.

## Relationship to earlier controls

AC-068 binds declared actor identity to the Holochain action author.

AC-069 prevents an empty or zero-weight tally from becoming a substantive result.

AC-070 prevents closure from creating a substantive outcome.

AC-071 addresses the remaining distributed conflict case when multiple valid resolution candidates exist.

## Qualification boundary

Validation establishes deterministic selection of the canonical candidate under the tested concurrent-outcome model. It does not establish exactly-once execution across arbitrary deployments.
