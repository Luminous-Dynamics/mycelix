# AC-089: Decision deadline integrity qualification

## Scope

AC-089 binds a newly created Decision's immutable deadline to the timestamp carried by its Holochain Create action.

## Required invariant

`Decision.deadline >= Create action timestamp`.

This prevents the Decision from declaring a deadline earlier than its own claimed Create action timestamp.

AC-089 does not require `Decision.created_at == action.timestamp`. The entry field remains non-authoritative, consistent with the AC-076 timestamp-provenance boundary.

## Determinism and timing boundary

The validator compares immutable deadline data against the validated Create action timestamp. It does not inspect current wall-clock time or mutable DHT collections.

Holochain documents action timestamps as self-reported, so this establishes internal ordering of declared action timestamps, not trusted external wall-clock time.

## Test coverage

Pure tests cover equality, later deadlines, and deadlines earlier than the Create action timestamp.

## Qualification boundary

This qualifies Decision temporal structure only. It does not establish trusted clock truth, voting eligibility, membership legitimacy, lifecycle authority, outcome completeness, or governance legitimacy.