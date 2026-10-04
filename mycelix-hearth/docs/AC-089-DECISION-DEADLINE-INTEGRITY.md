# AC-089: Decision deadline integrity qualification

## Scope

AC-089 binds a newly created Decision's immutable deadline to the authoritative timestamp of its Holochain Create action.

## Required invariant

`Decision.deadline >= Create action timestamp`.

This prevents a structurally impossible Decision whose voting deadline has already elapsed at the moment of publication.

AC-089 does not require `Decision.created_at == action.timestamp`. The entry field remains non-authoritative, consistent with the existing timestamp-provenance boundary.

## Determinism

The validator compares immutable entry data against the timestamp carried by the validated Create action. It does not inspect current wall-clock time or mutable DHT collections.

## Test coverage

Pure tests cover equality, future deadlines, and deadlines before the Create action timestamp.

## Qualification boundary

This qualifies Decision temporal structure only. It does not establish voting eligibility, membership legitimacy, lifecycle authority, outcome completeness, or governance legitimacy.