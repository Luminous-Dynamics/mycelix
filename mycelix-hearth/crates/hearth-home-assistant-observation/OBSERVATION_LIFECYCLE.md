# Explicit Home Assistant observation lifecycle

**Scope:** pure lifecycle mapping contract. This is not a database writer, transport, authority system, or device-control path.

## Why this layer exists

Typed decoding is necessary but not sufficient. Once an entity has emitted a valid observation, a later source state of `unknown` or `unavailable`, metadata drift, invalid current value, provider removal, or omission from a reconnect snapshot must not leave the old positive value silently current.

The lifecycle mapper turns each accepted compact projection into a sorted `ObservationBatch`. Every affected entity receives at most one operation. A projection can fail before there is an `EntityStreamProjection` to map; in that case, `map_projector_failure` emits a conservative `ProjectionRejected` invalidation for every entity in the current valid profile, because a multi-entity rejected delta may have changed more entities than the first error names. The transport must make the stream non-live (close the session) and require a fresh complete snapshot before resuming.

- `Upsert(HomeAssistantObservation)` for a current profile-valid sample.
- `Invalidate { entity_id, reason, at_micros }` when the system must stop treating a prior sample as current.

Contradictory operations for the same entity reject the entire batch. A frame outside the active profile's entity allowlist also rejects the whole batch rather than emitting an operation for an unapproved entity.

## Durable ordering and replay rule

Every batch carries `ObservationVersion { generation, sequence }`, compared lexicographically.

1. `generation` is a durable, monotonically increasing counter owned by the local transport/session coordinator. It must increase across reconnects and process restarts; do not reset it to a fixed constant after reboot.
2. The initial snapshot in a generation has sequence `0`.
3. Each accepted incremental provider event advances `sequence` monotonically before lifecycle mapping. A delta with sequence zero is invalid.
4. A consumer stores the highest applied version per entity and applies a batch operation only when its version is **strictly newer** than that entity's stored version. Equal means duplicate replay; lower means stale replay.
5. The store applies all operations in one batch atomically and stores the corresponding per-entity version in the same transaction as the upsert/invalidation.

This protects against a subtle distributed-systems failure: an invalidation may arrive and commit before a delayed older upsert is processed. The old upsert must not resurrect the stale positive value. The pure mapper validates the version's shape and supplies a comparison helper; it does not allocate or persist the durable generation counter and cannot guarantee correct consumption by itself.

## Lifecycle mapping rules

| Input condition | Output |
|---|---|
| Profile-valid decoded observation | Upsert |
| Source state `unknown` | Invalidate: `SourceUnknown` |
| Source state `unavailable` | Invalidate: `SourceUnavailable` |
| Current value fails codec/range/metadata/freshness validation after projection | Invalidate: `InvalidCurrentSample` |
| Transactional projector rejects a current provider event before producing a projection | Invalidate every entity in the active valid profile: `ProjectionRejected`; close stream and require a fresh complete snapshot |
| Allowlisted entity removed by provider | Invalidate: `EntityRemoved` |
| Allowlisted entity absent from an initial/reconnect snapshot | Invalidate: `MissingFromInitialSnapshot` |
| Frame outside the active allowlist | Reject entire batch |
| Duplicate or contradictory per-entity operations | Reject entire batch |
| Invalid profile/version | Reject entire batch |

An invalidation means **“do not rely on the previous observation as current.”** It does not mean the physical device is off, absent, safe, or in any other inferred state.

## Proposed conformance cases

The test suite includes positive upsert mapping, unknown/unavailable invalidation, range and metadata-drift invalidation, entity-removal invalidation, reconnect-snapshot omission, unallowlisted input rejection, contradictory update rejection, invalid-version rejection, and transactional projector-failure invalidation across the full allowlist with deterministic ordering. The version ordering helper tests that newer generations/sequences advance and equal or older versions are rejected.

The proposed HAOBS-011/012/013 benchmarks remain unmeasured. A runtime/database adapter must still show atomic persistence and stale replay rejection, and the executable harness must measure delta latency versus cache size, before those properties can be claimed end-to-end.
