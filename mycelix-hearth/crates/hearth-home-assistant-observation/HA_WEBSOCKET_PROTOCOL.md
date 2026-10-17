# Home Assistant WebSocket protocol state machine (HA-0.2A)

**Scope:** pure Rust protocol contract only. No socket, URL, TLS, token, filesystem, transport dependency, state write, service call, or device-control API is included.

This design is grounded in the [official Home Assistant WebSocket API](https://developers.home-assistant.io/docs/api/websocket/). Prefer the native `subscribe_entities` command over composing `get_states` and `subscribe_events` yourself: Home Assistant documents the entity subscription as an initial compact state snapshot followed by incremental changes, and supports restricting it to an explicit `entity_ids` list. It also binds every stream event to the exact subscription command ID.

## Protocol sequence

```text
new session epoch
  -> auth_required
  -> transport-owned authentication request
  -> auth_ok
  -> subscribe_entities(entity_ids = exact validated profile allowlist), id = 1
  -> matching result(success = true, result = null)
  -> first event with id = 1 and initial entity snapshot
  -> decode only allowlisted entities and apply the snapshot
  -> initial_snapshot_applied(epoch, subscription_id)
  -> LIVE
  -> subsequent events with id = 1 are incremental changes
```

The protocol machine builds the subscription allowlist directly from the validated HA-0.1 profile. It sorts IDs deterministically, rejects invalid/empty profiles, and never asks the transport to subscribe to every entity. The transport must treat entity IDs in the compressed event maps as untrusted input and must pass only profile-allowlisted state data to the existing HA-0.1 mapper.

## Snapshot/live boundary

The first event must be the initial snapshot. The transport should use a single serialized receive loop: decode/apply the initial snapshot, acknowledge it with `initial_snapshot_applied`, then proceed to later events. The machine does not enter `Live` until that explicit acknowledgement succeeds. A second event before the acknowledgement fails closed rather than risking a delta being applied before its base snapshot.

Home Assistant's compact stream has separate added, changed and removed entity fields. The transport must merge these according to the provider protocol, reject malformed/oversized payloads, and never treat the arbitrary attribute bag as household authority. The projector enforces each allowlisted entity's admitted `unit_of_measurement` and `device_class` assertions before committing any snapshot or delta to its cache. Metadata drift therefore fails at the transaction boundary rather than allowing a rejected downstream observation to leave poisoned cache state. The projector also rejects state strings and projected metadata containing control characters, so line breaks cannot forge multiline semantic values or poison logs. The HA-0.1 mapper independently rechecks exact allowlisted entities, state codecs, metadata assertions, and local-receive-time freshness.

## Invariants

- Authentication failure and out-of-order messages are terminal.
- The subscription result must match the exact command ID and a successful null result.
- The initial snapshot event and all subsequent change events must carry the active subscription ID.
- Only a validated, non-empty HA-0.1 entity allowlist may be subscribed.
- No entity changes are delivered before the initial snapshot is applied.
- Frames from stale connection epochs are ignored without advancing the active session.
- Reconnection requires a new machine, fresh authentication, a new entity subscription, and a new initial snapshot.
- Command-ID exhaustion fails closed.
- Credentials are never represented in this module, action enum, error types or debug values.

## Compact stream projection

The pure [entity stream projector](src/entity_stream.rs) understands the documented compact event groups:

- `a`: full compressed states for added entities;
- `c`: partial changes, including added fields and removed attribute names;
- `r`: entity removals.

It maintains a bounded semantic cache only for exact HA-0.1 profile entity IDs. It retains primary state, unit of measurement, and device class; it does not persist friendly names, arbitrary attributes, context identifiers, or network data. Changes are applied to a staged copy and committed only if every allowlisted item is valid. A change for an allowlisted entity absent from the current snapshot fails closed. Removed entities invalidate prior evidence instead of emitting a positive state observation. A delta that changes only ignored metadata (for example a friendly name or context) updates no semantic fields and emits no new observation frame, so it cannot refresh the freshness window of an unchanged measurement.

The transport must validate subscription ID and session epoch using the protocol machine before calling the projector. It must also enforce a frame-size bound before JSON parsing and apply the resulting initial snapshot synchronously before processing later change frames.

## Contract boundary

The protocol action set is closed. There is no `call_service`, `set_states`, arbitrary-command, registry mutation, or physical-effect constructor. The future transport must verify that its pinned Home Assistant version supports `subscribe_entities` and fail closed if not; it must not silently substitute an unqualified `get_states` + `subscribe_events` path. The official [Home Assistant JS client](https://github.com/home-assistant/home-assistant-js-websocket/blob/master/lib/entities.ts) selects the native stream for server versions at or above 2022.4.0 and retains a legacy path for older versions. Any legacy path would require its own ordered bootstrap tests and conformance profile. The future transport must also implement bounded JSON/message decoding, explicit TLS/certificate policy, local secret-provider integration, deadlines, bounded reconnect backoff, safe error logging, and serialized handling of each session's received frames.

This state machine does not authenticate the remote peer by itself, establish household authority, prove physical device state, or qualify an effect/provider. Physical control still belongs behind the independent Hearth adapter admission, exclusive executor lease, durable journal, current household authority, and outcome reconciliation path.

## Test plan

The 20 protocol tests cover authentication/order, exact sorted profile allowlisting, invalid profile rejection, result correlation and shape, initial snapshot ordering, explicit snapshot-apply acknowledgement, subscription/session identity mismatch, stale epoch handling, reconnect behavior, terminal states, ID exhaustion, and credential-free protocol actions. The 12 projector tests cover compact snapshot/diff/remove handling, strict allowlist projection, transactional rejection, malformed metadata, control-character rejection, and ensuring non-semantic attribute-only updates do not refresh observation freshness. Two cross-module integration tests exercise the entire protocol-gate → compact-projector → HA-0.1 mapper path: one verifies exact initial/change observations and that metadata-only changes cannot refresh freshness; the other verifies a wrong subscription ID fails before data can reach the projector.

Run from the repository root:

```sh
cargo test --manifest-path mycelix-hearth/Cargo.toml -p hearth-home-assistant-observation
```

This command has not been executed in this work session; no compilation or test pass is claimed.
