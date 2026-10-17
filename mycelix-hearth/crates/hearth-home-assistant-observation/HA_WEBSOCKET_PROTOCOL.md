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

Home Assistant's compact stream has separate added, changed and removed entity fields. The transport must merge these according to the provider protocol, reject malformed/oversized payloads, and never treat the arbitrary attribute bag as household authority. The HA-0.1 mapper remains the policy boundary for exact allowlisted entities, state codecs, metadata drift, and local-receive-time freshness.

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

## Contract boundary

The protocol action set is closed. There is no `call_service`, `set_states`, arbitrary-command, registry mutation, or physical-effect constructor. The future transport must still implement bounded JSON/message decoding, explicit TLS/certificate policy, local secret-provider integration, deadlines, bounded reconnect backoff, safe error logging, and serialized handling of each session's received frames.

This state machine does not authenticate the remote peer by itself, establish household authority, prove physical device state, or qualify an effect/provider. Physical control still belongs behind the independent Hearth adapter admission, exclusive executor lease, durable journal, current household authority, and outcome reconciliation path.

## Test plan

The pure unit tests cover authentication/order, exact sorted profile allowlisting, invalid profile rejection, result correlation and shape, initial snapshot ordering, explicit snapshot-apply acknowledgement, subscription/session identity mismatch, stale epoch handling, reconnect behavior, terminal states, ID exhaustion, and credential-free protocol actions.

Run from the repository root:

```sh
cargo test --manifest-path mycelix-hearth/Cargo.toml -p hearth-home-assistant-observation
```

This command has not been executed in this work session; no compilation or test pass is claimed.
