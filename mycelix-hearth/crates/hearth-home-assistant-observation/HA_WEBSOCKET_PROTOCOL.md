# Home Assistant WebSocket protocol state machine (HA-0.2A)

**Scope:** pure Rust protocol contract only. No socket, URL, TLS, token, filesystem, transport dependency, state write, service call, or device-control API is included.

This stage freezes the sequence required before a native read-only transport exists:

```text
fresh session epoch
  -> auth_required
  -> transport-owned authentication request
  -> auth_ok
  -> get_states (correlated result must be a state array)
  -> subscribe_events(event_type = state_changed)
  -> correlated success with null result
  -> LIVE
  -> state_changed frames may reach the HA-0.1 mapper
```

## Invariants

- A session starts in `AwaitAuthRequired`; messages out of order fail closed.
- Authentication failure is terminal.
- Bootstrap and subscription results must match the exact outstanding correlation ID and the expected result shape.
- State-change events before subscription acknowledgement are rejected.
- A live unrelated event is ignored; only the typed `StateChanged` input can be surfaced for mapping.
- Each connection has an owner-assigned session epoch. Frames tagged with an older epoch are ignored without advancing the active session.
- Reconnection means a new machine and a fresh auth/bootstrap/subscribe cycle. Old command IDs must never be routed into the new epoch.
- Command-ID exhaustion fails closed.
- Authentication secrets are not represented in this module. The eventual transport must obtain them from a local secret provider and must not log them.

## Contract boundary

`ProtocolAction` is deliberately a closed set, not a general JSON command builder. It can request authentication, state bootstrap, and `state_changed` subscription; there is no `call_service`, `set_states`, arbitrary command, or arbitrary event API. The future transport must still enforce URL/TLS policy, token minimization, frame-size and JSON-depth bounds, deadlines, reconnect jitter, and no-secret diagnostics.

The existing HA-0.1 `map_state` remains responsible for exact entity allowlisting, deterministic codecs, metadata drift, and local-receive-time freshness. This state machine establishes only protocol ordering/correlation; it does not authenticate the peer, prove device state, establish household authority, or qualify a provider.

## Test plan

The pure unit tests cover happy-path ordering, auth rejection, out-of-order messages, bootstrap payload shape, both command correlation IDs, subscription acknowledgement shape, premature state events, stale reconnect epochs, old-session replay, terminal states, ID exhaustion, and the absence of secret-bearing fields in the protocol action model.

To run from the repository root: `cargo test --manifest-path mycelix-hearth/Cargo.toml -p hearth-home-assistant-observation`. This command has not been executed in this work session.
