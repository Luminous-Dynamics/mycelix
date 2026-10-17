# Home Assistant WebSocket protocol state machine (HA-0.2A)

**Scope:** pure Rust protocol contract only. No socket, URL, TLS, token, filesystem, transport dependency, state write, service call, or device-control API is included.

Home Assistant documents the WebSocket auth/result/event protocol and correlation behavior in its [official WebSocket API reference](https://developers.home-assistant.io/docs/api/websocket/). One important design constraint is that an event carries the `id` of its subscription, and `get_states` plus `subscribe_events` are separate messages—not an atomic snapshot-plus-stream operation.

## Snapshot/stream consistency protocol

This state machine deliberately subscribes **before** requesting the initial state snapshot so there is no intentional unobserved interval between the snapshot and the event subscription:

```text
new session epoch
  -> auth_required
  -> transport-owned authentication request
  -> auth_ok
  -> subscribe_events(event_type = state_changed), id = 1
  -> matching result(success = true, result = null)
  -> get_states, id = 2
  -> matching result(success = true, result = state array)
  -> apply initial snapshot
  -> drain buffered state_changed events in receive order
  -> bootstrap_applied(epoch, subscription_id)
  -> LIVE
```

While `get_states` is in flight, correctly correlated `state_changed` events are buffered by the eventual transport. They must not be applied before the snapshot, because doing so could let the older snapshot overwrite a newer event. When the snapshot arrives, the transport applies it first, drains buffered events in original receive order, and calls `bootstrap_applied`. The state machine does not enter `Live` until that explicit acknowledgement succeeds.

## Invariants

- A session starts in `AwaitAuthRequired`; messages out of order fail closed.
- Authentication failure is terminal.
- Subscription and bootstrap results must match their exact outstanding correlation IDs and expected payload shapes.
- Every state-change event must carry the exact active subscription ID.
- State-change events before subscription acknowledgement are rejected; events after acknowledgement but before bootstrap completion are buffered.
- Bootstrap success returns ordered `BootstrapReady` then `DrainBufferedStateChanges` actions. The transport must apply the snapshot before replaying buffered events and must call `bootstrap_applied` only after the drain completes.
- A live unrelated event is ignored; only the typed `StateChanged` input for the active subscription can be surfaced for mapping.
- Each connection has an owner-assigned session epoch. Frames tagged with an older epoch are ignored without advancing the active session.
- Reconnection means a new machine and a fresh auth/subscription/bootstrap cycle. Old command IDs must never be routed into the new epoch.
- Command-ID exhaustion fails closed.
- Authentication secrets are not represented in this module. The eventual transport must obtain them from a local secret provider and must not log them.

## Contract boundary

`ProtocolAction` is deliberately a closed set, not a general JSON command builder. It can request authentication, state bootstrap, and the `state_changed` subscription; there is no `call_service`, `set_states`, arbitrary command, or arbitrary event API. The future transport must still enforce URL/TLS policy, token minimization, frame-size and JSON-depth bounds, deadlines, reconnect jitter, and no-secret diagnostics.

The existing HA-0.1 `map_state` remains responsible for exact entity allowlisting, deterministic codecs, metadata drift, and local-receive-time freshness. This state machine establishes protocol ordering/correlation only; it does not authenticate the peer, prove device state, establish household authority, or qualify a provider.

## Test plan

The pure unit tests cover happy-path ordering, auth rejection, out-of-order messages, bootstrap payload shape, both command correlation IDs, subscription ID binding, event buffering and drain ordering, explicit bootstrap completion, stale reconnect epochs, old-session replay, terminal states, ID exhaustion, and the absence of secret-bearing fields in the protocol action model.

To run from the repository root: `cargo test --manifest-path mycelix-hearth/Cargo.toml -p hearth-home-assistant-observation`. This command has not been executed in this work session.
