# Live data lifecycle

Praxis treats the selected data mode, conductor transport, and zome-call
authorization as separate state. Selecting Live mode is not evidence that a
resource is safe to request.

## Resource state model

Every network-backed `LocalResource` reads `tracked_data_source` in its source
closure, before creating its future. The resulting state has these meanings:

| Source | Resource behavior | UI claim |
| --- | --- | --- |
| Demo | Return deterministic representative examples | Demo banner identifies them as examples |
| Local | Return browser-owned records or an honest local empty state | No network record claim |
| Live, not ready | Return `WaitingForLive` without issuing a zome call | Waiting for conductor and signer |
| Live, ready | Issue the zome call | Render `Ready`, including a valid empty result, or `LiveError` |

`WaitingForLive`, `LiveError`, and `Ready(empty)` are intentionally distinct.
A disconnect or missing signer must never be presented as an empty network
collection, and a Live failure must never substitute local learner data.

Because `tracked_data_source` reads the shared provider signals reactively,
resources created during initial connection run again when signing becomes
ready. They also leave the ready state during disconnect/reconnect and retry
after the shared provider returns to ready.

## Mutation rule

Event handlers use the non-reactive `zome_calls_ready_untracked` snapshot.
Live mutations are blocked until both transport and authorized signing are
ready. Drafts and disclosure selections may still be persisted locally, but
the UI must say that no graph mutation was sent.

The application owns one provider around the router. Pages must consume that
context and must not mount nested providers, which would split lifecycle and
error state.

## Qualification

Run the dependency-free structural gate with:

```sh
python3 scripts/validate_learning_contract.py
python3 scripts/validate_routed_claims.py
```

This gate detects lifecycle-contract drift in source. It does not replace a
browser build or an authorized real-conductor test that exercises disconnect,
reconnect, signer loss, and successful retry.
