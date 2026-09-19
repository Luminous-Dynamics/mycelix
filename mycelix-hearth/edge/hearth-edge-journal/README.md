# hearth-edge-journal

Crash-safe local execution journaling for Mycelix Hearth.

HTH-AUTO-004A establishes one rule:

> Restart must never transform uncertainty into permission to repeat a side effect.

The crate wraps an existing `hearth-edge::EdgeAdapter`. It does not add Matter, Home Assistant, MQTT, OCPP, OpenADR, networking, device discovery, or Holochain access.

## Write ordering

For a fresh logical step the wrapper persists:

```text
Prepared
   |
   v
Dispatching     <-- durable before adapter.execute(...)
   |
   +-- Rejected / AdapterUnavailable / NotAttempted
   |       -> DefinitelyNotExecuted
   |
   +-- Timeout / TransportError
   |       -> Ambiguous
   |
   +-- Accepted
           -> Accepted
                |
                v
             verify(...)
                |
                v
             Verified
```

`Dispatching`, `Ambiguous`, and `Accepted` all recover as **reconcile-only**. The wrapper does not automatically invoke the inner adapter again. It routes the existing Hearth edge runtime into verification using the same idempotency key.

`Prepared` and `DefinitelyNotExecuted` are the only states from which automatic dispatch is allowed.

## Command certainty contract

Adapters wrapped by this crate must preserve these meanings:

- `NotAttempted`, `Rejected`, `AdapterUnavailable`: the side effect definitely did not occur.
- `TimedOut`, `TransportError`: execution is ambiguous.
- `Accepted`: the command was accepted but outcome remains unverified.

A driver that cannot prove rejection/non-availability happened before execution must return `TransportError` or `TimedOut`, not `Rejected`.

## Durable journal

`FileJournal` uses one JSON record per idempotency key.

On Unix/NixOS each update:

1. writes a new temporary file,
2. calls `sync_all()` on the file,
3. atomically renames it over the prior generation,
4. calls `sync_all()` on the parent directory.

`JournaledAdapter::new_physical` refuses a journal that does not report `JournalDurability::Durable`. `MemoryJournal` is intentionally volatile and test-only.

This tranche assumes a **single writer per journal root**. Process exclusivity should be enforced by the future NixOS service wrapper before real adapters are admitted.

## Crash windows

| Last durable phase | What may have happened? | Automatic restart behavior |
|---|---|---|
| no record | nothing | prepare normally |
| `Prepared` | no command dispatched | safe to dispatch after fresh policy evaluation |
| `Dispatching` | command may have reached device | verify only; never blindly resend |
| `DefinitelyNotExecuted` | adapter contract proves no side effect | safe to dispatch again |
| `Ambiguous` | side effect may have occurred | verify only |
| `Accepted` | side effect may have occurred | verify only |
| `Verified` | requested outcome was observed | reuse verified result |

The existing `hearth-edge` authority checks still run before every logical attempt. The journal does not grant authority; it constrains what execution may happen after uncertainty.

## HTH-AUTO-004A limits

This PR deliberately does **not** yet provide:

- multi-process writer leasing,
- persistent scheduler queues,
- durable household telemetry,
- cross-device transactions,
- DHT receipt publication,
- historical simulation/replay,
- automatic recovery of irreversible ambiguous actions,
- physical device adapters.

Those belong to later qualification tranches.