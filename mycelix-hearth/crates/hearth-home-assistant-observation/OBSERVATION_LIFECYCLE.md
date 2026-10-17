# Home Assistant observation lifecycle and invalidation (HA-0.2C)

**Status: pure in-memory lifecycle contract; no disk durability or native transport qualification is claimed.**

This layer sits above the HA-0.2A protocol gate, strict JSON decoder, compact entity projector, and HA-0.1 semantic mapper. It answers a separate question:

> Given a positive typed observation that was valid earlier, is it still current *now*?

A projector cache entry is not sufficient proof of currentness. The lifecycle layer emits explicit revision-bound invalidations and prevents old positives from remaining usable across source uncertainty, disconnects, expiry, malformed input, or reconnect.

## Currentness theorem

```text
current positive observation
= exact allowlisted semantic observation
+ exact active session epoch
+ admitted freshness deadline not reached
+ protocol subscription acknowledged
+ initial snapshot applied and acknowledged
+ lifecycle phase == Live
```

The absence of a current positive means **unknown/not currently established**, not "device is off", "device is absent", or "device is safe".

## Required transitions

- `unknown` / `unavailable`: invalidate an active positive for that entity.
- `removed`: invalidate an active positive; removal is not physical-state evidence.
- Freshness deadline reached: invalidate at `now >= valid_until_micros`.
- Transport disconnect: invalidate active positives and close the session.
- New session: synchronously emit exactly one `SessionReplaced` invalidation for every prior active positive before authentication/subscription begins. Old positives cannot be read as current during reconnect.
- Reconnect snapshot: an omitted entity stays non-current and emits no second invalidation. A present entity may become current only after the replacement snapshot is applied and acknowledged; `unknown`/`unavailable` remains non-current.
- Malformed JSON, invalid allowlisted state, codec/range failure, profile metadata drift, unexpected event ordering, or mapping rejection: reject the whole stream session, invalidate active positives, and require a fresh session/bootstrap. Projector rollback alone is insufficient because the old positive may already have been consumed downstream.
- Duplicate/lower session epochs: no state mutation and no resurrection.
- Duplicate invalidation: no second invalidation record when no active positive revision remains.
- Semantically equivalent state replay: no new positive revision and no freshness extension.

## Protocol handshake

The lifecycle layer intentionally duplicates critical binding checks rather than trusting arbitrary caller ordering:

```text
start_session(epoch)
-> bind_subscription(epoch, subscription_id)
-> subscription_acknowledged(epoch, subscription_id)
-> initial snapshot projected
-> snapshot_applied(epoch, subscription_id)
-> Live
```

The transport still must obtain each transition from the actual Home Assistant protocol state machine. This API does not authenticate the server or independently prove the caller's frame came from the socket.

## Revision and privacy model

Every entity has a monotonic local revision counter. A positive record identifies its new revision and any positive revision it supersedes. An invalidation record cites the exact prior positive revision it invalidates. Tombstone/high-water revisions remain after invalidation to prevent old local replay from reusing an earlier revision.

The lifecycle record contains only the allowlisted entity ID, local session epoch, lifecycle revision, local monotonic time, transition type, and bounded reason enum. It contains no raw state, arbitrary attributes, friendly names, token, URL, or credential. Do not publish unsalted hashes of low-entropy household states; they can reveal values such as `on`/`off` by dictionary attack.

## Checkpoint / restart

`ObservationLifecycleCheckpointV1` exports the versioned profile reference, highest accepted session epoch, and per-entity revision/tombstone map. It deliberately exports no positive state. Restoring a checkpoint enters `Dormant`; a strictly newer session and a fresh snapshot are mandatory before any observation can become current.

The checkpoint is only a serializable state shape. The crate does not write/fsync it, atomically replace it, authenticate it, or protect it against rollback. To claim crash-replay resistance, a runtime must persist it with crash-safe ordering and independent integrity/rollback protection. Until then the replay guarantee is process-local only.

## Run

`cargo test --manifest-path mycelix-hearth/Cargo.toml -p hearth-home-assistant-observation`

The command has not been executed in this work session. See [lifecycle benchmark targets](OBSERVATION_LIFECYCLE_BENCHMARKS.md); every target is proposed/unmeasured.
