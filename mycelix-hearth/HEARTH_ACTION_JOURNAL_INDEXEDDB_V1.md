# Hearth Action Journal — IndexedDB Adapter V1

**Status:** browser persistence adapter authored; no native/WASM compilation, browser integration, reload, or dispatch-gate qualification is claimed.
**Related work:** HTH-UX-005 / [issue #5060](https://github.com/Luminous-Dynamics/mycelix/issues/5060), building on the state model and journal-before-dispatch contract in [PR #5059](https://github.com/Luminous-Dynamics/mycelix/pull/5059) and [PR #5068](https://github.com/Luminous-Dynamics/mycelix/pull/5068).

## Paradigm shift

A button callback and a toast are ephemeral UI events. Hearth should track a minimally described operation across browser restarts, while preserving the difference between local intent, a request that may have been sent, and evidence of an outcome.

The journal answers: **what did this browser durably record about the attempt?** It does not independently answer whether a Holochain mutation committed or whether peers have current data.

## Implemented in this tranche

- A native `web-sys` IndexedDB adapter, without adding a third-party IndexedDB crate.
- A versioned `JournalEntryV1` with an allowlisted action enum, opaque operation ID, selected-Hearth scope fingerprint, target fingerprint, explicit lifecycle status, and monotonic timestamps.
- Unknown schema versions and unknown record fields fail closed.
- Scope/target fingerprints must be lowercase `sha256:` plus 64 hex characters. `sha256_fingerprint` uses Web Crypto SHA-256 with a length-delimited, domain-separated preimage. `new_operation_id` uses the browser cryptographic `randomUUID` API and fails closed if unavailable.
- Read/write changes return only after the IndexedDB request and the transaction-completion event succeed.
- Transition reads, checks the legal lifecycle transition, and queues the replacement write in one read/write transaction; it does not do a stale read in one transaction then overwrite in another.
- `StoragePosture` exposes whether a strict transaction option was observed and whether persistent origin storage was granted (or is unknown).
- Restart recovery changes every recovered `Submitted` row to `OutcomeUnknown`, persists that transition, and only then returns the current list. It never replays an action.
- A deterministic in-memory fake supports commit-failure and recovery tests.

## Live dispatch integration (HTH-UX-011)

The follow-on branch/PR wires the durable gate into all five currently live UI mutation handlers: bond tending, gratitude, voting, presence, and care completion.

- Context startup opens IndexedDB and finishes restart recovery before exposing a journal handle to live handlers.
- Each handler first validates the selected Hearth scope and action target, computes domain-separated scope/target fingerprints, writes Prepared, then writes Submitted, and only sends the zome call if both transaction-completion results succeeded.
- If storage is unavailable or either required write reports an error, the handler shows a Not sent message and returns before the network call.
- On a decoded successful response, the resident receives saved feedback immediately and the journal then records SourceChainCommitted. If this final local write fails, the UI warns that the saved result was received but local recovery state is not confirmed; users are told not to repeat the action until reconciled.
- On transport/call error, the UI reports OutcomeUnknown and the journal attempts to persist that state. Persistence failure is separately surfaced and never makes the action safe to retry.
- The local operation ID is not sent to Holochain and is not represented as a backend idempotency key.

This narrows accidental client-side duplicate risk but is still not exactly-once remote execution. If the browser loses all local storage after dispatch, the client journal cannot recover its own history; domain reconciliation and a separately qualified backend idempotency contract remain necessary.

## Dispatch protocol required before production integration

The adapter exists, but current Hearth live action handlers are **not yet routed through it**. Until that follow-on integration is complete, do not claim that actual UI dispatches are journal-gated or that blind replay has been eliminated end-to-end.

The required sequence is:

1. Generate an operation ID with cryptographic randomness and construct only allowlisted metadata.
2. Insert `Prepared` and await transaction completion.
3. Transition to `Submitted` and await transaction completion.
4. Only then send the request through the live action handler.
5. Persist `SourceChainCommitted` only after a successful, decodable domain response. Record `OutcomeUnknown` for transport, timeout, decode, or restart ambiguity. Use `ExplicitlyRejected` only with a typed contract proving the effect did not apply.
6. On reload, persist the conversion of each `Submitted` row to `OutcomeUnknown` before showing the recovered history.
7. Never automatically resend `OutcomeUnknown`. Resolve it only through a future authoritative, correctly scoped reconciliation read or a server-side idempotency contract.

A browser-local operation ID is not a backend deduplication key unless the zome explicitly implements and qualifies that contract. This adapter does not deliver exactly-once effects.

## Durability and browser-storage caveat

The adapter requests an IndexedDB transaction with `durability: "strict"` and checks the transaction's reported setting before admitting writes. The write method waits for transaction completion rather than treating an individual request's success as commit success.

This is still bounded local evidence, not a promise against every power-loss/storage-controller failure. Browser storage may be removed by the user, cleared in private browsing, limited by quota, or evicted. The Storage API's `persisted()` result is reported as posture only; a grant does not prevent user deletion or device loss. No claim is made that every browser implements strict durability identically.

References:
- [MDN: IDBDatabase.transaction() and transaction durability](https://developer.mozilla.org/en-US/docs/Web/API/IDBDatabase/transaction)
- [MDN: StorageManager.persist()](https://developer.mozilla.org/en-US/docs/Web/API/StorageManager/persist)
- [MDN: IndexedDB API — basic terminology and storage lifecycle](https://developer.mozilla.org/en-US/docs/Web/API/IndexedDB_API/Basic_Terminology)
- [MDN: SubtleCrypto.digest()](https://developer.mozilla.org/en-US/docs/Web/API/SubtleCrypto/digest)
- [MDN: Crypto.randomUUID()](https://developer.mozilla.org/en-US/docs/Web/API/Crypto/randomUUID)

## Privacy boundary

The journal intentionally excludes free-text message bodies, household/member display names, raw agent keys, and arbitrary payload serialization. The caller must supply canonical identity bytes to the domain-separated fingerprint helper; it must not feed human names or message text into the journal.

A plain hash is **not encryption or anonymization**. Fingerprints can be linkable or guessable if input material is public or low entropy. Use them only as minimized local locators, never as authorization credentials. Selected-Hearth fingerprints are part of the record so recovery can detect scope changes rather than silently attributing an old attempt to the current household.

## Authored test coverage

Current module tests cover:
- strict record round-trip and allowlisted metadata;
- invalid/raw/noncanonical fingerprint rejection and digest formatting;
- unknown schema version and unknown fields failing closed;
- legal transition, timestamp monotonicity and terminal status preservation;
- injected Prepared persistence failure leaving no journal row;
- restart recovery converting Submitted to OutcomeUnknown before returning a row.

These tests are authored, **not executed**. They do not substitute for a real IndexedDB browser test, multiple-tab race test, reload test, or native/WASM compile.

## Benchmarks (targets, not results)

1. **Zero dispatch before journal gate:** 100% of live sends occur only after both Prepared and Submitted transaction-completion events.
2. **Fail-closed persistence:** zero sends when IndexedDB is unavailable, strict durability is unsupported, or either required journal commit fails.
3. **Restart recovery:** 100% of recovered Submitted rows become durably OutcomeUnknown before presentation; zero automatic replay of unknown attempts.
4. **Atomic state:** zero illegal state transitions accepted across same-tab or multi-tab tests.
5. **Privacy:** zero message bodies, display names, raw agent keys, or arbitrary input payloads in serialized journal records.
6. **Crash matrix:** 10,000 deterministic injected crash/reload schedules with zero client-side blind replay, including before Prepared, between Prepared/Submitted, after Submitted/before send, in flight, after remote commit/before response, and after response/before confirmation commit.
7. **Browser qualification:** test Chromium, Firefox, and WebKit profiles where available; record storage mode, transaction-option support, quota failure, browser restart/reload, and multiple-tab conflict results. Unsupported required storage semantics must produce a visible not-sent outcome rather than silently downgrade.
8. **Human factors:** 100% of recovered Submitted attempts labeled as unknown; at least 90% of participants understand that “unknown” is neither proof of failure nor proof of success.

## Nonclaims

This tranche does not wire the adapter into live action dispatch, prove browser storage cannot be evicted, guarantee remote deduplication, establish globally unique operation IDs across devices/agents, provide exactly-once execution, reconcile an unknown write, or prove DHT propagation/global synchronization. Those remain separate engineering and qualification gates.
