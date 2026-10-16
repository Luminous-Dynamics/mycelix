# Pulse Durable Delivery Protocol: Outbox, Inbox, Fencing and Receipts

- **Status:** Design contract for follow-on implementation; not integrated or qualified
- **Scope:** Public `Luminous-Dynamics/mycelix` repository
- **Related implementation:** `crates/pulse-transport-core/src/delivery_state.rs`
- **Follow-on:** [Issue #5062 — durable outbox/inbox replay](https://github.com/Luminous-Dynamics/mycelix/issues/5062)
- **Related benchmarks:** [Transport benchmark gates](TRANSPORT_BENCHMARKS.md) and [routing research agenda](ROUTING_RESEARCH_AGENDA.md)

## 1. The claim we should make

Pulse should target **at-least-once transmission attempts with durable, idempotent logical acceptance**—not unconditional exactly-once network delivery. Packets can be lost, duplicated, reordered, or acknowledged at different layers; a sender can crash after transmitting but before persisting the result.

The useful user-visible guarantee is narrower and testable:

> For a given recipient and stable logical-message identity, repeated delivery attempts and replay of accepted evidence produce at most one logical inbox item; delivery is shown only after a receipt is verified against the exact expected context. Accepted unexpired messages are never silently discarded; they are delivered, explicitly expired/revoked, or surfaced as failed/unknown.

This guarantee applies only after durable storage and receiver integration are implemented and tested. The present delivery lifecycle is in-memory; it does not meet the crash-recovery part yet.

## 2. Identity and evidence boundaries

Keep the following identifiers distinct:

- **Logical message ID:** stable across retries, adapters and restart.
- **Recipient identity:** the exact destination principal/device or identity scope defined by Pulse's product semantics.
- **Envelope commitment:** cryptographic digest of the exact serialized end-to-end protected envelope and relevant immutable context. Reusing a message ID for different envelope content must be rejected, not treated as a valid duplicate.
- **Attempt ID:** unique for one send attempt; never substitutes for the logical message ID.
- **Outbox record version:** monotonically increasing compare-and-swap revision for one durable record.
- **Worker epoch/fencing token:** monotonically increasing authority generation held by the current dispatcher lease. A stale dispatcher may observe a late callback, but must not mutate state after a newer epoch takes ownership.
- **Evidence event ID:** stable identity for one evidence fact; replaying the same event is idempotent, while reusing an event ID with different content is a hard conflict.

IDs and commitment formats need domain separation, versioning and explicit length bounds. Never derive a dedupe key from plaintext. Do not assume distinct adapter names imply independent failure domains.

### Receipt binding

A delivery receipt must be authenticated by Pulse's trusted verifier and bind at least:

1. logical message ID;
2. intended recipient identity;
3. the exact immutable envelope commitment (or equally strong context), which the trusted verifier must authenticate from the receipt—not merely compare against caller-supplied metadata;
4. receipt protocol/version and the semantics of the acknowledgement;
5. authenticated recipient-receive time if the product wants to distinguish on-time from late receipt;
6. anti-replay material or a receipt/event ID scoped to the verifier's replay policy.

A transport ACK or relay receipt cannot fill this role. A local timestamp recording when the sender first *observed* the receipt is not proof of when the recipient received the message. Until the trusted verifier exposes authenticated receive-time semantics, conservatively reject a first-observed receipt at/after expiry rather than inventing on-time delivery.

## 3. Durable state and event protocol

Every state change is committed with an expected record version. A transaction must append the new evidence/event and update the derived state together. A conflicting version or conflicting reuse of an event/attempt ID fails closed.

Suggested durable outbox states:

- `Queued`: encrypted opaque envelope and policy snapshot have been committed.
- `Dispatching`: one attempt intent and current worker epoch are durably recorded before calling an adapter.
- `PossiblySent`: recovery cannot establish whether the adapter transmitted; keep the outcome ambiguous.
- `AwaitingReceipt`: a send/forwarding acknowledgement exists, but no recipient receipt has been verified.
- `RetryScheduled`: a retry time and remaining budget have been committed.
- `Delivered`: trusted recipient receipt accepted against exact immutable context.
- `Expired`, `Revoked`, `Failed`, `OutcomeUnknown`: explicit terminal or unresolved dispositions; none may be silently converted to success.

A minimal event vocabulary should represent queue acceptance, attempt start, adapter outcome, permission/policy revision, expiry, revocation, verified recipient receipt, and retention/deletion. Store opaque ciphertext and the minimum necessary metadata; never journal plaintext or raw secrets merely to ease replay.

### Required send ordering

1. In a storage transaction, check expiry, consent/policy generation, resource budget and current fencing epoch; reserve an attempt ID, increment the bounded attempt budget, and write `Dispatching`.
2. Commit the transaction.
3. Send the same opaque envelope/immutable commitment through the chosen adapter outside the database transaction. Never hold a storage transaction open across network I/O.
4. In a second transaction, append the authenticated adapter observation only if the record version and fencing epoch still match. If the outcome cannot be persisted or the process dies, recover as `PossiblySent` / `OutcomeUnknown`.
5. Retry with the same logical message ID and envelope commitment, but a new attempt ID and the same bounded budget. The receiver must deduplicate durably before exposing the logical message.
6. Mark `Delivered` only after verifying a recipient receipt; persist the verified receipt evidence (or sufficient immutable evidence to re-verify it), not just a Boolean.

There is no atomic transaction spanning local disk and arbitrary radio/network transmission. The design therefore must handle the “sent, then crashed before recording send result” window by retrying idempotently—not by pretending the send did not happen.

## 4. Receiver inbox and single logical presentation

The receiver needs a durable uniqueness constraint on the logical dedupe identity. In one transaction it must:

1. validate envelope framing and bounded input size;
2. run the existing end-to-end authentication/decryption and authorization pipeline;
3. check immutable identity/commitment against any existing inbox item;
4. if new, persist accepted content plus dedupe identity before exposing it to the UI;
5. if an exact duplicate, return/reissue receipt evidence without creating a second logical inbox item;
6. if the same identity arrives with a different commitment/context, reject and record a security-relevant conflict;
7. emit the recipient receipt only after the protocol's defined durable-acceptance point.

The UI should key rendering/read/unread state to the logical inbox identity, not transport path or attempt. Retention/deletion must bound dedupe storage without opening a replay window for still-valid messages. The expiry, retention and key-rotation policy must agree on the maximum replay horizon.

“Exactly once” cannot promise exactly-once arbitrary external side effects unless those effects participate in the same transaction or have their own idempotency contract. Pulse can make inbox acceptance and user-visible logical presentation idempotent; other actions must declare their own semantics.

## 5. Crash/restart fault matrix

Inject a crash immediately before and after every durable write and network boundary. Recovery should be deterministic and should never infer success from missing evidence.

| Fault point | Required recovery |
|---|---|
| Before initial outbox commit | No accepted-message promise; no adapter send occurred |
| After outbox commit, before attempt reservation | Resume eligible queued record |
| After attempt intent commit, before adapter call | Recover attempt as possibly started; retry only under the same bounded idempotency context |
| After adapter call, before outcome commit | `PossiblySent`; never assume “not sent”; retry with the same logical identity |
| After receiver inbox commit, before receipt send | Duplicate is recognized; reissue receipt without a second inbox item |
| After receipt send, before sender persists it | Receipt may be replayed; verify it again and commit idempotently |
| After receipt validation, before delivered-state commit | Replay verification then commit; do not trust an uncommitted in-memory token after restart |
| Concurrent old/new dispatcher around lease expiry | Old fencing epoch cannot overwrite newer state or create unbounded duplicate attempts |
| Permission revocation races a reserved attempt | No new reservation or retry after revocation; any already in-flight/possibly-sent attempt remains explicitly unknown until receipt/expiry and may accept later verified receipt evidence |
| Expiry races receipt arrival | Apply the explicit authenticated receive-time policy; do not use sender observation time as recipient receive time |
| Storage full / I/O error / corrupt record | No silent eviction or reported delivery; fail closed and retain recoverable diagnostics within bounded space |
| Same event ID, different event contents | Hard conflict; keep prior durable state and surface evidence corruption |

## 6. Resource and operational limits

All limits must be explicit, independently configured and tested at limit and limit+1:

- outbox/inbox item count and total bytes;
- maximum envelope and metadata size;
- attempts per message, retries per peer, and global concurrent dispatches;
- age and size of evidence/dedupe retention;
- per-peer relay storage, bandwidth, and energy budgets;
- event replay batch and recovery work budget;
- receipt verification concurrency;
- clock-skew tolerance and expiry uncertainty behavior.

Backpressure must reject or defer new work explicitly. It must not silently evict accepted, unexpired outbox records or the dedupe evidence needed to prevent duplicate presentation. A deployment with weaker storage durability must state that weaker guarantee.

## 7. Qualification gates

These are proposed engineering gates, not existing results:

- **0** false `Delivered` transitions from adapter/gateway ACKs or corrupted/replayed receipts.
- **0** duplicate logical inbox items across duplicate paths, event replay, crash/restart or concurrent delivery attempts.
- **0** sends newly initiated at/after expiry or after applicable revocation.
- **100%** of crash points leave each accepted record recoverable as queued, possibly sent, delivered, expired, revoked, failed or explicitly unknown.
- **100%** of conflicting duplicate IDs/event IDs fail closed.
- **100%** bounded behavior at item/byte/attempt/event-retention limits and limit+1.
- Replaying the same valid evidence event 1, 2, 10 or 1,000 times yields the same final logical state as applying it once.
- Fault injection covers every row of the matrix on the exact tested commit. State the database, filesystem, sync policy, device/OS and power-loss assumptions; a process-kill test alone is not proof of power-loss durability.
- All benchmark reports bind the exact source SHA, schema, corpus, configuration and toolchain. Queued CI is not a pass.

## 8. Current contract prototype status

The isolated crate now has `crates/pulse-transport-core/src/delivery_journal.rs`, an executable **in-memory reference model** for the storage-neutral journal boundary. It defines:

- immutable identity as logical message ID + recipient ID + a caller-computed 32-byte envelope commitment;
- optimistic stream revision compare-and-swap, with owned snapshot/history reads so a database backend need not expose references into a process-local cache;
- a versioned persisted-event DTO carrying an explicit per-message stream revision. Replay rejects revision gaps/reordering and accepts duplicate replay only at the original committed revision;
- global event-ID idempotency and explicit rejection when an event ID is reused for different content;
- a starter hard cap of 256 journal events per logical message in the in-memory reference model; a durable backend must return explicit capacity/storage errors and cannot silently pretend a failed history read is empty;
- monotonic worker epochs that fence stale workers, treating a fenced in-flight attempt as possibly sent / unknown;
- distinct policy-revocation handling: block future attempts while retaining a reserved in-flight attempt as unresolved, so an authenticated late adapter outcome may be recorded without permitting retries and a verified recipient receipt may still resolve the delivery;
- bounded attempt count, attempt outcome transitions, retry-not-before checks, expiry/revocation and receipt binding;
- a versioned `PersistedJournalEvent` DTO whose receipt variant stores encoded signed bytes plus a verifier-version label—not a trusted Boolean or capability—and rehydrates only after the caller selects the matching verifier and the raw receipt is re-verified against the exact message, recipient and envelope commitment;
- deterministic in-memory replay of a complete sample event history, plus negative controls for tampered receipt bytes, altered envelope commitment and unsupported event schema versions.

The reference model carries the envelope commitment through the receipt-verifier API and refuses to map a token to `Delivered` for a different commitment, message or recipient. This protects the local contract boundary; it does not prove that a real receipt signature cryptographically encodes the commitment. The configured production verifier must verify the signed receipt's commitment field (or equivalent immutable context). The reference model does not claim production cryptographic verification itself: the receipt verifier remains a trust boundary. It also does not survive process exit, provide a database transaction, serialize a stable storage schema, or prove filesystem/power-loss durability. Every future database backend must implement compare-and-swap, event-ID uniqueness, the immutable identity constraint, and fencing atomically—not as separate read-then-write operations. The owned-read journal API avoids references into an in-memory cache, while the 256-event cap bounds the reference model's per-message history; production storage still needs explicit retention/compaction rules and documented power-loss durability assumptions. The event's raw receipt material must be re-verified on load; a saved status/boolean or process-local capability is not cryptographic evidence.

This code is a contract test harness and integration seam, not the supported Pulse outbox/inbox implementation. Rust compile/test evidence remains required.

## 9. Research basis and limitations

- [RFC 9171 — Bundle Protocol Version 7](https://www.rfc-editor.org/rfc/rfc9171.html) explicitly says BP itself does not ensure end-to-end bundle delivery; application-level mechanisms are needed.
- [NATS JetStream documentation](https://docs.nats.io/concepts/jetstream) describes at-least-once delivery, stable publication IDs, duplicate tracking, and double acknowledgements as separate mechanisms. Its duplicate window is a configured system property—not a universal exactly-once guarantee.
- [Apache Kafka delivery semantics](https://kafka.apache.org/35/design/design/) distinguishes publishing durability from consuming semantics and cautions that exactly-once claims have boundaries under process and storage failure.

These are design precedents only. Pulse's contract, implementation and exact qualification evidence must stand on their own.

## 10. Non-goals for this design tranche

- No claim that the current in-memory lifecycle is crash-safe.
- No database-engine choice imposed by the transport-neutral crate.
- No plaintext to transport adapters.
- No transport, relay or model permitted to bypass authorization, consent, expiry or privacy policy.
- No hardware, radio interoperability, or release qualification implied by this document.
