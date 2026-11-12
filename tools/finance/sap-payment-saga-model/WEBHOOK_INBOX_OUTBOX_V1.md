# Provider webhook inbox/outbox V1

Status: PostgreSQL schema and atomic-acceptance function specification. Not yet wired into the existing fiat sidecar or verified against a live PostgreSQL instance.

## Purpose

A valid provider signature is only the ingress authenticity boundary. The application needs a separate durable acceptance boundary so that a crash, duplicate delivery, or concurrent web server replica cannot lose an accepted event or create duplicate processing work.

Stripe recommends verifying the raw request body and signature before acting, then returning a successful response promptly rather than waiting for complex payment work. Mycelix must preserve that low-latency posture without acknowledging an event before it has been durably recorded. The design therefore commits the encrypted inbox record and one outbox intent in the same PostgreSQL transaction, then returns 2xx; effect execution occurs asynchronously.

Official Stripe guidance:
- Signature verification and raw-body requirement: https://docs.stripe.com/webhooks#verify-manually
- Delivery / endpoint handling: https://docs.stripe.com/webhooks

## Schema

Migration: `migrations/0001_provider_webhook_inbox.sql`.

### Inbox

The inbox primary uniqueness boundary is:

`provider_id + provider_account_scope + endpoint_id + environment + event_id`

This distinguishes test and live mode and allows separate registered endpoints to process the same provider event independently.

The immutable event subject records:
- provider, provider account and resource/connected-account scopes;
- endpoint and test/live environment;
- provider event ID/type and provider event-created timestamp;
- delivery signature timestamp (not the same field as the provider event-created timestamp);
- signature profile, key reference and verifier revision;
- provider-contract digest;
- SHA-256 of exact raw request bytes;
- application-encrypted raw body bytes and encryption-key reference;
- verification and receipt timestamps.

The table stores encrypted raw bytes, not plaintext webhook JSON. The signature secret is never stored. The adapter must compute the digest from the exact raw bytes that passed HMAC verification, then encrypt those same bytes. Database roles should grant the ingress service INSERT/SELECT only on the immutable inbox and conflict tables; UPDATE/DELETE belongs to a separate retention/cryptographic-erasure role.

### Delivery-attempt ledger

The `provider_webhook_delivery` table preserves every successfully authenticated delivery attempt separately from the canonical inbox event. A duplicate delivery is still a new delivery observation: it records the delivery-time signature timestamp, signing-key reference, verifier revision, provider-contract digest and raw-body digest, linked to the already accepted inbox row. A collision links the attempt to both the existing inbox record and its conflict quarantine record.

This avoids silently discarding operationally useful evidence when secrets rotate or the same event is retried. The table does not store another copy of the body; the inbox keeps the canonical encrypted body and the conflict table keeps encrypted changed-content evidence. Delivery rows are append-only by service permissions, not updatable delivery state.

Dispositions are:
- `accepted`: first verified delivery inserted the inbox, object-generation notification and event outbox row in one transaction;
- `duplicate`: exact event scope/body/contract replay; no new object generation and no second outbox row;
- `conflict`: same event key with changed content, provider region, contract, object, API version, payload kind or resource account; no new object work.

The exact key-rotation metadata is an audit record, not proof that the signing key remains trusted today. The signature verifier and key-lifecycle policy remain responsible for that decision.

### Conflict quarantine

A repeated event ID with a different raw-body digest, provider contract digest, event type or resource account scope is not an ordinary duplicate. The function records a conflict with both body digests and the encrypted incoming bytes, then returns `conflict`. It does not enqueue another effect.

This makes the rule explicit: same event ID + same content/scope is an idempotent duplicate; same ID + changed content is a security/reconciliation exception.

### Event-processing outbox and object refresh queue

The event outbox contains exactly one stable `work_id` per accepted inbox record. The work ID is a local processing identity derived from the event delivery identity. It is **not** a provider idempotency key and **not** an economic `effect_id`. Its unique `inbox_id` foreign key prevents duplicate local work for one accepted delivery.

Event outbox states are:
- `pending`: event accepted and awaiting object refresh;
- `indeterminate`: the associated read/reconciliation work did not complete with a durable result;
- `reconciled`: a verified provider-object state refresh covered this event's generation;
- `quarantined`: an operator/reconciler isolated the event.

The separate `provider_webhook_object_reconciliation` table is keyed by provider/account/resource-account/environment/object ID. It coalesces notifications for the same provider object with monotonically increasing local `requested_generation` and `completed_generation` counters. These are local queue generations, not provider timestamps or an event order.

Every newly accepted event increments `requested_generation`. A worker leases the current generation and performs a provider GET/read-only refresh using a pinned provider contract and API profile. If another notification arrives while the fetch is active, the generation increments while the existing lease remains intact. After the fetch, the worker records the canonical digest of the current provider object state and completes only the generation it actually fetched. If `requested_generation` is now higher, the object immediately returns to `pending` for another refresh.

The worker then marks event outbox rows as `reconciled` only through the generation it covered. A late event remains pending, even if an earlier refresh completed. The object fetch is a read; it must not dispatch a new payment/refund.

### Why webhooks are notifications, not an ordered ledger

Stripe explicitly does not guarantee webhook delivery order. Events can arrive more than once, and event creation timestamps are not a reliable total-order cursor (ties are possible). Therefore:
- Deduplicate delivery with provider + provider-account + endpoint + environment + event ID.
- Preserve each authenticated delivery as an immutable receipt and exact raw-body digest.
- Bind event type, underlying object ID, payload form and API version.
- Treat `event_created_at` and signature timestamp as distinct diagnostic/verification fields, not ordering cursors.
- Fetch/reconcile the provider object's current state using its object ID and a pinned API version. Do not blindly apply a snapshot from an old webhook as the current state.
- For thin notifications, retrieve the referenced provider object before using its fields.
- Preserve duplicate and reordered event receipts; reconcile the object state, not event arrival order.

The inbox binds a receipt to: provider/account/endpoint/resource account, environment, event ID/type, object ID, event API version, payload form, raw-body digest, signature profile/key reference/verifier revision and provider-contract digest.

### Atomic acceptance

The `accept_provider_webhook_v1` function:
1. inserts the verified encrypted event under the unique provider/account/endpoint/environment/event key;
2. on exact duplicate, returns `duplicate` without incrementing the object generation or creating another outbox row;
3. on changed body, contract, object ID, API version, payload form, type or resource scope, persists a conflict and returns `conflict`;
4. on new delivery, atomically increments the object's requested generation, records that generation on the inbox row and inserts one event-processing outbox row with a stable `work_id`;
5. returns `accepted`.

The handler must commit the SQL transaction before sending 2xx. A transaction failure means no event accepted; provider retry can redeliver the same event. The unique constraint serializes competing replicas at the database boundary.

This is an application contract: PostgreSQL cannot independently prove that an HMAC was verified. Only the trusted ingress service should be granted EXECUTE on the acceptance function, and only after raw-body signature verification and application-layer encryption.

### Crash and concurrency rules

- Crash before database commit: no event accepted; provider retries.
- Crash after commit but before 2xx: provider retries; same event and digest returns `duplicate`.
- Two replicas receive the same event concurrently: unique constraint yields one accepted row and one duplicate.
- Same event ID with changed body/contract/object/version: durable conflict quarantine; no second outbox work row.
- Two different events for the same object: each gets a unique inbox receipt; object generation increments and reads can be coalesced.
- New event during an active object refresh: requested generation advances while the active lease stays attached to its captured generation; completion leaves a newer generation pending.
- Crash/lease expiry during object GET: the read task becomes `indeterminate`, then may be safely retried because GET is side-effect free. It does not change a payment effect.
- Object state refresh succeeds: commit the object-state digest and advance completed generation only through the leased generation; mark corresponding event work rows reconciled in the same DB transaction.
- A webhook saying refund updated/failed is not itself a final provider settlement receipt unless the provider contract says that particular object/event status is final and its object state is independently verified.
- No provider webhook or object refresh creates, cancels, or retries an economic effect. Actual payment/refund dispatch remains in the compensation/payment saga with its own owner grant, liquidity reservation, stable economic effect ID and provider retry permit.

The migration includes a worker claim query using `FOR UPDATE SKIP LOCKED` on the object-reconciliation table. A read lease expiry moves to `indeterminate`; a worker may later repeat the read. There is no automatic replay of an external payment effect from this queue.

## Security and privacy requirements not implemented by the DDL

- TLS termination (Stripe requires HTTPS in live mode).
- Provider source-IP filtering as defense in depth; not a replacement for HMAC verification.
- Secret storage, rotation, and key-reference lifecycle.
- Encryption and rotation of per-record/body encryption keys.
- Database role provisioning, backup/restore and retention/crypto-erasure policy.
- Provider API lookup/absence semantics and signed/verified settlement evidence.
- Holochain action resolution, owner grant verification and actual economic effect execution.

The database schema records a `verified_at` assertion and verifier profile. It does not verify the cryptographic signature itself. The Rust/Go HMAC module is the current reference authentication implementation; sidecar route wiring remains issue #6079.

## Benchmarks and qualification plan

Run against a real PostgreSQL instance with the migration and exact build SHA pinned.

Corpus and contention:
- 1k / 10k / 100k / 1m unique events;
- 0%, 50%, 90%, and 99% exact duplicates;
- 0.001%, 0.1%, and 1% same-event-ID / changed-body conflicts;
- 1 / 2 / 8 / 32 / 64 concurrent ingress replicas;
- body sizes 1 KiB / 16 KiB / 128 KiB / 1 MiB (subject to configured HTTP limit);
- worker pool sizes 1 / 4 / 16 / 64;
- injected crash after inbox insert, after object-generation increment, after outbox insert, after commit-before-2xx, during object GET, after GET-before-generation-commit and after generation-commit-before-event-work-marking.

Measurements:
- ingress p50/p95/p99, transactions/sec and CPU per accepted event;
- duplicate/conflict classification correctness;
- unique-constraint contention and transaction retry rate;
- inbox-to-outbox commit latency;
- queue depth, oldest-pending age and lease-expiry-to-reconciliation delay;
- object-refresh claim throughput and lock wait time; generation coalescing ratio;
- payload encryption/decryption cost and encrypted storage growth;
- zero duplicate event-processing work IDs for one delivery scope; zero cross-scope object refresh identities;
- zero event acknowledgements before durable commit;
- zero expired-lease automatic re-dispatch without reconciliation.

Do not publish a throughput/SOTA claim until tests include the actual PostgreSQL version, storage class, durability settings, replica count, isolation level, payload corpus, and raw measurements. The target is correctness under crash/replay, ordered generation completion under out-of-order event delivery, and stable p95/p99 under contention—not a higher benchmark number on a synthetic in-memory map.

## Industry relationship

The transactional outbox pattern addresses the dual-write problem by storing state and event intent atomically. AWS Saga guidance explicitly calls out dual writes, event preservation, idempotent participants, eventual consistency and the lack of saga transaction isolation; it recommends semantic locking where concurrent workflows can otherwise operate on stale data:
- https://docs.aws.amazon.com/prescriptive-guidance/latest/cloud-design-patterns/saga-choreography.html
- https://docs.aws.amazon.com/prescriptive-guidance/latest/cloud-design-patterns/saga-orchestration.html

Mycelix's intended differentiator is to bind that durable inbox/outbox boundary to a provider-contract digest, authenticated owner authority, stable compensation identity, independently reconstructed journal and reconciliation evidence. The migration alone does not achieve that integration or outperform an industry provider.

## Next implementation steps

1. Restore a buildable fiat sidecar; currently only `sidecar/src/main.rs` exists despite `mod config;` and `mod stripe;` declarations.
2. Wire the raw-body HMAC verifier before JSON parsing and before any event mutation.
3. Add the SQL migration to a real database migration runner and test the atomic acceptance function with concurrent connections.
4. Encrypt payloads at the application boundary and provision distinct DB permissions for ingress, workers and retention.
5. Implement outbox claiming and a crash-recovery worker; expired leases must go to `indeterminate`.
6. Bind provider event IDs and the provider contract digest into compensation/payment replay events.
7. Qualify a full fake-provider / real-PostgreSQL / real-Holochain integration before any production claim.


## Append-only outbox transition log

Migration `migrations/0002_provider_webhook_outbox_event_log.sql` adds `provider_webhook_outbox_event`. The mutable `provider_webhook_outbox` row is now explicitly a query projection; the event stream is the replay subject.

Rust reference:
- `src/outbox.rs`

Independent Go reference and adversarial suite:
- `../sap-payment-saga-go-verifier/outbox.go`
- `../sap-payment-saga-go-verifier/outbox_test.go`

Shared eight-frame KAT:
- `fixtures/sap-webhook-outbox-v1.kat`

The event digest profile `MXSAP-WEBHOOK-OUTBOX-EVENT-V1` binds sequence, stable effect ID, event kind, actor, timestamp, prior/next state, provider contract digest, request digest, lease, authority-grant digest, retry-permit digest, absence-evidence digest, applied-effect evidence digest, reason code and previous event digest.

The reference reducers enforce:
- sequence continuity, previous-digest chaining and anchored tail digest;
- one immutable effect ID, provider contract digest and request digest for the stream;
- accepted → pending → leased;
- a dispatch-authorization event bound to the active lease before an indeterminate or applied outcome;
- lease expiry → indeterminate, never directly back to pending;
- indeterminate → pending only through a retry-authorization event carrying owner authority, provider retry permit and authoritative-absence evidence;
- effect-applied only with the active lease and independently verified effect-evidence digest;
- changed event bytes, changed identity, reordering and truncation are rejected.

The SQL function `append_provider_webhook_outbox_event_v1` uses `SELECT ... FOR UPDATE`, verifies next sequence, previous digest, current projection state, lease identity and allowed transition, then appends the event and updates the projection in one transaction. A trigger rejects UPDATE/DELETE of event rows. A deferred constraint trigger ensures every new outbox row has an `accepted` sequence-1 event before commit.

**Required ingress transaction:** after raw signature verification and application-layer payload encryption, begin one PostgreSQL transaction, call `accept_provider_webhook_v1(...)`, then call `append_provider_webhook_outbox_event_v1(... sequence 1, kind accepted ...)`, then commit. Only after commit may the HTTP handler acknowledge acceptance. Exact duplicate delivery returns duplicate and does not append another acceptance event. A changed event under the same id is quarantined, not enqueued.

The SQL layer validates digest shape and continuity but does not recompute canonical event hashes. The trusted adapter must compute the Rust/Go canonical digest; a separate replay job must recompute every event and compare the anchor and projection. This is an explicit claim limit, not a hidden database guarantee.

The benchmark plan now includes exact chain replay and Postgres append/claim/recovery under 1–64 concurrent workers, crash injection at each database/external-effect boundary, and p50/p95/p99, queue age, lock wait, duplicate conflict and zero duplicate stable effect identities. No Postgres performance result is claimed until run on identified hardware/settings with raw samples.
