# Prism execution and recovery reference model v0.1

**Status:** executable test-only reference model and design contract. It is not a production write-ahead journal, transport adapter, server idempotency service, signed receipt, or exactly-once guarantee.

## The paradigm shift: requests are transactions with evidence

An outbound request has at least four distinct facts:

1. **Intent:** a workload and principal request a specific operation against an exact origin and request digest.
2. **Authority:** Prism grants a short-lived, one-attempt capability under a particular policy and network scope.
3. **Dispatch:** a transport adapter may begin handing request bytes to a socket (possibly an already-established pooled socket).
4. **Effect and observation:** the remote service may apply the operation; the application may receive a response; an independent observer may corroborate the socket and peer.

These facts must not be collapsed into one boolean named success. A TCP connect is not an HTTP request. A request write is not proof of a server-side commit. A response read is not, by itself, an independently reconciled business effect. A missing response is not proof that the request was not applied.

The reference model is prism-egress/tests/execution_recovery_model.rs. It has 25 deterministic tests and no non-standard dependencies. The tests are source-present only until executed on the exact PR head.

## Durable dispatch boundary

The production contract proposed for implementation is write-ahead:

- Create a unique logical operation ID and a unique transport attempt ID.
- Bind the attempt to workload, principal, exact origin, method, canonical request digest, policy generation, network generation, and (when supported) a stable idempotency key.
- Append and durably flush Prepared, then Authorized.
- Immediately before the transport adapter can write request bytes, append and durably flush DispatchMarked.
- The transport adapter must reject any write unless the matching durable dispatch marker has been committed.
- Record transport disposition and application response in later journal records; never rewrite an earlier phase in place.

DispatchMarked means **the request may have been dispatched**. It is intentionally conservative. If the process dies after the marker but before the first write, recovery may not infer that nothing happened. Conversely, absence of a dispatch marker is a safe pre-dispatch boundary only if the durable journal and adapter gate make it impossible to write first and persist later.

The journal needs monotonic sequence numbers, a previous-record digest, an append-only record digest, durable flush semantics, and a startup verifier. Hash chaining detects accidental mutation/reordering but is not, by itself, authentication against a privileged attacker who can rewrite the entire journal. Production needs an explicit key, storage, rollback, and trust-root threat model.

## Recovery decision table

| Durable state and evidence | Recovery action | Why |
|---|---|---|
| Durable Prepared or Authorized; no DispatchMarked exists | May create a new attempt after fresh authorization | The adapter was never permitted to dispatch |
| Journal durability or write-ahead ordering is uncertain | Hold as unknown | A missing marker is not trustworthy evidence |
| DispatchMarked; no response or authoritative remote result | Hold; query a non-mutating remote status/idempotency endpoint when available | The remote side effect may already have happened |
| DispatchMarked; a valid linearizable server record says applied | Reconcile the original operation | Do not issue a second mutation |
| DispatchMarked; a valid linearizable server record proves not applied | A new attempt may be authorized with the same logical idempotency key and a fresh one-use capability | The earlier attempt is authoritatively resolved as absent |
| Remote lookup is eventually consistent, stale, expired, missing, or ambiguous | Hold as unknown | A cache miss is not proof of non-application |
| Transport reports a response but socket evidence is missing | Preserve the response; mark independent socket corroboration incomplete | Pool reuse can explain no new connect event |
| Kernel event contradicts workload or peer authority | Contradiction; quarantine | Do not average a security contradiction into a score |

The production implementation must never send a second mutating request merely because a timeout elapsed, a process restarted, a response was missing, a kernel connect event was absent, or an eventually-consistent lookup returned not-found.

## Why kernel connect observation is not an HTTP dispatch oracle

A kernel connect hook reports connection establishment attempts. It does not report every HTTP request on an existing connection. HTTP/1 keep-alive, HTTP/2 multiplexing, and any future pool can carry a new request without a new connect(2) event. Therefore:

- A matching connect event can corroborate a socket's workload and peer.
- No connect event cannot prove no HTTP request was written.
- A connect event cannot prove request bytes, TLS completion, response bytes, or remote business effect.
- Observer loss or detach/reattach gaps invalidate independent coverage.
- A response on a pooled connection requires a connection-identity bridge and a retained, independently observed connection lease if kernel peer corroboration is required.

The current reference model keeps this limit explicit. It does not invent a request ID in a kernel event, nor equate socket-cookie identity with an HTTP/2 stream ID.

## Idempotency and remote reconciliation

The preferred recovery primitive is a **non-mutating, authoritative operation-status lookup**, addressed by a server-recognized idempotency key or durable operation ID.

A usable server contract must define:
- key namespace and principal scope;
- origin/endpoint and method scope;
- exact request-digest binding (same key with a different body must fail);
- concurrent duplicate behavior (one logical effect, deterministic replay or conflict);
- a documented retention period that exceeds the entire recovery horizon;
- a linearizable or otherwise explicitly sufficient status lookup;
- stable semantics across server failover, region failover, and record restoration.

A not-found result from an eventually-consistent replica is not an authoritative absence proof. A lookup outside the retention window is not an absence proof. A key collision or request-digest mismatch is a contradiction, not a retry opportunity.

When the status service authoritatively reports Applied, reconcile the original operation and retain its result. When it authoritatively proves DefinitelyNotApplied, a new transport attempt may be authorized using the same logical idempotency key, but it must get a fresh attempt ID, fresh single-use ExchangeCapability, fresh current policy check, and current network scope. If no adequate server contract exists, quarantine the unknown outcome for explicit reconciliation rather than retrying.

This model deliberately does not promise "exactly once" delivery. The strongest honest claim is conditional: at-most-one remote effect per logical key **only when the remote service enforces the documented deduplication contract**, with unresolved states preserved as unknown.

## Industry comparison

- **HTTP semantics (RFC 9110 §9.2.2):** safe methods plus PUT and DELETE are defined as idempotent in intended effect; non-idempotent requests should not be automatically retried without a way to establish idempotent semantics or detect that the original was not applied. Prism should enforce this as a typed recovery policy rather than a global retry toggle.
- **Durable workflow engines:** Temporal documents that an Activity may execute more than once and recommends idempotent Activity code. Its event history makes orchestration durable, but external activity side effects still need application-level idempotency. Prism focuses more narrowly on the network authority and evidence boundary around each outbound attempt.
- **Service-mesh retries:** Envoy provides mature timeout, retry, circuit-breaker, and upstream connection-pool controls. These optimize availability, but retry policy alone does not prove that an arbitrary remote side effect was not applied. Compare configured behavior and the remote server contract rather than claiming Prism replaces Envoy.
- **Network policy engines:** Cilium supplies workload-level L3/L4/L7 controls and DNS-derived policy. Its DNS policy documentation describes TTL-dependent new-connection behavior and the fact that existing connections can remain allowed. That complements Prism's per-request authority; neither plane should be mistaken for the other.

Primary references:
- RFC 9110 §9.2.2: https://www.rfc-editor.org/rfc/r9110.html#section-9.2.2
- Temporal Activities: https://docs.temporal.io/activities
- Temporal Activity idempotency guidance: https://docs.temporal.io/activity-definition
- Envoy v1.40 retry policy: https://www.envoyproxy.io/docs/envoy/v1.40.0/configuration/http/http_filters/router_filter
- Cilium DNS-based policy and limitations: https://docs.cilium.io/en/latest/security/dns/

## PEAB fault-injection and benchmark protocol

Use the same receiver, request corpus, fault schedule, storage durability setting, and workload concurrency for all baselines:

1. No retry (control).
2. Generic retry-enabled HTTP client with no server idempotency contract.
3. Retry-enabled client with a server-enforced idempotency key.
4. Envoy retry policy with a documented remote idempotency contract.
5. Durable workflow activity with idempotent receiver semantics.
6. Prism pre-dispatch journal gate with a non-mutating authoritative status lookup.
7. Prism with the same remote contract plus independent socket/peer reconciliation.

Inject process termination or power-loss at every boundary: before and after each journal append; before and after durable flush; between durable DispatchMarked and first byte; during partial request write; after all request bytes are handed off; after server commit but before response; while response headers/body are read; after response read but before the receipt is durable; during pooled HTTP/1 reuse; during concurrent HTTP/2 streams; during cancellation; during DNS/policy generation change; and during observer event loss or detach/reattach.

Report at least:
- duplicate remote effects per logical operation (target: zero under declared server contract);
- false DefinitelyNotApplied classifications (target: zero);
- retries after an unresolved dispatched attempt (target: zero);
- unresolved outcomes correctly held (target: 100%);
- pre-dispatch recovery precision (target: 100% under verified write-ahead ordering);
- authoritative reconciliation coverage and lookup latency p50/p95/p99;
- crash-to-reconciled latency p50/p95/p99;
- journal fsync/flush latency p50/p95/p99, bytes per attempt, write amplification, recovery scan time;
- transport attempts per logical operation and idempotency-key collision/mismatch counts;
- CPU, RSS, allocations, connection reuse, TLS handshake count, and tail request latency.

Security acceptance is non-compensatory: any duplicate effect, false absence proof, unauthorized socket, retry from unknown state, or missing required evidence is a failed gate even if throughput and latency improve. Publish availability loss caused by conservative quarantine as a separate metric; do not hide it in a blended score.

## Production implementation sequence

1. Implement a durable append-only journal with crash-consistent ordering and startup verification.
2. Create a transport adapter boundary that requires the durable dispatch token before any request bytes may be written, including on pooled connections and HTTP/2 streams.
3. Define typed TransportDisposition and RemoteOperationStatus contracts with exact attempt/request/identity binding.
4. Add a fake receiver with a durable idempotency table and linearizable status lookup; inject crash points at every boundary.
5. Integrate real HTTP/1 and HTTP/2 adapters; verify request-write and response-read hooks independently of kernel connect events.
6. Add an independent connection identity bridge for pooled sockets; do not treat a userspace socket cookie copied into a log as independent observation.
7. Only then benchmark against the industry baselines and consider production rollout.

Until these steps are implemented and exact-head execution evidence exists, this model is a source-level hypothesis, not a production safety guarantee.
