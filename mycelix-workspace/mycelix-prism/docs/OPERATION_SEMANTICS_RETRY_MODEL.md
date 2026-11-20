# Prism operation-semantics-aware retry and hedging model v0.1

**Status:** dependency-free, test-only reference model with 39 deterministic tests. It is not yet wired into EgressCapability, the production Reqwest transport, Envoy filters, or a real idempotency receiver. No runtime result is claimed.

## The shift: retry authority comes from operation semantics, not the HTTP verb

A network error answers "did transport complete?" It does not answer "is repeating this operation safe?" A method name or an Idempotency-Key header is not evidence that a particular resource actually implements the claimed behavior.

Prism should carry an explicit, versioned operation-effect contract alongside the one-use request capability:

- **ReadOnly:** the resource contract says the requested effect is read-only. Incidental access logs/metrics may still occur; the claim concerns the effect requested by the caller.
- **IdempotentEffect:** repeating the exact same operation has the same intended effect as applying it once. Incidental logs, audit records or revision history may still be recorded more than once.
- **KeyedDeduplication:** the resource publishes and enforces an idempotency contract scoped to a unique key and exact request fingerprint, including key expiry, concurrent duplicates, and completion-consistent replay.
- **NonIdempotent:** repetition may create another effect. If dispatch is uncertain, hold and reconcile rather than retrying.

The contract is bound to logical operation ID, workload, principal, exact origin, method, request digest and expiry. A resource contract may explicitly classify a POST as idempotent. A bare method string or a caller-supplied Idempotency-Key field cannot self-authorize retry. A standards-aware method registry may supply the method's standardized semantics, but Prism must still bind that assertion to the actual route/resource policy and request scope.

## Retry versus hedging

A sequential retry starts after the prior attempt has failed or timed out. A hedge is more demanding: it starts another upstream request while the original remains in flight.

The model separates them:

- Read-only and explicitly idempotent operations may be retried after an ambiguous dispatch only while their exact resource contract is valid. Hedging additionally requires explicit resource permission.
- A keyed operation may be replayed with the **same** key, request fingerprint and scope only while the receiver's published contract is valid. A retry after completion should return a response consistent with the completed operation. A receiver may reject a duplicate while the first operation remains in progress (e.g. a conflict); Prism must not interpret that conflict as proof of completion.
- The stronger CoalesceWithOriginal behavior permits hedging only when the receiver atomically joins the duplicate to the original in-flight operation. A RejectWhileInProgress contract permits a sequential same-key replay, but this model denies a hedge because the second in-flight request would only be rejected rather than safely sharing the first operation.
- Non-idempotent operations cannot be retried or hedged from an unknown dispatched state. They require a tuple-bound, authoritative DefinitelyNotApplied result or explicit manual reconciliation.
- A trusted, attempt-bound proof that no request bytes were written permits a new attempt regardless of semantic class. The proof must not be inferred from the absence of a new kernel connect event, particularly when HTTP/1 keep-alive or HTTP/2 multiplexing is in use.

Every admitted retry still requires a fresh attempt ID and a fresh one-use ExchangeCapability. Same-key replay is a new transport attempt for the same logical operation, not a new logical effect.

## Standards and current industry behavior

### RFC 9110

RFC 9110 §9.2.2 defines idempotence by the *intended effect* of multiple identical requests. PUT, DELETE and safe methods are idempotent. It explicitly permits repeating idempotent requests after communication failure before the response is read, while advising against automatic retry of non-idempotent requests unless their semantics are known to be idempotent or the client can detect that the original was not applied.

RFC 9110 also clarifies that idempotence concerns the requested effect: a server may still log each request, retain revision history, or perform other non-idempotent incidental actions. Prism's contract therefore classifies the intended operation effect, not every internal server event.

Reference: https://www.rfc-editor.org/rfc/rfc9110.html#section-9.2.2

### IETF Idempotency-Key work in progress

As of 20 November 2026, the current discovered document is draft-ietf-httpapi-idempotency-key-header-08, published 22 October 2026 and expiring 25 April 2027. It is an active Internet-Draft, **not a published RFC**; cite it as work in progress, not as an adopted standard.

The draft says a general client cannot assume a server respects an Idempotency-Key header. It places responsibility on the resource to publish its idempotency specification, including expiry behavior where applicable. Keys must not be reused with a different request payload. It describes completed duplicates as requests that should receive a response consistent with the completed user intent, and concurrent duplicates as requests that should receive a resource conflict response.

The reference model adopts stronger local requirements where they are needed for safe automatic replay: a nonzero key digest, exact request-fingerprint binding, rejection of same-key/different-fingerprint reuse, atomic deduplication, durable behavior across server failover, a documented retention window, and completion-consistent replay. The draft permits a resource-specific fingerprint strategy; Prism currently models an exact canonical request digest as the conservative profile.

References:
- https://datatracker.ietf.org/doc/html/draft-ietf-httpapi-idempotency-key-header-08
- https://datatracker.ietf.org/doc/draft-ietf-httpapi-idempotency-key-header/

### Envoy v1.40

Envoy exposes a mature retry policy, timeout budget and backoff strategy. Its reset-before-request policy only retries when request headers have not been sent upstream. Broader reset and 5xx conditions include failures where an upstream may have received the request. Envoy also supports hedging: a retry can be issued without resetting the original request, leaving multiple upstream requests in flight.

Prism should compare against both narrow pre-send retry and broader retry/hedging configurations. Its differentiation is not that Envoy lacks retries; it is that the retry decision is tied to an explicit operation-effect contract and evidence of remote-effect safety.

References:
- https://www.envoyproxy.io/docs/envoy/v1.40.0/configuration/http/http_filters/router_filter
- https://www.envoyproxy.io/docs/envoy/v1.40.0/intro/arch_overview/http/http_routing

## Executable model

File: prism-egress/tests/retry_semantics_model.rs

The 39 deterministic tests cover:
- non-idempotent unknown outcomes and safe/idempotent operation contracts;
- POST with explicit idempotent resource semantics;
- method-only negative controls;
- exact method/origin/principal/request binding and contract expiry;
- same-key replay with request fingerprint and key-expiry checks;
- missing mismatch rejection, missing completion-consistent replay, unknown concurrent behavior, atomic deduplication and server-failover durability;
- sequential duplicate rejection versus concurrent hedge coalescing;
- explicit permission for read-only/idempotent hedging;
- exact no-write proof binding and authoritative remote Applied/DefinitelyNotApplied reconciliation;
- eventually-consistent absence, mismatched operation/origin and generated non-idempotent replay controls.

The model is source-present only until the exact PR head compiles and executes it. It proves no production behavior today.

## Production API direction

Extend the production capability contract with a private OperationEffectContract:
- contract digest/version and issuer;
- exact workload/principal/origin/method/request digest scope;
- effect class (read-only, idempotent effect, keyed deduplication, non-idempotent);
- expiry and policy generation;
- if keyed: key digest, fingerprint rule, atomicity, concurrent duplicate behavior, failover durability, and retention deadline;
- if hedging is enabled: explicit parallel-duplicate permission.

The application must not be able to set idempotent=true as an unchecked boolean. Contract creation should be mediated by trusted route/resource policy, with provenance and version retained in the receipt. The transport adapter still needs the durable dispatch token and fresh exchange capability. Retry decisions must not be hidden inside Reqwest or an ambient proxy.

## PEAB retry/hedge qualification

Use the same receiver and fault schedule for:
1. no retry;
2. Envoy reset-before-request;
3. broad Envoy reset/5xx retries with retry budget;
4. Envoy timeout hedging;
5. generic client retries without server idempotency;
6. keyed receiver with atomic, persistent deduplication;
7. durable-workflow activity with idempotent receiver;
8. Prism semantic classifier, then Prism classifier plus durable journal and independent evidence.

Cross:
- operation effect class (read-only, idempotent, keyed, non-idempotent);
- method (GET, HEAD, PUT, DELETE, POST, PATCH) and route declaration (conforming, custom idempotent, unknown);
- failure location (pre-connect, before write with transport proof, after partial write, after full request, after server commit/before response, response lost);
- duplicate behavior (coalesce, reject while in progress, completion replay, unknown);
- same key/same payload, same key/different payload, different key/same payload;
- key expiry before/after retry and resource failover with stale/absent dedup state;
- sequential retries versus hedges at 1/2/4 in-flight copies;
- concurrency 1/16/64/256 and fault rate 0/1/10%.

Report:
- false-safe retry classifications (target 0);
- duplicate remote effects per logical operation (target 0 under declared server contract);
- non-idempotent unknown retries/hedges (target 0);
- valid idempotent/keyed operations incorrectly quarantined;
- unknown outcomes correctly held;
- key collision, mismatch and expiry handling;
- retry and hedge amplification, extra upstream work and cancelled-but-still-running requests;
- p50/p95/p99 latency, success/availability, CPU, RSS, allocations, receiver storage IOPS and journal fsync latency.

Provisional targets after a matched baseline exists:
- zero false-safe classifications and zero duplicate remote effects under the declared contract;
- zero non-idempotent retries or hedges from unknown state;
- at least 99.9% correct admission/hold classification for the declared test corpus (separate false-allow and false-deny rates; security gate requires 0 false allows);
- report the availability cost of holding unknown operations separately;
- publish latency and extra-work Pareto curves for retries and hedges.

A low-latency score may not compensate for an unauthorized duplicate effect. Do not call the model "exactly once": it only permits safe replay under a declared resource contract and leaves unsupported/unknown cases unresolved.
