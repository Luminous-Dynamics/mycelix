# MYC-COMPUTE-001A — Typed Browser Worker Substrate Plan

## Status

Planning child of `MYC-COMPUTE-000R` / draft PR #1777.

This file is intentionally non-executable. It freezes the smallest implementation
surface for the first shared browser-compute substrate so the future Rust PR does
not accidentally absorb domain semantics.

## Objective

Create one reusable Rust crate for browser worker lifecycle and typed message
transport that can be consumed by Leptos applications and domain-specific WASM
compute adapters.

Proposed crate:

```text
mycelix-workspace/crates/mycelix-browser-worker/
```

The crate owns generic worker plumbing only.

## Proposed modules

```text
src/
  lib.rs
  protocol.rs
  client.rs
  lifecycle.rs
  generation.rs
  limits.rs
  receipt.rs
  error.rs
```

No FL, DP, FHE, ZKP, finance, governance, identity, scientific, or Symthaea
algorithm code belongs in this crate.

## Protocol types

The first executable contract should remain deliberately small.

Conceptually:

```rust
pub struct WorkerRequestV1 {
    pub request_id: RequestId,
    pub operation: OperationId,
    pub profile: ComputeProfileRefV1,
    pub subject: Option<SubjectRef>,
    pub input_commitment: CommitmentRef,
    pub deadline_ms: Option<u64>,
    pub payload: Vec<u8>,
}

pub struct WorkerResponseV1 {
    pub request_id: RequestId,
    pub operation: OperationId,
    pub profile: ComputeProfileRefV1,
    pub worker_generation: WorkerGenerationId,
    pub outcome: WorkerOutcomeV1,
    pub output_commitment: Option<CommitmentRef>,
    pub receipt: BrowserComputeReceiptV1,
    pub payload: Vec<u8>,
}
```

The exact wire/canonicalization profile must be frozen before these values become
stable cryptographic commitments. The first implementation may use
`serde_wasm_bindgen` for transport while treating that transport representation
as non-authoritative unless/until a canonical wire profile is registered.

## Lifecycle

The client should expose explicit states rather than one bool:

```text
Unavailable
Starting
Ready
Degraded
Terminated
Crashed
```

Per request:

```text
Prepared
Dispatched
Accepted
Executing
Completed
Rejected
Cancelled
TimedOut
WorkerFailed
Indeterminate
```

## Worker generation

Every started worker receives a fresh local generation identity.

Responses from an old generation are rejected/quarantined after restart.

Restart does not inherit:

- pending request authority;
- one-shot tokens/capabilities;
- request replay windows;
- protocol-round membership;
- privacy accounting;
- cryptographic key state requiring re-establishment;
- domain state that requires durable recovery.

## Pending request table

The client may maintain a bounded pending table keyed by `RequestId`.

Each entry binds:

```text
request_id
operation
profile
input_commitment
worker_generation
created_at/deadline
disposition
```

A terminal response removes/closes the entry but does not erase its receipt or
historical evidence where the caller chooses to retain it.

Unknown, mismatched, duplicate, or stale terminal responses fail closed.

## Error model

At minimum:

```text
WorkerUnavailable
WorkerStartFailed
WorkerCrashed
MalformedRequest
MalformedResponse
UnsupportedProtocolVersion
UnsupportedProfile
UnknownRequest
CorrelationMismatch
OperationMismatch
ProfileMismatch
InputCommitmentMismatch
StaleWorkerGeneration
DuplicateTerminalResponse
RequestTooLarge
ResponseTooLarge
DeadlineExceeded
Cancelled
TransportFailure
IndeterminateExecution
```

Do not collapse domain rejection into transport failure.

## Resource profile

The generic runtime should allow a caller-supplied bounded profile such as:

```text
max_request_bytes
max_response_bytes
max_pending_requests
max_operation_id_bytes
max_profile_id_bytes
max_execution_ms
```

The first qualification fixtures should use intentionally small bounds so every
limit is easy to exercise.

## JavaScript/WASM boundary

Preferred direction:

```text
Rust request type
  -> serde_wasm_bindgen / bounded transport envelope
  -> postMessage
  -> worker-side Rust/WASM decode
  -> domain adapter
  -> Rust response type
  -> postMessage
  -> Rust client decode
```

A thin JS module may initialize the WASM worker module. It should not implement
business/cryptographic/scientific semantics.

## API shape

A future client API should be narrow and explicit, conceptually:

```rust
BrowserWorkerClient::start(config)
BrowserWorkerClient::is_available()
BrowserWorkerClient::generation()
BrowserWorkerClient::request(request)
BrowserWorkerClient::cancel(request_id)
BrowserWorkerClient::terminate()
BrowserWorkerClient::set_event_handler(...)
```

The API must not contain `verified=true`, `authorized=true`, or similar generic
semantic claims.

## Receipt boundary

`BrowserComputeReceiptV1` is generic execution evidence only.

It binds execution provenance such as:

```text
request/profile/input identity
worker generation
transport/lifecycle disposition
output commitment if supplied
start/end timing evidence
implementation/build qualification reference where available
```

It does not establish the domain theorem.

A DP adapter would emit/compose a DP-specific receipt; FL an FL-specific receipt;
secure aggregation a protocol transcript/receipt; scientific computation a
scientific evidence object.

## Simulation policy

Simulation must be an explicit profile/classification.

The generic client must not automatically substitute a simulation when a real
worker fails for an authoritative request.

A development adapter may deliberately request `DevelopmentSimulation`; that
receipt must carry the simulation classification and cannot be promoted by the
shared layer.

## Prior-art migration

Do not copy either current implementation wholesale.

Praxis provides useful evidence for:

- `web_sys::Worker` integration;
- background lifecycle;
- Leptos reactive consumption.

Symthaea provides useful evidence for:

- correlation IDs;
- pending response resolution;
- broadcast events;
- worker error handling.

The shared crate should avoid inheriting application-specific `Reflect` message
shapes, polling semantics, leaked closure patterns where avoidable, or unsafe
`Send`/`Sync` abstractions without an explicit reviewed justification.

## Qualification child — MYC-COMPUTE-001AQ

Require a real browser-worker test boundary for the strongest path.

Minimum corpus:

1. typed request/response round trip;
2. unknown protocol/profile rejection;
3. malformed request;
4. malformed response;
5. wrong correlation ID;
6. wrong operation;
7. wrong profile;
8. wrong input commitment;
9. duplicate terminal response;
10. stale response after worker restart;
11. timeout before completion;
12. explicit cancellation;
13. startup failure;
14. worker crash/error event;
15. request size bound;
16. response size bound;
17. pending-request-count bound;
18. simulation cannot emit a qualified execution classification;
19. authoritative request does not silently fall back to simulation;
20. terminated worker rejects new work;
21. worker restart creates a new generation;
22. deterministic protocol identity for fixed deterministic fixture;
23. exact-head checkout and immutable qualification subject.

Unit tests may cover pure transition logic, but at least one browser lane must
exercise actual `Worker` message transport.

## Build/CI direction

The first implementation should add a dedicated narrow qualification workflow
rather than relying only on broad repository CI.

Suggested gates:

```text
exact subject checkout
pinned Rust toolchain consistent with current frontend truth lineage
rustfmt
native library tests for pure state/protocol code
wasm32 check
warnings-denied Clippy where target permits
headless browser worker integration tests
checkout immutability
postflight exact-head evidence
```

Queued/unexecuted jobs are not PASS.

## Deferred work

Do not add in MYC-COMPUTE-001A:

- worker pools;
- SharedArrayBuffer/threading;
- rayon;
- WASI;
- GPU/WebGPU compute;
- service-worker orchestration;
- persistent job queues;
- distributed scheduling;
- domain algorithms;
- cross-origin worker execution;
- automatic authority/capability acquisition.

These can be separate children after the simple theorem is qualified.

## Nonclaims

A passing worker substrate does not prove any domain computation is correct,
private, secure, authorized, scientifically valid, or production ready. It proves
only the frozen transport/lifecycle behavior for the tested profile.
