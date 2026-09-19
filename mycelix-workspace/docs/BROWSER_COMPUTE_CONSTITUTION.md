# MYC-COMPUTE-000R — Browser Compute Constitution v0.1

## Purpose

Freeze the first cross-domain Mycelix contract for browser/WASM computation before
introducing a shared compute-worker runtime.

This document is normative architecture only. It does not add a worker crate,
change Leptos runtime behavior, authorize any computation, or qualify any
cryptographic/scientific result.

It stacks conceptually on the frontend truth line culminating in the canonical
`MycelixApplication` composition boundary. The frontend may request computation
and present its status, but neither the UI nor the worker boundary may manufacture
semantic authority.

## Governing theorem

> **The UI does not create truth. A browser worker does not create authority.**
>
> Authoritative semantics belong to qualified domain/core implementations.
> Browser workers are bounded execution adapters that may execute those semantics
> and return evidence-bearing results.

The canonical direction is:

```text
Leptos presentation
      |
      | typed request
      v
browser compute adapter
      |
      | qualified operation/profile
      v
canonical Rust semantics
      |
      | typed outcome + receipt
      v
browser compute adapter
      |
      v
Leptos presentation
```

## Required separations

The architecture MUST preserve all of the following non-equivalences:

```text
UI state                       != computational authority
worker availability            != capability authorization
worker success                 != result validity
WASM execution                 != qualification
local computation              != trusted computation
transport success              != protocol completion
JavaScript object shape        != canonical protocol encoding
simulation output              != production evidence
encrypted computation          != valid input
algorithm output               != scientific evidence
result                         != receipt
receipt                        != authority
historical qualification       != live/current authorization
connection readiness           != signing/effect readiness
```

## Compute profiles

Every consequential browser-compute operation MUST identify an exact versioned
compute profile. Unknown profiles fail closed.

Conceptually:

```text
ComputeProfileRefV1 {
    profile_id,
    revision,
    implementation_identity,
    semantic_contract_identity,
}
```

A profile identifies what semantics are intended to execute. It is not itself a
runtime capability and does not prove that a concrete execution was correct.

## Authority classes

Browser compute implementations MUST distinguish at least:

```text
DevelopmentSimulation
Experimental
ProductionUnqualified
Qualified(profile)
```

The classes are not interchangeable.

A simulation MAY support explicitly non-authoritative visualization/development
flows. It MUST NOT silently satisfy an authoritative request and MUST NOT emit a
receipt that can be confused with a qualified execution receipt.

For authoritative profiles:

```text
worker unavailable -> explicit failure
```

not:

```text
worker unavailable -> silent simulation fallback
```

## Typed request contract

The shared worker substrate should converge on a typed request envelope rather
than stringly typed `action` plus arbitrary `JsValue`/`Reflect` traffic.

Conceptually:

```text
WorkerRequestV1 {
    request_id,
    operation_id,
    profile_ref,
    subject_ref,
    input_commitment,
    deadline,
    payload,
}
```

Qualification-significant fields MUST have deterministic language-neutral
encoding before they become stable commitments, signatures, or receipt identity.
Serde shape alone is not automatically a canonical wire contract.

## Typed outcome contract

Conceptually:

```text
WorkerResponseV1 {
    request_id,
    operation_id,
    profile_ref,
    outcome,
    output_commitment,
    receipt_ref,
}
```

The result model MUST distinguish successful execution, domain rejection,
cancellation, timeout, malformed input, unsupported profile, worker failure, and
indeterminate execution state where applicable.

A generic `ok: true` or one `failed` state is insufficient for consequential
operations.

## Execution lifecycle

The worker substrate MUST represent lifecycle state explicitly enough that the
caller can distinguish at least:

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

Profiles may narrow or extend this vocabulary.

If the caller cannot establish whether execution crossed a consequential commit
boundary, it MUST retain an indeterminate state rather than infer that no effect
occurred.

Pure computations may have no external commit boundary; effectful adapters must
compose the owning effect/authority theorem rather than inventing one here.

## Correlation and replay

Every request/response pair MUST bind an exact correlation identity.

The runtime MUST reject or quarantine, as profile-defined:

- response for an unknown request;
- response with the wrong operation/profile;
- duplicate terminal responses;
- stale response from a previous worker generation;
- replayed request where the operation is non-replayable;
- same request identity with different committed input;
- same output identity with conflicting committed bytes.

A restarted worker does not automatically inherit outstanding authority or
request state.

## Capability boundary

The worker MUST NOT infer permission from:

- the fact that the UI can construct a request;
- a route being visible;
- a worker being installed;
- a profile being recognized;
- a prior successful request;
- a cached positive receipt;
- model/LLM recommendation;
- Holochain/WebSocket connectivity.

Where an operation requires authorization, the exact current authority object or
verifiable bounded authority evidence must be supplied/validated through the
owning authority layer.

The worker substrate itself does not mint capabilities.

## Input truth

Valid transport encoding does not make the input semantically valid.

Each operation owns its exact input-validation theorem. Examples include:

- gradient shape/domain bounds for federated learning;
- privacy-unit/adjacency/contribution-bound validity for DP;
- ciphertext/proof/round binding for secure aggregation;
- canonical amount/rate profiles for finance;
- exact theorem/profile/evidence identity for scientific verification.

```text
well-formed bytes != valid domain input
```

## Output truth

Worker completion proves only that the worker returned a value under the
observed execution path.

It does not by itself prove:

- mathematical correctness;
- cryptographic security;
- differential privacy;
- scientific validity;
- authorization;
- external effect completion;
- reproducibility;
- currentness;
- absence of worker/browser compromise.

Stronger claims require their owning qualified theorem and evidence.

## Receipt discipline

A receipt is an evidence object, not a boolean success badge.

A future generic browser-compute receipt should bind at minimum:

```text
BrowserComputeReceiptV1 {
    request_identity,
    profile_identity,
    implementation_identity,
    input_commitment,
    output_commitment,
    lifecycle_disposition,
    worker_generation,
    execution_context,
    started_at,
    completed_at,
    qualification_reference,
    nonclaims,
}
```

Domain receipts may embed or reference this transport/execution evidence while
remaining responsible for domain semantics.

A browser-compute receipt MUST NOT be deserialized into a positive authority
capability.

## Worker generation and restart

Worker process/generation identity is first-class execution provenance.

A restart MUST NOT silently preserve:

- in-memory pending requests;
- one-shot capabilities;
- secret material whose profile requires re-establishment;
- protocol-round authority;
- privacy-accounting state;
- replay windows/nonces;
- domain state that requires durable reconstruction.

If an operation depends on durable state, that state belongs in the owning
qualified subsystem, not in an unqualified worker heap.

## Secret handling

The shared worker layer should minimize secret exposure and MUST NOT create a
new long-lived secret store merely for convenience.

Profile-specific cryptographic code owns key lifecycle. Where browser-held keys
are permitted, receipts/profiles must distinguish key presence from key authority
and identify the exact key epoch/profile without exporting private key material.

## Randomness

Randomness is a semantic dependency when the algorithm requires it.

The worker API must allow the owning operation to distinguish qualified entropy
sources from deterministic test RNGs.

In particular:

```text
deterministic test randomness != differential-privacy randomness
```

and test reproducibility MUST NOT be represented as production privacy/security.

## Resource bounds

Treat worker messages as untrusted inputs across the execution boundary.

The implementation SHOULD support profile-specific bounds for:

- request bytes;
- response bytes;
- vector/tensor dimensions;
- nesting depth;
- execution time/deadline;
- outstanding request count;
- memory budget;
- decompression/decoding amplification where relevant.

Resource exhaustion is a typed failure, not permission to weaken validation.

## Cancellation and timeout

Cancellation and timeout are explicit lifecycle evidence.

For pure computation, the caller may treat a cancelled/timed-out result as no
usable result unless profile evidence says otherwise.

For effectful computation/adapters:

```text
timeout != definitely not committed
```

and reconciliation must be owned by the effect theorem.

## JavaScript boundary

JavaScript glue should be minimized.

The target design is:

```text
Rust typed value
 -> serde_wasm_bindgen / frozen wire profile
 -> worker transport
 -> Rust typed value
```

rather than application-specific chains of `js_sys::Reflect`, ad hoc property
names, and undocumented object shapes.

A small JS loader may remain necessary for WASM module/bootstrap mechanics; it
must not become a second implementation of authority-bearing algorithms.

## Frontend truth

The presentation layer MAY compress compute state but MUST NOT strengthen it.

Examples:

```text
ProductionUnqualified -> "Experimental", not "Verified"
Indeterminate          -> "Unknown", not "Failed"
WorkerAvailable        -> "Compute available", not "Authorized"
ReceiptPresent         -> "Evidence available", not "Correct"
Simulation             -> visibly simulation/development
```

Color/icon/animation alone must not carry a consequential distinction.

## Cross-domain ownership

The browser compute substrate owns only generic execution plumbing:

- worker lifecycle;
- typed request/response transport;
- correlation;
- timeout/cancellation;
- worker-generation provenance;
- bounded execution-envelope evidence;
- generic error/lifecycle vocabulary.

It MUST NOT absorb domain semantics such as:

- FL aggregation rules;
- DP accounting;
- FHE/secure-aggregation protocol semantics;
- ZKP proof meaning;
- finance arithmetic/settlement;
- governance authority;
- scientific evidence qualification;
- identity validity;
- Symthaea cognitive/scientific claims.

Those remain in their canonical Rust owners.

## Prior art and convergence

Existing Praxis and Symthaea worker bridges demonstrate useful mechanisms such
as `web_sys::Worker`, correlation IDs, broadcasts, and background compute.

They are prior art, not automatically the canonical shared substrate. The new
implementation should converge the useful mechanics while replacing ad hoc
message shapes and application-specific authority assumptions.

## Qualification direction

The first executable child (`MYC-COMPUTE-001A`) should qualify at minimum:

1. typed request/response round trip;
2. deterministic profile/operation binding;
3. unknown profile rejection;
4. malformed payload rejection;
5. correlation-ID mismatch rejection;
6. duplicate terminal response rejection;
7. stale worker-generation response rejection;
8. timeout;
9. cancellation;
10. worker startup failure;
11. worker crash/error event;
12. oversized request/response rejection under a test profile;
13. result/profile substitution rejection;
14. simulation cannot emit a qualified receipt;
15. authoritative request cannot silently fall back to simulation;
16. restart does not silently inherit pending authority/state;
17. fixed input/profile/context produces deterministic protocol identity where
    the owning operation is deterministic;
18. checkout/runtime qualification binds the exact implementation subject.

Browser/e2e qualification should use a real worker boundary rather than unit
functions pretending to be worker transport.

## Deliberate nonclaims

MYC-COMPUTE-000R does **not** establish:

- browser sandbox security;
- WASM memory safety beyond Rust/toolchain properties;
- worker isolation from a compromised origin/page;
- cryptographic security;
- differential privacy;
- secure aggregation;
- scientific validity;
- authorization;
- Holochain integrity;
- external-effect completion;
- performance/SIMD/threading readiness;
- reproducible builds;
- production qualification.

It defines only the boundary that later executable work must preserve.

## Immediate child sequence

```text
MYC-COMPUTE-000R  browser compute constitution
        |
        v
MYC-COMPUTE-001A  typed Rust worker substrate
        |
        v
MYC-COMPUTE-001AQ exact-head/browser qualification
        |
        +--> DP browser adapter
        +--> FL browser adapter
        +--> secure-aggregation browser adapter
        +--> scientific-compute adapter
```

Domain adapters may be developed only after their own canonical semantics are
identified and must not use the shared worker substrate as a reason to duplicate
those semantics in the frontend.
