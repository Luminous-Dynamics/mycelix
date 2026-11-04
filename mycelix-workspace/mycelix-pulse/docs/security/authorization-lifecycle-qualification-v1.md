# Pulse Authorization Lifecycle Qualification v1

**Profile:** `PULSE-AUTH-LIFECYCLE-QUAL-001`  
**Status:** Research corpus; not runtime-qualified  
**Dataset:** Synthetic only; no production effects  
**Created:** 4 November 2026

## Purpose

Qualify authorization as an end-to-end distributed lifecycle, not as a collection of isolated allow/deny unit tests. The corpus forces each observation into six separate planes:

1. **Host decision** — what the Holochain conductor actually authorized or rejected for the exact incoming call.
2. **Domain result** — whether coordinator/application policy executed, rejected or did not run.
3. **Source-chain result** — whether an action was actually authored.
4. **Integrity validation** — whether an independent validator accepted or rejected the authored operation.
5. **Projection state** — whether DHT/UI-derived state is current, stale, conflicting or unknown.
6. **Observer result** — what the caller actually learned, including no acknowledgement and unknown outcome.

These planes are not interchangeable. In particular:

- a DHT record saying `active` is not a host authorization decision;
- a coordinator result is not proof that an operation reached a source chain;
- local source-chain authoring is not proof of remote DHT acceptance;
- a lost response is not proof of denial or non-application;
- a DirectSignal with no acknowledgement is not proof that the recipient rejected it;
- a content digest is not issuer authentication or proof that its contents are true.

## Paradigm shift: authority as a typed, evidence-carrying lifecycle

The proposed lifecycle is:

```text
authority intent
  -> exact grant identity and scope
  -> conductor authorization
  -> domain transition
  -> source-chain action
  -> independent validation
  -> DHT integration / projection
  -> observed outcome
  -> revocation / expiry / migration
  -> recovery and independently verified history
```

The system must preserve an evidence record for each transition. An application capability record is a projection and audit reference; it is not a substitute for a real host CapGrant. A domain policy may independently reject an operation even after the conductor authorizes the zome call.

### Exact identities

Future runtime traces should bind (without exposing secrets):

- DNA and cell identity;
- exact grant Create/Update ActionHash;
- assigned grantee AgentPubKey;
- grant tag and secret fingerprint reference;
- zome and exact function allowlist;
- authority generation / revocation generation;
- runtime and protocol version;
- logical operation ID and separate attempt ID;
- source-chain action hash, validation disposition and projection version where observed.

No raw CapSecret, message plaintext, private payload or unnecessary stable identifier belongs in routine traces.

### Concurrency rule

A call that overlaps revocation must not be assigned an order using wall-clock timestamps alone. Capture the actual conductor decision, durable revoke event and causality evidence. Until the runtime's linearization point is established, classify the ambiguous outcome as `UNKNOWN`; do not retrofit a favorable sequential explanation.

### Retry and recovery rule

A logical operation ID remains stable across retries; attempt IDs change. State-changing effects need durable idempotency/deduplication. The permitted claims are at-most-once logical effect and conditional progress under stated liveness assumptions—not exactly-once network delivery.

### Independent validation rule

For adversarial mailbox transitions, the test coordinator should be deliberately permissive while the production integrity WASM remains unchanged. The test must separately prove:

1. the malicious coordinator authored an action;
2. the receiving validator evaluated that action;
3. the production validator rejected it;
4. the remote observer's canonical mailbox projection did not change.

A coordinator-side error without these observations is insufficient.

## Corpus

The companion JSON contains 27 synthetic scenarios across:

- conductor capability and exact scope;
- revocation and overlapping calls;
- retries, crashes and recovery;
- 0.7.0-to-0.7.1 migration and DirectSignal grants;
- independent validation, roots and fork ambiguity;
- policy mapping and observability.

Each scenario provides an event sequence, explicit happens-before edges where known, expected observations by plane, and assertions. The schema is closed-world: unknown fields fail validation. The corpus includes no raw secrets or personal data.

## Qualification stages

| Stage | Required evidence | Claim ceiling |
|---|---|---|
| 0. Corpus/schema | Schema validation, unique IDs, exact event references, canonical bytes | Corpus integrity only |
| 1. Pure contract | Rust tests for plane separation, identities, Unknown semantics, idempotency key rules | Contract model only |
| 2. Abstract model | Bounded exhaustive interleaving/model check; negative mutants for each invariant | Encoded finite model only |
| 3. Conductor runtime | Real signed AppRequest::CallZome, host grant/revoke, exact function scope, restart tests | Exact runtime/commit and tested bounds |
| 4. Multi-conductor | Independent validators, malicious coordinator, forks, partitions, DHT convergence | Exact topology, workload and faults |
| 5. Independent replay | Second verifier consumes redacted traces and reproduces dispositions/counterexamples | Exact trace and verifier version |

A stage cannot inherit a PASS from a later or unrelated stage. Every receipt binds the exact source SHA and artifact/runtime identities.

## Hard safety gates

These are qualification requirements, not measured results:

- zero unauthorized host accepts across all negative controls;
- zero protected domain effects after confirmed host revocation;
- zero invalid remote updates accepted by the production validator;
- zero duplicate logical state changes for replayed operation IDs;
- zero authority inferred from stale, missing or conflicting projections;
- zero timeout-to-denial or timeout-to-success fabrication;
- zero raw capability secrets in logs, signals, public records or evidence artifacts;
- 100% of test events classified as observed, explicitly unknown, or not applicable; no silent omission;
- all unsupported access semantics reject before any source-chain action.

Finite test counts cannot prove a universal zero-failure rate. Report denominators, confidence bounds, schedule coverage and mutation sensitivity.

## Benchmark protocol

### Workload matrix

Start with:

- agents: 2, 10, 50, 100; 500 stretch;
- active grants: 1, 100, 1,000, 10,000; 100,000 stretch;
- concurrent grant/revoke/call streams: 1, 10, 100;
- source-chain/DHT history: 1,000 to 1,000,000 relevant actions;
- network loss: 0%, 1%, 5%;
- partitions: 0, 30, 120 seconds;
- restarts: before and after each durable lifecycle boundary;
- payloads: empty diagnostic, 1 KiB, 64 KiB and 1 MiB where the operation legitimately carries data.

Run at least 30 independent process-level repetitions per configuration. Publish raw per-run observations and at least 100 observations before reporting within-run percentiles. Separate warm/cold cache, local and remote call paths, and healthy versus faulted topology.

### Metrics

| Metric | Definition |
|---|---|
| Host revocation enforcement latency | Durable conductor revoke completion to first verified host denial of the same grant |
| Projection convergence latency | Host revoke completion to DHT/UI/remote projection showing the matching revocation |
| Authorization latency | Call issue to conductor's observed allow/deny disposition |
| Domain completion latency | Call issue to durable domain outcome, not merely host authorization |
| Retry amplification | Physical attempts / logical operation IDs |
| Duplicate effect rate | Repeated durable effects per logical operation ID |
| Recovery time | Restart to a verified, consistent host/domain disposition |
| Partition safety | Unauthorized accepts during partition; must be zero |
| Unknown accuracy | Unknown outcomes correctly retained rather than fabricated |
| Resource cost | CPU, peak RSS, disk writes/bytes, network bytes and energy if instrumented |

**Do not combine host enforcement freshness and projection knowledge freshness into one “revocation latency.”** The first is a safety property; the second is a convergence property.

### Provisional performance objectives

The following are targets to test, not claims:

- local host revocation denial p95 ≤ 50 ms on a declared LAN;
- healthy-LAN projection convergence p95 ≤ 1 s;
- ≥20% p95 lifecycle improvement over a matched, semantically equivalent baseline;
- zero safety regressions at every tested load and fault level.

Freeze targets after the first exact matched baseline. If the host's documented execution boundary cannot support a meaningful 50 ms measurement, revise the target transparently rather than moving the measurement start point.

## Industry and research comparison

| Comparator | What it demonstrates | Pulse's fair comparison |
|---|---|---|
| Google Zanzibar | Large-scale relationship authorization, external consistency and production latency/availability claims | Do not compete on raw central-service QPS. Compare freshness semantics, offline operation, local sovereignty, exact revocation and topology |
| SpiceDB / OpenFGA | Explicit request consistency modes and cache/freshness trade-offs | Measure the cost of strong freshness vs latency; preserve explicit unknown rather than silently serving stale authority |
| UCAN | Portable cryptographic delegation and attenuation | Compare delegation depth, proof size, offline verification, revocation freshness and key rotation |
| Biscuit | Attenuated bearer authorization with verifier-side revocation identifiers/lists | Compare revocation distribution latency, list cost, offline behaviour and stale-token handling |
| Holochain native capability grants | Conductor-enforced incoming zome-call authorization with grant identity/scope | Prove exact 0.7.1 grant/revoke, assigned caller binding, direct-signal grants and multi-conductor effects |

No cross-system latency claim is valid without matching semantics, hardware, topology, dataset cardinality, cache state, consistency mode, fault assumptions and measurement boundaries.

## Highest-value research questions

1. **Revocation linearization:** What exact event is the enforcement boundary for a concurrent host call and grant deletion? Determine by instrumented runtime traces, not timestamp inference.
2. **Authority epochs:** Can an explicit monotonically increasing authority generation prevent stale claim/projection resurrection across grant updates and 0.7.0→0.7.1 re-issuance?
3. **Fork semantics:** What deterministic, separately authorized protocol resolves competing roots without privileging DHT arrival order?
4. **Evidence minimization:** What is the minimal redacted event record that lets an independent verifier reproduce authorization histories without exposing capability secrets, message metadata or unnecessary identities?
5. **Compositional verification:** Can the Rust history checker, bounded model and Sweettest trace exporter share a frozen event vocabulary while keeping the reference checker independent?
6. **Failure-domain calibration:** When does a remote peer's non-observation indicate delay, partition, validator rejection or application-level denial? Preserve these as distinct outcomes.

## Required next implementation increments

1. Export the six-plane event trace from the real 0.7 canary without recording secrets.
2. Build an independent trace/schema verifier and deterministic canonical receipt.
3. Add the malicious-coordinator/remote-validator Sweettest using production integrity WASM.
4. Instrument host grant revocation and protected call overlap to establish the actual ordering boundary.
5. Port the exact same scenarios to the accepted 0.7.1 tuple only after the release profile and canonical-source parity gate are accepted.
6. Extend #5091's bounded model with authorization grant/revoke/claim states, crash points and unknown outcomes; do not merge delivery and authorization into one state machine.

## Sources and provenance

- Pulse 0.7.0 migration canary: https://github.com/Luminous-Dynamics/mycelix/pull/4891
- Holochain 0.7.1 migration profile: https://github.com/Luminous-Dynamics/mycelix/pull/5718
- Holochain 0.7.1 qualification contract: https://github.com/Luminous-Dynamics/mycelix/issues/5719
- Holochain workspace closure auditor: https://github.com/Luminous-Dynamics/mycelix/issues/5720
- Pulse bounded delivery model-checking work: https://github.com/Luminous-Dynamics/mycelix/issues/5091

This profile is a research specification and synthetic corpus. It does not claim conductor runtime correctness, malicious-validator resistance, cryptographic truth, linearizability, crash consistency, liveness, privacy, production readiness or benchmark superiority. Those claims require the corresponding later qualification stages.
