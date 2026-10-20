# CivOS Multi-Agent Correctness and Resilience Qualification v0.1

**Status:** proposed research contract; unmeasured; not production qualification  
**Contract ID:** `CIVOS-MACQ-001`  
**Source baseline:** `Luminous-Dynamics/mycelix@dbc883566ea937ee8a273c1d33d153cc6262dd9b`  
**Machine-readable scenarios:** [civos-multi-agent-scenarios-v0.json](./civos-multi-agent-scenarios-v0.json)  
**Corpus schema:** [civos-multi-agent-scenarios-v0.schema.json](./civos-multi-agent-scenarios-v0.schema.json)

## Why this work exists

Mycelix has many useful domain tests, and it already contains a shared Sweettest conductor harness. The missing piece is a reusable cross-cluster qualification contract: a way to say exactly which system invariants were exercised, under which peer schedules and failures, against which source revision and runtime tuple.

The current [repository maturity matrix](../../README.md#Maturity-Matrix) itself warns that unit-test counts do not establish multi-agent validation. The Finance Sweettest plan still lists a concurrent credit-limit race as a stretch case, while the separate Finance race audit marks the earlier balance lost-update fixes as addressed through optimistic locking/retry. Those are different properties: avoiding a lost update does not, by itself, prove that two independently authorized requests cannot both pass a stale aggregate-capacity check. Draft [PR #5248](https://github.com/Luminous-Dynamics/mycelix/pull/5248) starts a test-first scenario from two independent provider agents against one shared receiver. It has not been executed, so the race remains a candidate to qualify—not a reproduced exploit. This benchmark turns the gap into an explicit, reproducible question rather than assuming that more tests or a successful compile imply correctness.

This is **not** a claim that the race is exploitable in every deployment, that every scenario applies to every cluster, or that any performance target has been achieved.

## Paradigm shift

Traditional project reporting often foregrounds feature count, unit-test count, raw throughput, or uptime. Those are insufficient for civic systems: a fast system that authorizes the wrong actor, spends the same contribution twice, oversubscribes scarce resources, loses an identity, or presents stale information as current is not successful.

Measure **correctness under adverse schedules first, service outcomes second, and speed third**.

The benchmark uses four separate judgments:

1. **Safety:** did any forbidden or contradictory state become accepted?
2. **Knowledge quality:** did the system distinguish verified, stale, unavailable, conflicting, unknown, and confirmed-empty states?
3. **Recovery and liveness:** did valid work eventually become visible after the documented recovery/synchronization condition?
4. **Efficiency:** what latency, memory, bandwidth, and compute did that correct outcome cost?

A strong performance score cannot compensate for a safety failure. A lack of evidence is not a pass. A timeout is not automatically a safety violation, but it must be reported as a liveness failure if the contract requires eventual completion.

## Scope

Use this contract to qualify critical Mycelix cross-agent and cross-cluster workflows. It is intentionally protocol-neutral; test adapters call the actual implementation and expose a small common observation format.

Required first adapters:

- **Identity / Personal:** actor binding, protected identity posture, recovery and disclosure states.
- **Finance:** idempotency, duplicate IDs, confirmations, concurrent reservations and capacity/credit limits.
- **Commons:** resource requests, claims, assignments and work-completion evidence.
- **Governance:** proposal/vote authorization, stale state, replay and policy version binding.
- **Bridge / Attribution:** cross-cluster caller authorization, event provenance and receipt context binding.

Not every scenario applies to every cluster. Each adapter must declare applicability and the reason. “Not implemented” or “not applicable” is not a pass; report them separately.

This is a benchmark contract only. It does not change production logic, choose a new consensus algorithm, introduce a global transaction coordinator, or claim that Holochain supplies serializable cross-agent transactions. Tests must use the actual chosen conflict-resolution/allocation protocol. Where that protocol cannot establish a unique outcome, the safe answer may be explicit conflict/unknown rather than fabricated certainty.

## Scenario protocol

The canonical cases live in `civos-multi-agent-scenarios-v0.json`; [the JSON Schema](./civos-multi-agent-scenarios-v0.schema.json) specifies the machine-readable contract shape. They cover:

- authorization and caller/actor binding;
- authored-versus-peer-visible state;
- retries and idempotency-key collisions;
- concurrent capacity claims and stale updates;
- unavailable and stale sources;
- consent expiry, revocation and minimum disclosure;
- signed evidence context/replay checks (when an authenticated receipt format is implemented);
- restart, partition and recovery;
- cross-cluster authority boundaries;
- unsupported schema versions and inconsistent identity storage;
- deterministic acceptance/rejection parity.

For each execution, record:

- full source commit SHA and dirty/clean state;
- Holochain, HDK, HDI, conductor, client and harness versions;
- DNA and hApp digests, fixture-corpus SHA-256 and scenario IDs;
- peer count, actors, fault schedule, network conditions, seed and timeout policy;
- action submitted, authoring-conductor result, peer visibility result, validation result, final state and sanitized error category;
- elapsed time, memory/RSS, bytes transferred if measurable, and recovery time;
- expected versus actual outcome, including all counterexamples.

The result should be a machine-readable receipt. A test command returning exit code zero is not sufficient provenance on its own. A queued, skipped, cancelled, or unrelated workflow is not a pass for the tested commit.

## Required test layers

### Layer A — reference-model and mutation tests

Freeze a small state-machine oracle for each scenario. Confirm that known-valid inputs reach the specified state and that mutations (actor, amount, resource, request ID, policy revision, expiry, signer, schema version) flip the expected judgment. Keep oracle fixtures independent from the implementation under test. A hash detects fixture changes; it does not prove the oracle's expected outcome is correct.

### Layer B — real multi-agent Sweettest

Use multiple independently keyed agents and real hApp/DNA bundles. Do not emulate DHT visibility with a mock for cases whose purpose is to test DHT behavior. Distinguish:

- accepted on the authoring conductor;
- published/available to peers;
- discovered through the intended query/link path;
- independently validated by each relevant peer.

Wait for the explicit synchronization condition for eventual-visibility assertions; also test bounded timeout and unavailable outcomes. Avoid relying on arbitrary sleeps as a correctness oracle.

Prefer the maintained Rust/Sweettest path for Holochain integration tests. Holochain's official tooling guidance says Tryorama is no longer officially supported as of Holochain 0.6.1. Existing legacy Tryorama suites can remain useful while they are migrated, but they should not be the only newly developed test path.

### Layer C — deterministic fault schedules

Inject duplicate delivery, reversed ordering, concurrent requests, interrupted connectivity, late synchronization, conductor restart, stale cached observations, revoked/expired grants, malformed payloads, invalid signatures and mismatched schema revisions. Fix random seeds in the release corpus and report any generated fuzz/property tests separately.

### Layer D — deployment comparison

Only after correctness gates pass, compare the real Mycelix path to:
1. the frozen single-process reference model (semantic oracle, not a performance competitor);
2. the previous implementation at a pinned SHA; and
3. a documented conventional-service baseline when a real pilot supplies equivalent semantics and hardware.

Use the same workload, payload sizes, peers/users, data cardinality, failure schedule and hardware. Publish all configurations. A centralized database and a distributed Holochain system have different consistency, network, replication and trust semantics; comparisons must state those differences rather than flattening them into one “faster” number.

## Proposed release gates (not measured)

| Gate | Proposed target | Interpretation |
|---|---|---|
| Critical-case coverage | 100% of declared critical scenarios executed for each release-critical adapter | A skipped test is uncovered, not passed |
| Known-answer integrity | 100% expected acceptance/rejection outcomes on frozen vectors | Report corpus size and versions |
| Unauthorized / invalid evidence | 100% rejection across specified negative controls | Includes unknown trust anchor, invalid signature, context substitution and expiry when relevant |
| Duplicate effect | 0 duplicate logical effects under specified retries, reordered delivery and restart | Same idempotency key with a different payload must reject as conflict |
| Capacity safety | 0 successful final allocations beyond the declared capacity | If serialization is unavailable, unresolved claims must remain explicitly pending/conflicted |
| Source-state semantics | 0 unavailable/stale/conflicting inputs represented as verified-current or confirmed-empty | Verify UI/API output as well as internal enums |
| Recovery | 100% of valid committed actions recover according to the scenario's documented durability boundary | Do not count local pending work as committed |
| Peer visibility | 100% of valid fixtures visible to intended peers within a preregistered bound in the declared test topology | Record the bound and every timeout; this does not prove all topologies converge |
| Reproducibility | Every qualification receipt binds commit, runtime tuple, DNA/hApp digests, corpus digest and seed/schedule | Missing identity fields make the run non-comparable |

## Performance and industry comparison

Do not invent a global latency threshold before measuring the workload. For the first baseline, publish p50/p95/p99 for authoring latency, peer-visibility latency and complete workflow latency; throughput; peak RSS; bytes per successful operation; restart/recovery time; and failed/unknown/conflicted outcomes.

An aspirational optimization hypothesis is **at least 20% lower p95 end-to-end latency or resource use per successful operation on three representative workloads**, with identical semantics and no safety-gate regressions. That is a research target, not an acceptance substitute or a claim of state of the art. Keep each workload's result visible; do not let a weighted aggregate hide a regression.

For any claim of “better than state of the art,” choose a named baseline and publication/revision, replicate its relevant workload where licensing and practicality permit, report hardware/configuration, and compare equivalent semantics. Mycelix should earn differentiation through correctness under partial failure, explicit uncertainty, privacy-preserving evidence, recovery, and measured cost—not architectural vocabulary.

## State-of-the-art and industry comparison

Use external benchmarks for the layer they actually measure; do not collapse unlike guarantees into one leaderboard.

| Comparison layer | Useful reference | What it tells us | What it does not prove |
|---|---|---|---|
| Holochain runtime | [Holochain Wind Tunnel roadmap](https://www.holochain.org/roadmap/) and [0.7 compatibility matrix](https://developer.holochain.org/resources/compatibility/holochain-0.7/) | Runtime behavior/performance scenarios, host metrics and version-appropriate baselines | Correctness of Mycelix's application-level consent, recovery, finance or governance policies |
| Transaction-performance methodology | [Hyperledger Caliper](https://github.com/hyperledger-caliper/caliper) and its [benchmark examples](https://github.com/hyperledger-caliper/caliper-benchmarks) | Workload-driven transaction/read throughput, latency, success rates and resource consumption | A fair cross-platform comparison unless the system connectors and workload semantics are equivalent; current Caliper connector support must be checked before choosing a live comparator |
| Credential interoperability | [W3C Verifiable Credentials Data Model 2.0](https://www.w3.org/TR/2025/REC-vc-data-model-2.0-20250515/) and [OpenID4VCI 1.0 Final](https://openid.net/specs/openid-4-verifiable-credential-issuance-1_0-final.html) | Credential structure, issuer-holder-verifier separation and interoperable issuance | That an issuer is trustworthy, a claim is true, the grant is authorized for this operation, or Mycelix conforms without conformance testing |
| Proof of authenticator possession | [WebAuthn Level 3 Recommendation](https://www.w3.org/TR/webauthn-3/) (25 August 2026) | Origin-scoped public-key credentials and signed authenticator assertions | A drop-in replacement for a Holochain agent key, identity recovery, or the trust policy selecting accepted issuers/authenticators |

### A defensible claim of improvement

For each claim, preregister the workload and a named baseline before measuring. Run both systems on documented hardware with the same actors, payload sizes, data cardinality, retry policy and fault schedule. Report raw samples and uncertainty, not only a mean or composite score. Keep runtime version, DNA/hApp digest, benchmark corpus digest, workload seed and host metrics in the result receipt.

Prioritize these cross-layer metrics:

- **Cost per correct completed operation:** compute, memory, bytes and elapsed time divided by operations that reached the specified valid terminal state. Report rejected, pending, conflicting and unknown outcomes separately.
- **Authorization and lifecycle failure rate:** measure false acceptance and false rejection against frozen known-answer vectors. For safety-critical decisions, any invalid acceptance is a hard-gate failure, not a small performance penalty.
- **Recovery quality:** time to resume valid operations after restart/partition, number of duplicate logical effects, and percentage of cases in which state remains ambiguous.
- **User-visible truthfulness:** proportion of cases where UI/API correctly distinguishes local authoring, peer visibility, confirmed current state, stale, expired, revoked, unavailable and unknown.
- **Interoperability:** independent verifier agreement on the same versioned vectors, including unsupported algorithms/schema versions and trust-policy mismatch.

The first competitive claim should be narrow, for example: *on three preregistered Mycelix civic workflows, the candidate reduces p95 cost per correct completed operation by at least 20% relative to a pinned Mycelix baseline, while passing all safety gates*. That threshold is a hypothesis, not a current result. Any external “better than SOTA” claim must name the system/version, match the semantics that can be matched, and publish the failure cases as well as successes.

## Runtime and upgrade discipline

Record the exact runtime tuple per test. The official [Holochain 0.7 compatibility table](https://developer.holochain.org/resources/compatibility/holochain-0.7/) recommends the 0.7 line, but the [0.6 → 0.7 upgrade guide](https://developer.holochain.org/resources/upgrade/upgrade-holochain-0.7/) documents action-model changes, transport changes, changed DNA hashes, renamed databases, and no direct database migration path. Do not globally bump the workspace merely to make versions match; qualify each cluster's behavior-preserving migration separately and explicitly plan user-data continuity.

For credentials and status, use the [W3C Verifiable Credentials Data Model 2.0 Recommendation](https://www.w3.org/TR/2025/REC-vc-data-model-2.0-20250515/) as an interoperability reference where it fits. Reuse its ecosystem concepts, but do not claim VC conformance merely because an internal Mycelix receipt carries similar fields.

## Execution sequence

1. Validate the corpus, schema and traceability manifest with the Rust validator in `tools/civos-benchmark-validator/`; freeze the corpus, schema and coverage digests, and retain the corpus-validation receipt. This validates the specification and maps candidate tests, not system behavior.
2. Review `civos-multi-agent-coverage-v0.json`: v0.1.0 currently marks 18 of 21 scenarios `unmapped` and 3 `candidate_partial`. These are planning classifications, not scenario pass rates.
3. Convert each unmapped scenario into an executable test with an independent negative control and an exact source/test reference. Any partial mapping must state remaining gaps; do not promote it based on test-count increases alone.
4. Execute and review the candidate Finance cross-agent receiver-limit regression [PR #5248](https://github.com/Luminous-Dynamics/mycelix/pull/5248), then add the highest-risk idempotency and Personal identity/source-state cases to the existing test infrastructure.
5. Run exact-head tests locally where the pinned runtime is available; record precise blockers rather than waiting on hosted runners.
6. Add fault-injection and restart cases; qualify the baseline with reproducible receipts containing exact runtime, source head, scenario digest, coverage digest, test command, seed, negative-control result, and before/after state digests.
7. Only after the required behavior scenarios pass may performance comparisons be published; only after controlled deployments may real-world outcome claims be made.

## Nonclaims

This document does not claim that these tests have been implemented or run, that production behavior passes the targets, that Holochain offers global serializability/BFT for arbitrary business operations, that cryptographic authenticity proves real-world truth, or that a passing synthetic corpus proves civic impact. Those require separate evidence.
