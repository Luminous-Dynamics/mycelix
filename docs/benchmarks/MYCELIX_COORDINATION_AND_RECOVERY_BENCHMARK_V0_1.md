# Mycelix Coordination and Recovery Benchmark (MCRB) v0.1

**Status:** Proposed benchmark contract; no benchmark results are claimed  
**Tracking:** [PLAT-STEWARD-001](https://github.com/Luminous-Dynamics/mycelix/issues/6322)  
**Related:** #3617, #5719, #5718, #5913, #5914

## 1. Purpose

MCRB measures whether a decentralized coordination system can preserve authority, evidence, privacy and recoverability while performing useful work under real network conditions.

It is not a TPS contest. Mycelix must not claim superiority because a different system has a lower transaction rate under weaker consistency, replication, identity or privacy semantics.

The benchmark has two separate questions:

1. **Correctness envelope:** does the system preserve the required invariants?
2. **Performance envelope:** within the correctness envelope, what latency, throughput, resource and operational costs does it achieve?

A performance result is invalid when its corresponding correctness controls fail.

## 2. Compared systems and fairness rules

### Required Mycelix/Holochain comparison

- Holochain 0.6.1: current repository-generation baseline where an exact reproducible runtime can be assembled.
- Holochain 0.7.1: migration candidate after source, Cargo, Nix, client and lockfile closure.
- Both must run the same logical application contract, data distribution, validation semantics and workload seed wherever the APIs allow.

Do not reuse a 0.6 conductor database for 0.7.1. Treat export/import and fresh-conductor deployment as a separate migration experiment.

### Conventional baselines

- PostgreSQL with a single-writer service and an append-only outbox/event projection is a conventional centralized authority baseline for CRUD and coordinated workflows.
- Hyperledger Fabric, using a version-matched Hyperledger Caliper workload where appropriate, is a permissioned-ledger baseline for comparable multi-organization endorsement/commit workflows.
- IPFS/libp2p or an Iroh-only test is a networking/storage component comparison, not a direct application-authority competitor unless the same validation, authorization, privacy and recovery semantics are implemented above it.

Caliper's documented metrics—throughput, percentile latency and resource use—are useful common reporting conventions. They do not make different finality or validation models semantically equivalent.

### Mandatory fairness controls

Pin and publish:

- hardware model, CPU allocation, memory, storage, NIC, OS and kernel;
- region and network topology, RTT distribution, packet loss and bandwidth limits;
- conductor/peer counts, replication conditions, data sizes and record distributions;
- exact source revisions, dependency locks, build flags and runtime versions;
- validation, endorsement, consistency and authorization rules;
- client concurrency, arrival-rate controller, warm-up, measurement and cooldown;
- query mix, write mix, key distribution and deterministic random seed;
- failure schedule and recovery protocol.

Report local-LAN, constrained-WAN and partitioned-network conditions separately. Never merge their results into one headline number.

## 3. Workload model

Use a synthetic community coordination application with the following record types:

- peer-authored immutable declarations;
- append-only corrections/revisions;
- scoped capability/consent grants and revocations;
- read projections with explicit source and freshness markers;
- cross-domain workflow intents;
- externally executed effect receipts;
- recovery and migration checkpoints.

Use the same schema, data and operation mix across comparable implementations. Keep authority and projection layers explicit.

## 4. Scenario suite

| ID | Scenario | Primary question |
|---|---|---|
| MCRB-01 | Single-agent append and read | What is local write/read cost and integrity behavior? |
| MCRB-02 | Multi-agent publication | How do writes integrate and become visible to expected peers? |
| MCRB-03 | DHT convergence | How long until each expected peer can retrieve the valid record? |
| MCRB-04 | Cross-role workflow | What is the cost and correctness of a multi-DNA OtherRole workflow? |
| MCRB-05 | Partition and rejoin | Are valid records reconciled without silently overwriting or fabricating state? |
| MCRB-06 | Capability revocation | When does revoked authority stop producing successful protected effects? |
| MCRB-07 | DirectSignal security | Are sender identity, grant scope, receiver behavior and incompatible wire versions handled correctly? |
| MCRB-08 | Duplicate/replay and uncertain outcomes | Are retries idempotent or reconciled before another mutation? |
| MCRB-09 | Source-chain recovery | Can the expected chain be restored with correct key identity and explicit state transitions? |
| MCRB-10 | Application export/import | Are domain records reconstructed without conflating action hashes, DNA hashes or external legal effect? |
| MCRB-11 | Private/public boundary | Can private source-chain data be kept out of public DHT and projection paths? |
| MCRB-12 | Adversarial peers | Do invalid, stale, revoked, wrong-scope and equivocal claims fail closed? |
| MCRB-13 | Resource exhaustion | Do rate limits and bounded work prevent unbounded amplification? |
| MCRB-14 | Deployment profile composition | Do unified and resilience profiles contain the required roles and no undeclared targets? |

Every scenario must define setup, preconditions, actors, operation trace, expected invariants, failure injection, completion criterion, evidence artifacts and verifier.

## 5. Correctness invariants

These are hard gates, not performance aspirations.

1. **No false authorization:** zero unauthorized protected effects in the frozen deterministic negative-control corpus.
2. **Revocation barrier:** after the system reports revocation as effective under the tested policy, no subsequent protected effect may be authorized by that revoked grant. Measure the barrier explicitly; do not hide propagation delay.
3. **No invalid history rewrite:** corrections append new revisions; they do not mutate historical claims in place.
4. **Unknown is not success:** unknown outcome requires reconciliation before retry or a new mutation.
5. **Idempotent effect handling:** duplicate/replayed requests cannot create duplicate external effects under the defined idempotency key and receipt contract.
6. **Private-data containment:** source-chain-only data never becomes public through gossip, signals, logs, analytics or projections.
7. **Provenance integrity:** every qualified result binds to the actual signed action/author and exact evidence artifact.
8. **Recovery integrity:** all records promised by the recovery contract are restored or reported as explicitly unavailable; no silent loss.
9. **Topology integrity:** every same-hApp OtherRole call resolves to a declared role in the tested profile; separately installed hApps use explicit orchestration.
10. **No false qualification:** mock, fixture, source-only, compile-only, skipped, queued or stale-head evidence cannot satisfy runtime qualification.

For deterministic frozen fixtures, the acceptance target is 100% agreement with an independent oracle. A single false authorization or private-data leak fails the correctness gate regardless of throughput.

## 6. Metrics

### User-visible performance

- operation success rate and qualified operations per second;
- p50, p95 and p99 end-to-end latency;
- read freshness and stale-read rate;
- time to peer discovery and first valid remote read;
- DHT convergence latency distribution;
- signal delivery rate and latency;
- revocation observation and enforcement latency;
- cross-role workflow completion latency.

### Resilience and recovery

- recovery point objective and recovery time objective, measured by data class;
- acknowledged-record loss count;
- time to rejoin and regain expected network coverage;
- fraction of expected records restored;
- convergence after partition/rejoin;
- rate of unresolved/ambiguous operation outcomes;
- time to reconcile uncertain external effects;
- backup, export, import and restore duration.

### Resource and economics

- CPU-seconds and peak CPU;
- peak and steady-state RSS;
- disk bytes per accepted record and database growth;
- network bytes per successful operation and per convergence event;
- energy per qualified operation when instrumented;
- storage and bandwidth cost per million records;
- operator-hours per deployment, upgrade and recovery;
- cost per thousand successful qualified workflows.

### Correctness and assurance

- false-authorize count;
- missed-revocation count;
- duplicate-effect count;
- private-data leakage count;
- invalid-history acceptance count;
- false-positive fixture/mock qualification count;
- oracle mismatch count;
- percentage of runs with complete, reproducible evidence receipts.

## 7. Scale ladder

Use scale as a test dimension, not a claim about supported capacity:

| Tier | Participants | Purpose |
|---|---:|---|
| Smoke | 3–5 | deterministic behavior and packaging |
| Development | 20 | routine integration and failure injection |
| Qualification | 100 | multi-peer convergence, churn and resource envelope |
| Stress | 500+ | exploratory scale, bottleneck and failure characterization |

A tier is only considered completed when the actual topology, peer count and duration appear in the raw result bundle. Simulated peers do not count as real conductors.

For each tier, test steady-state, burst, churn, partition/rejoin and cold-start conditions. Record whether the workload is a real conductor network, a simulator, or a mixed test.

## 8. Targets and decision rules

Do not invent a universal DHT convergence SLO before measuring the supported topology. Establish the first exact 0.6.1 baseline, then run 0.7.1 under the identical workload and hardware.

Proposed qualification policy:

- Correctness: 100% pass on frozen deterministic controls; zero security-invariant violations.
- Reproducibility: every headline result has pinned source, locks, toolchain, topology, workload seed and raw traces.
- Migration: no 0.7.1 promotion while dependency closure, fresh-conductor runtime, multi-agent tests, client E2E or restore/export semantics remain unqualified.
- Performance: report p50/p95/p99 and confidence intervals; compare like-for-like workloads; disclose regressions instead of averaging them away.
- Improvement claim: require a measured Pareto improvement in at least one declared user-facing metric without worsening hard correctness invariants or materially regressing another declared metric.
- Industry claim: compare only equivalent semantics; label non-equivalent comparisons as component or architectural comparisons.

Candidate hypotheses to test—not achieved results:

- 0.7.1 should not materially regress p95/p99 workflow latency or resource-per-qualified-operation versus the 0.6.1 baseline under matched conditions.
- Local-first workflows should remain usable during temporary peer unavailability where the application policy permits offline writes.
- Recovery and migration should become routine, measurable operations rather than bespoke operator exercises.
- The architecture registry should detect undeclared roles and unresolved OtherRole targets before a bundle is promoted.

Numeric SLOs should be set only after the first baselines identify actual workloads, user expectations and operating envelopes.

## 9. Evidence receipt

Each run should emit a content-addressed receipt containing:

- benchmark ID and version;
- source repository, exact 40-character SHA and tree digest;
- Cargo, JavaScript and Nix lock digests;
- conductor, client, Lair, Kitsune2 and tool versions;
- hardware and network profile;
- topology/profile and exact DNA/hApp hashes;
- workload source, scenario, seed and parameters;
- run start/end, peer lifecycle and failure schedule;
- raw traces, logs, metrics and artifact digests;
- oracle/verifier implementation and version;
- scenario-level outcomes and uncertainty;
- measured, modelled, proposed and unexecuted status distinctions.

A verifier must reject incomplete provenance, mismatched artifact digests, missing required scenarios, stale source heads and results whose runtime identity does not match the declared profile.

## 10. Relationship to existing work

MCRB is not a replacement for:

- #5719 / #5718: Holochain 0.7.1 target and migration qualification;
- #3617: repository consumer qualification matrix;
- #5913: Shared Data Substrate Qualification across Holochain, PostgreSQL, vector retrieval and analytical storage;
- #5914: substrate pin reconciliation;
- application-specific assurance programmes for Finance, Property, Food, Civic, Identity and Pulse.

MCRB supplies a common coordination/resilience workload and reporting contract. Application-specific tests remain responsible for domain semantics and legal/safety boundaries.

## 11. Claim ceiling

This document defines a proposed benchmark. It contains no measured Mycelix result, no industry ranking, no proof of superior throughput, no production readiness claim and no Holochain 0.7.1 runtime qualification.


## 12. Reuse Holochain Wind Tunnel as the runtime measurement layer

MCRB should not recreate a second peer-network performance lab.

Holochain's published Wind Tunnel work already describes scenarios for app installation, zome calls, DHT synchronization, validation receipts, remote signals and different validation/topology configurations. Its published 2026 status also describes geographically distributed test machines and host metrics such as CPU, memory and disk I/O. The public Wind Tunnel report format includes a DHT sync-lag scenario that measures the interval between a peer publishing a record and other peers observing it.

Use a two-layer design:

### Layer A — substrate/runtime measurements

Consume or reproduce the relevant Wind Tunnel scenarios for the exact Holochain version under test:

- peer discovery and connectivity;
- DHT sync lag and remote visibility;
- zome-call and app-install latency;
- validation-receipt workflows;
- remote/direct signal transport and delivery;
- validation load, resource consumption and churn.

Keep the upstream run ID, scenario version, peer topology, runtime version and raw artifact digests. Do not compare a 0.6.1 historical Wind Tunnel report with a 0.7.1 run as if it were a matched experiment; rerun the same scenario definition on both exact runtime generations.

### Layer B — Mycelix application semantics

Add Mycelix-specific behavior around the runtime events:

- did the expected source author and capability authorize this operation?
- did a stale, expired, revoked or wrong-scope grant ever authorize a protected effect?
- did all expected agents observe the same eligible records after convergence?
- did duplicate/replayed requests cause more than one external effect?
- did unknown outcomes trigger reconciliation rather than blind retry?
- did private source-chain data remain private across gossip, signals, logs and projections?
- could records and workflow state be recovered with correct identity and provenance?
- did every reported PASS bind to the same runtime, bundle and source head?

This is where Mycelix can contribute something beyond a raw substrate performance report: a reproducible measure of **qualified coordination**, not just network activity.

References for harness authors:

- Holochain Wind Tunnel overview: https://blog.holochain.org/wind-tunnel-seeing-inside-the-storm/
- Example public DHT sync-lag report: https://holochain.github.io/wind-tunnel/run.20260212014621.b34bbc7f77fcf7dc00c385e4b6b5d5d015b4e55f.html
- Holochain 0.7.1 upgrade guide and DirectSignal/grant requirements: https://developer.holochain.org/resources/upgrade/upgrade-holochain-0.7/

## 13. Industry comparison: compare the right semantic layer

The benchmark should publish a comparison matrix rather than one vendor leaderboard.

| Comparator | What it measures well | Valid Mycelix comparison | What it does not prove |
|---|---|---|---|
| Holochain Wind Tunnel | DHT, peer, zome, validation, signal and host operating envelope | Direct substrate-level comparison across exact runtime versions | Application authorization, revocation correctness, private-data containment or business-effect uniqueness |
| PostgreSQL 18 + outbox | Centralized ACID transaction/query baseline; pgbench supports built-in and custom SQL workloads | Same application workflow implemented with one authoritative transactional service and a replayable outbox | Peer-authored autonomy or decentralized validation |
| Hyperledger Fabric 2.x + Caliper | Permissioned-ledger workload benchmarking; success rate, throughput, latency and resource use | Multi-organization endorsement/commit workflows with matched policy and payload size | Holochain's agent-centric validation semantics or offline-first operation |
| Yjs / Automerge | Collaborative CRDT data convergence and conflict-free merges | Document-editing and merge behavior, offline editing and sync overhead | Identity authority, legal consent, transaction finality or external effect execution |
| Iroh / IPFS | Transport or content-addressed dissemination components | Blob transfer, relay/path quality and content retrieval | Application-level authorization, DHT validation or qualified state transitions |

PostgreSQL comparison should use custom transaction scripts as well as the default pgbench workloads; the default pgbench scenario is TPC-B-like and its documentation warns against treating replaying the exact same run as a general statistical method. Use independent randomized repetitions for performance estimates while preserving a seed for bug reproduction.

Hyperledger Caliper's current README lists Fabric v2.X as its supported blockchain connector and success rate, read/write throughput, latency and resource consumption as performance indicators. Record that version and connector limitation in the comparison report. Caliper is a useful methodology and a Fabric baseline where semantics are matched, not proof that Caliper supports every ledger or that the Fabric and Holochain models are equivalent.

Yjs/Automerge comparisons are component-level CRDT comparisons. They are useful for collaboration and offline merge workloads, but must not be scored as a substitute for cryptographic authorship, capability enforcement, revocation or irreversible side effects.

## 14. Statistical and publication protocol

- Run at least 5 independent repetitions for smoke/qualification comparisons and preferably 10 or more for headline performance claims; report the actual count, median, dispersion and confidence interval.
- Randomize run order or use balanced ordering to reduce warm-cache, thermal and background-load bias.
- Preserve per-operation samples or a documented sampling policy; report tail latency rather than only the average.
- Separate offered load from achieved throughput. Record scheduled arrival rate, client concurrency, skipped requests, retries and backpressure.
- Publish the workload code, raw traces or an access-controlled reproducibility package, manifest hashes and analysis scripts.
- Use paired comparisons only when the scenarios, payload, topology, hardware, load controller and correctness contract are sufficiently equivalent.
- Never average away a correctness failure, peer-specific failure, partition condition or low-connectivity cohort.
- Publish confidence intervals and per-peer distributions when aggregation could hide an outlier, slow peer or unfair impact.

### Candidate performance gate after baseline

Once the exact 0.6.1 baseline exists, define a bounded migration non-regression gate for each workload, not a single aggregate score. A reasonable **proposed decision threshold** for routine workflows is to require 0.7.1 p95 latency and resource-per-qualified-operation to remain within 10% of the matched 0.6.1 baseline, unless a deliberate trade-off is documented and approved. This is a review threshold to test, not an existing result or a promise that 0.7.1 will meet it.

For new capabilities where no baseline exists, publish the first measured operating envelope, then set a user-facing SLO from actual workflow needs. Do not manufacture universal DHT convergence or recovery SLOs from a small synthetic lab.



## 15. Metric integrity is a benchmark gate

A benchmark result is itself an evidence object. The harness must qualify the measurements before interpreting performance.

A published Holochain Wind Tunnel report from 12 February 2026 illustrates why. Its DHT Sync Lag scenario ran with 10 peers on Holochain 0.6.1-rc.0 and Wind Tunnel 0.6.0. The report shows an all-agent sync-lag mean of approximately 7.81e29 seconds, while some individual peer means are tens to hundreds of seconds and two peer means are approximately 3.46e29 and 3.56e30 seconds. The same scenario reports two errors. The public report does not explain the cause; MCRB must preserve that uncertainty rather than guessing.

This run is a **measurement-quality example, not a performance baseline or target**. A summary statistic with impossible values must not be used to rank or compare systems.

Required metric-integrity checks:

- Record the clock source and synchronization method for every host.
- Measure local durations with monotonic clocks; use wall-clock timestamps only for cross-host correlation with recorded offset and uncertainty.
- Reject negative, non-finite, overflowed or implausibly out-of-run durations.
- Preserve raw outliers and classify them; never silently trim, clamp, winsorize or average them away.
- Publish peer-level distributions before any aggregate, and reject an aggregate if one or more member samples fail integrity checks.
- Verify send/receive counters, sample counts, denominators and missing-peer coverage reconcile.
- Mark the whole benchmark run as measurement_invalid when time/metric integrity fails; it cannot support a performance claim even if the application correctness tests passed.

The benchmark verifier should distinguish:

- application_fail — an application invariant failed;
- measurement_invalid — the measurement evidence is malformed or insufficient;
- infrastructure_incomplete — required peers/jobs/artifacts did not complete;
- qualified — both application correctness and measurement integrity passed.

Never collapse those states into a generic failed/passed boolean.



## 16. Canonical executable artefacts

The machine-readable scenario contract is `MCRB_V0_1_SCENARIO_MANIFEST.json`; the corresponding run-receipt contract is `MCRB_V0_1_RUN_RECEIPT.schema.json`. These files distinguish proposed scenarios from executed evidence. A run may be marked executed-pass only when both the application correctness gates and the new metric-integrity receipt fields qualify. The scenario manifest explicitly reuses Holochain Wind Tunnel for substrate measurements and reserves the MCRB layer for Mycelix application semantics.



## 17. Executable metric-integrity verifier

The dependency-free Rust crate at `mycelix-workspace/tools/mcrb-metric-integrity` is the first executable component of MCRB's measurement-assurance layer. Its `validate_batch` API has three distinct outcomes: `Valid`, `Invalid` and `Incomplete`.

It enforces workload-declared duration limits; monotonic local clocks; named clock sources and synchronization methods; host offset/uncertainty bounds; expected peer coverage; unique operation IDs; and publication/observation counter reconciliation. It reports global and per-peer p50/p95/p99 only when the batch is fully valid and complete. Outliers inside the declared workload window are retained and exposed by sample index; the library never trims or rewrites the raw sample list.

Unit tests are authored for valid distributions, preserved outliers, NaN/negative/out-of-window durations, missing peers, excessive clock uncertainty, counter overflow/shortfall, wall-clock duration misuse, duplicate operation IDs and empty runs. The exact-path Rust workflow is part of this PR. **The code has not yet been compiled or tested by a runner**; the source is a candidate verifier, not a qualified benchmark component.



The verifier treats an operation identifier as unique **within an observer peer**, not globally: the same logical publication may legitimately be observed by several peers, while duplicate samples from the same peer are rejected. Both cases have dedicated unit-test fixtures.


## 18. Qualified operations per second (QOPS)

MCRB's headline application metric should be **qualified operations per second**, not raw transactions per second.

Let:

- (N_q) be the number of attempted operations that completed successfully and have a verified receipt binding the exact operation, authorized principal/capability, source record/action, required peer-observation condition and (where applicable) external effect receipt;
- (T) be the fixed measurement-window duration in seconds.

[
QOPS = \frac{N_q}{T}
]

A QOPS value is **ineligible**, not merely lower, when a hard correctness invariant fails or measurement integrity is invalid. It must never be computed only from successful responses while ignoring rejected, timed-out, skipped or unknown operations.

Every QOPS report must also publish:

- offered operations per second and total offered operations;
- total attempts, retries and backpressure;
- qualified success fraction (N_q / N_{offered});
- correct authorization denials and unexpected denials;
- unknown outcomes and reconciliation duration;
- duplicate external effects and acknowledged-record loss;
- p50/p95/p99 latency for both all attempts and qualified completions.

Expected denials are not counted as successful business operations; they are reported as a separate correctness outcome. This prevents a system from inflating QOPS by refusing difficult operations, silently dropping attempts or making authorization too permissive.

This is a proposed MCRB metric, not an established industry standard or measured Mycelix advantage. Publish it alongside conventional throughput and latency metrics so other systems can reproduce the same contract.



## 15. Qualified Operations Per Second (QOPS)

MCRB's headline application metric is **Qualified Operations Per Second (QOPS)**, not raw transactions per second.

For a declared measurement window:

`QOPS = count(operations satisfying the scenario's complete qualification contract) / measured_window_seconds`

An operation qualifies only when its scenario-specific contract is satisfied. The contract can require:

- authenticated source and correct authority for the requested operation;
- integrity validation and exact provenance binding;
- the required peer-visibility/convergence threshold, when the operation requires remote visibility;
- an independently verifiable external-effect receipt, when an external side effect is part of the operation;
- no duplicate effect, replay, unresolved timeout or ambiguous outcome;
- the declared freshness/currentness and privacy conditions.

Do not require remote convergence for an operation whose declared policy is intentionally local/offline. Record the policy and class separately so local-first operation is not mistaken for a missing remote dependency.

Every offered operation must receive an outcome classification:

- `qualified_success` — complete contract satisfied; eligible for QOPS;
- `correct_denial` — request correctly denied by policy; not a QOPS success, but not a system failure;
- `unexpected_denial` — valid requested operation denied unexpectedly;
- `failed` — a known operation failure;
- `unknown` — outcome cannot yet be established; reconciliation required;
- `duplicate_effect` — the request created a duplicate external effect; hard failure.

Publish QOPS together with offered operations/s, qualified-success fraction, correct denials, unexpected denials, known failures, unresolved unknowns and duplicate effects. This prevents a system from appearing fast by dropping work, denying everything or counting a submitted zome call before the intended effect is proven.

QOPS is **not directly comparable** across implementations until their per-scenario qualification contracts are equivalent. For a centralized PostgreSQL or permissioned Fabric implementation, implement the same authorization, provenance, effect-receipt and recovery contract as far as that platform permits; document any non-equivalent guarantee.

## 16. Wind Tunnel compatibility is a qualification gate, not an assumption

Wind Tunnel is the appropriate upstream substrate-load-testing reference and should be reused where the exact toolchain is compatible. However, MCRB must not infer Holochain 0.7.1 support from the Wind Tunnel project's own version number.

At the time this benchmark was reviewed (19 November 2026):

- Holochain's roadmap reports Holochain 0.7.1 as the released latest 0.7 runtime, and the 5 November Dev Pulse says its tools, libraries and documentation were updated/tested and ready for development.
- The public Wind Tunnel repository's published compatibility table lists Wind Tunnel v0.7.1 against Holochain 0.6.3 / Kitsune2 0.4.1. It also lists pinned Holochain 0.7.0 / 30-node reports, but that does not establish a supported Wind Tunnel release pairing for Holochain 0.7.1.
- Holochain's 17 November roadmap still records the Wind Tunnel-for-0.7 workstream as in progress (77%, 23/30 points), with analysis/core-scenario work not complete in the visible snapshot.

Therefore:

1. Treat Wind Tunnel as the source of scenario design, instrument patterns and runtime telemetry—not an assumed drop-in runner for Holochain 0.7.1.
2. Record the exact Wind Tunnel commit/release, Holochain binary SHA/version, Kitsune2, Nix inputs and scenario artifacts for every run.
3. Before using it for 0.7.1, demonstrate a clean build and a minimal 0.7.1 run (app install, zome call, DHT sync and remote signal) on a fresh conductor. A 0.7.0 run or a Wind Tunnel v0.7.1 release tag is not enough.
4. If upstream compatibility is not yet available, maintain a small Mycelix-owned adapter/port with a versioned compatibility patch and upstream contribution plan. Keep it separate from the application semantic verifier.
5. Do not merge 0.6.3, 0.7.0 and 0.7.1 measurements into one trend line without separate runtime labels and matched scenario semantics.

The MCRB runtime adapter must be an independently qualified integration, with an explicit `unsupported`, `source-audited`, `build-verified`, `smoke-run` or `qualified` status.

## 17. Current comparator versions and caveats

- **PostgreSQL:** the ADR's PostgreSQL 18.6 baseline remains useful as a frozen historical comparison. PostgreSQL 18.7 was released on 12 November 2026; add it as the current 18.x patch candidate and record exact image/binary digest. Use custom `pgbench` scripts for the actual workflow; the built-in TPC-B-like workload is not a substitute for the Mycelix coordination contract.
- **Hyperledger Fabric / Caliper:** pin Caliper and its Fabric connector/SDK precisely. Current Caliper project documentation lists Fabric v2.x support, but marks the Network SDK and Gateway SDK integrations as deprecated. Do not treat a passing Caliper benchmark as proof of equivalent endorsement/finality/privacy semantics.
- **CRDT collaboration:** Yjs and Automerge remain component comparisons for offline document editing, merge behavior and convergence, not direct authority/revocation/effect-finality competitors.
- **Transport/content addressing:** Iroh, libp2p and IPFS can benchmark transport, relay and content distribution costs, not Mycelix application authority on their own.

Version freshness is part of benchmark validity: a comparator profile must state whether it is a frozen historical baseline or a current supported candidate. Do not silently upgrade one side of a matched experiment.



## 18. Measurement-integrity claim ceiling — MCRB-TIME-001

The current `mycelix-mcrb-metric-integrity` implementation is an intermediate structural gate, not yet an independent clock-evidence verifier.

Its current `DurationSample` contains a caller-supplied duration value, and its `ClockObservation` contains caller-supplied clock metadata. It cannot yet recompute elapsed time from raw monotonic start/end ticks, prove a clock epoch remained stable within the sample, or independently pair a publish event with the first valid remote-observation event by record digest.

Until [MCRB-TIME-001 #6335](https://github.com/Luminous-Dynamics/mycelix/issues/6335) is implemented and exact-head qualified:

- global/per-peer percentiles may be used only for local sample distributions under the existing basic integrity contract;
- do not call the clock metadata independently attested;
- do not claim independently qualified cross-host DHT convergence latency from caller-supplied duration values;
- cross-host latency must include lower/upper uncertainty bounds derived from the source and observation hosts' calibration;
- if the uncertainty interval is wider than the predeclared tolerance, classify the measurement invalid/incomplete and withhold the point-latency claim.

MCRB-16 now includes raw monotonic tick-pair, clock-epoch, matched event/record-digest and uncertainty-interval controls. The run-receipt schema requires the corresponding integrity fields. This is a deliberate claim ceiling, not a reason to wait before doing source-level work on the other MCRB scenarios.



## 19. Local tick-derived timing implementation update

The MCRB metric-integrity candidate now represents local duration samples as raw monotonic start/end ticks, clock ID and clock epoch. The verifier derives duration from the tick delta and the declared nonzero tick frequency; it no longer accepts a caller-supplied `duration_ms` field as the value used for percentile aggregation.

The authored tests now cover valid derived distributions, outlier preservation, reversed ticks, clock-ID mismatch, clock-epoch mismatch, zero tick frequency, derived duration outside the declared window, missing peer samples, clock uncertainty metadata, counter overflow/shortfall, wall-clock declaration, duplicate peer/operation samples, cross-peer observation of the same logical operation and empty batches.

This is a source-level increment only. The exact branch head has not yet passed Cargo tests or Clippy. Cross-host DHT convergence remains **not qualified**: the verifier still needs matched publish/first-valid-observation events, record-digest binding, calibration validity and uncertainty intervals from both hosts under #6335.

Do not claim the 13 authored tests passed until the exact-head workflow proves it.


## 20. Cross-host uncertainty-interval primitive

The metric-integrity candidate now includes a separate `cross_host` calculation primitive. It accepts a publisher/observer event pair bound to the same operation ID and record digest, along with separate calibration records for each peer.

The calculation corrects each host's local wall-clock event timestamp by its signed offset, adds clock uncertainty conservatively, verifies each event falls within its calibration validity window, and returns a non-negative latency interval. A point estimate is withheld when the corrected center is negative or combined uncertainty exceeds the predeclared tolerance.

This is a useful step beyond subtracting two uncorrected timestamps, but it is not yet a full cross-host qualification path. The pure helper is not yet wired into `MeasurementBatch` or the canonical run receipt; calibration/event records are still caller-supplied and have not been bound to signed raw artifacts, and pair uniqueness/completeness still need integration. MCRB-16 and the cross-host latency claim remain unqualified until that integration and exact-head tests pass.


## 15. Holochain 0.7.1 runtime regression gate (observed 19 November 2026)

Holochain 0.7.1 is the latest released 0.7 runtime and was declared ready for development by upstream on 5 November. That is not equivalent to Mycelix production qualification. The public Holochain issue tracker currently contains concrete reports that are relevant to Mycelix workloads:

| Upstream reference | Reported behavior | Mycelix consequence | MCRB gate |
|---|---|---|---|
| [#6071](https://github.com/holochain/holochain/issues/6071) | Multi-function capability grants reportedly admitted calls after revoke in 4/6 rounds on 0.7.1; single-function grants denied in 3–21 ms | Consent withdrawal, staff-session termination and other protected actions may remain authorized after revocation | MCRB-17; critical security blocker |
| [#6062](https://github.com/holochain/holochain/issues/6062) | Large gossip offer over about 10,900 op IDs reportedly fails with SQLite variable limit and can stall convergence; 1 MiB gossip budgets were reported as a workaround | Peer records may not converge; publication can have no eligible storage peers | MCRB-18; availability blocker |
| [#6066](https://github.com/holochain/holochain/issues/6066) | Time-slice reads reportedly scan large Action/ChainOp data; at about 100k ops DHT update work can exceed its 15-minute cadence | Sustained CPU consumption, slow conductor startup and cells remaining not started | MCRB-19; scale/operations blocker |
| [#6064](https://github.com/holochain/holochain/issues/6064) | Wasmer module reloads reportedly retain machine code in a shared engine, causing memory growth with cell installation and idle reload | Multi-cell / long-lived conductor resource exhaustion | MCRB-20; operational blocker |
| [#6065](https://github.com/holochain/holochain/issues/6065) | Explicit existing-agent-key installs reportedly fail once Lair holds around 40+ keys because a single key-list reply exceeds an 8 KiB frame | Multi-agent identities and `restore_from_dht` may fail for a large keystore | MCRB-21; recovery/install blocker |

These are upstream-reported findings and reproduction claims, not independent Mycelix measurements. The MCRB run must capture the exact upstream issue state, runtime build, Lair and Kitsune2 versions, configuration, DNA, data scale and raw evidence.

PR [#6063](https://github.com/holochain/holochain/pull/6063) is merged to upstream `develop`, but the later #6071 report explicitly requests a backport to the 0.7 line. Do not assume that a merge to `develop` is included in the released 0.7.1 binaries.

### Release-target decision

- Keep 0.7.1 as the **current integration/qualification candidate** because it is the released 0.7 toolchain with updated libraries and migration guidance.
- Do not promote 0.7.1 to a Mycelix production baseline while a critical revocation regression remains unresolved for the deployed grant pattern.
- Test mitigations such as one grant per function only as explicit candidate policies; measure the additional grant count, authority state and attack surface. Do not silently change the application authorization model as a workaround.
- Test the 1 MiB gossip-budget workaround only with a pinned config and verify convergence, relay/network load and resource costs.
- Track upstream 0.7.2 planning issue [#6067](https://github.com/holochain/holochain/issues/6067), but do not pin or describe 0.7.2 as released until the release is actually published and its full component graph is compatible.
- Separate fixed upstream source, unreleased `develop`, release candidate, released binary and Mycelix exact-head qualification in every evidence receipt.

The 0.7.1 production gate is therefore conjunctive: dependency closure + source/API migration + capability/DirectSignal negative controls + multi-agent runtime + DHT convergence + large-key restore + long-lived resource tests + independent client E2E. A faster benchmark cannot compensate for a security or recovery failure.


### Composition with existing Mycelix authorization owners

MCRB-17 is deliberately a **Holochain adapter/runtime regression test**, not a second authorization or revocation engine:

- [#6236 — LISE trajectory-wide revocation propagation](https://github.com/Luminous-Dynamics/mycelix/issues/6236) owns the broader inventory of sessions, caches, tokens, background workers, delegated credentials, alternate routes and effective/observed revocation epochs.
- [#6193 — one-shot capability linearizability](https://github.com/Luminous-Dynamics/mycelix/issues/6193) owns the consistency contract and concurrent history checker for unique one-shot capability consumption under crash, partition and failover.
- [#6189 — effect-boundary interposition](https://github.com/Luminous-Dynamics/mycelix/issues/6189) owns the deterministic sandbox and exact capability enforcement at the effect boundary.

MCRB-17 supplies the substrate-specific negative control for Holochain 0.7.x multi-function grants. It should feed its exact runtime outcome into those owner programmes; it must not replace their semantics or imply that Holochain grant revocation alone proves global propagation.

### Executable QOPS accounting

The Rust metric-integrity crate now includes a pure QOPS reducer. It accepts the complete expected operation-ID set and one terminal outcome per ID, plus the independently produced metric-integrity report and scenario correctness result. It derives rates itself and suppresses all rate outputs when the denominator is incomplete, any duplicate operation ID appears, a duplicate external effect is observed, a hard correctness gate fails, or metric integrity is not Valid.

Outcome classes are:

- qualified_success — counts toward QOPS;
- correct_denial — reported but does not count as business success;
- unexpected_denial — reported and excluded from QOPS;
- failed — known failure, retained in the offered denominator;
- unknown — unresolved, retained in the denominator and never treated as success;
- duplicate_effect — non-compensatory failure that suppresses QOPS.

The reducer verifies that every supplied offered ID has exactly one outcome and rejects unexpected or duplicate outcome IDs. It does not prove that the offered-ID ledger itself is complete: a qualification receipt must bind the generator, workload seed, source digest, measurement-window identity and append-only offered-ID ledger. The benchmark must not let a caller construct a smaller denominator after seeing results.

The QOPS unit-test corpus covers complete accounting, missing outcome, duplicate outcome, unoffered operation, duplicate side effect, hard-correctness failure, metric-integrity failure and zero/empty measurement window. These are authored tests; exact-head Cargo test/Clippy is still the execution gate.

## 17. Wind Tunnel compatibility is a separate release gate

Do not equate a Wind Tunnel version with a Holochain version. They are independent version numbers and their dependency graphs must be inspected.

The public Wind Tunnel repository manifests inspected on 19 November 2026 show:

- Released Wind Tunnel v0.7.1 (published 21 July 2026) pins the Holochain 0.6.3 family: client 0.8.3, HDK 0.6.3, HDI 0.7.3 and Kitsune2 0.4.1.
- The then-current Wind Tunnel main manifest pins Holochain/HDK 0.7.0, HDI 0.8.0, Rust client 0.9.0 and Kitsune2 0.5.2.
- Mycelix's 0.7.1 migration profile requires Holochain/HDK 0.7.1, HDI 0.8.1, Rust client 0.9.1 and the Kitsune2 0.5.1 crate family; the Holonix bootstrap-server source 0.5.2 is a distinct component.

Therefore, neither the Wind Tunnel v0.7.1 release tag nor current main can be assumed to qualify the Mycelix Holochain 0.7.1 runtime. Reuse its framework/instrumentation where compatible, but first port and pin the Holochain client binding, runner, scenario crates, hApp dependencies and Kitsune2 graph as one coherent unit.

This is tracked separately in [#6337](https://github.com/Luminous-Dynamics/mycelix/issues/6337), with MCRB-22 as the release-compatibility scenario. Until that work is qualified, MCRB should use source-pinned Mycelix-specific harnesses or a verified compatible Wind Tunnel commit—not a version-number assumption.


### QOPS run-receipt binding

The run-receipt schema now requires a first-class QOPS record containing:

- an offer-ledger digest, entry count and workload-generator digest;
- a measurement-window ID, monotonic clock ID/epoch, raw start/end ticks and tick frequency;
- one terminal outcome count per class, including missing/unexpected/duplicate classifications;
- hard-correctness and metric-integrity dispositions;
- offered rate, qualified rate and qualified-success fraction.

The Rust reducer checks outcome coverage against the supplied expected-ID set. The receipt must additionally bind that set to the append-only offered-operation ledger and the exact workload generator/seed. A schema-valid receipt is not, by itself, proof that the offered ledger is complete: the independent verifier must recompute the ledger digest, confirm its entry count, verify the monotonic window and reconcile outcome IDs against that ledger.

QOPS eligibility requires a valid metric-integrity report, all declared hard correctness gates passing, complete one-to-one outcome coverage and zero duplicate external effects. For invalid, incomplete or not-evaluated QOPS, all performance-rate fields must be null rather than zero or a misleading partial rate.


### QOPS local-first versus remote qualification

Every versioned scenario now declares qops_requires_cross_host_visibility. This prevents a runner from deciding after the experiment whether remote observations were necessary.

For scenarios with this flag false, a local-only result may qualify if all other scenario requirements pass; cross-host timing remains NotEvaluated. For scenarios with the flag true, QOPS requires valid complete cross-host event pairs and calibrated clocks. The receipt records the flag, cross-host status and aggregate eligibility. A remote-observation workload cannot claim QOPS based on local write latency alone.


## 18. Offer-first QOPS denominator and canonical workload ledger

MCRB now defines the workload denominator as an artifact created before dispatch, not a list assembled after outcomes are known.

The offer-ledger verifier binds run/scenario identity, sequence, operation ID, request digest, idempotency requirement, monotonic schedule, clock identity/epoch, operation class, workload manifest digest, generator digest and parameter-set digest. Its canonical digest uses a domain separator, fixed field order, length-prefixed byte strings and big-endian integers. Entries are committed in sequence order; they are never sorted before hashing.

QOPS accepts only the opaque VerifiedOfferLedger returned by the verifier. It no longer accepts a caller-supplied expected-operation-ID vector or caller-supplied duration. The denominator is the verified entry count; the time window is derived from raw monotonic start/end ticks and tick frequency. Failed, denied, missing and unknown outcomes stay in that denominator. External-effect offers must declare an idempotency key, and duplicate keys are rejected.

The new MCRB-23 scenario injects truncation, omitted failed offers, post-result appends, duplicate/reordered operations, altered request digests, cross-run reuse, clock mismatches, out-of-window offers and missing idempotency keys.

**Authenticity boundary:** a recomputed hash proves internal consistency only. It does not prove that the workload generator or runner was honest, that every planned dispatch was entered into the ledger, or that outcome events were independently observed. Public benchmark claims still require an independently attested runner/orchestrator identity, workload generator digest, raw ledger artifact digest and custody-bound execution receipt. The source-level ledger verifier does not close that external trust boundary.

Related: [#6342 — independent offered-operation denominator](https://github.com/Luminous-Dynamics/mycelix/issues/6342).


## 19. Scenario-bound visibility policy

The requirement for cross-host observation is part of the frozen scenario contract, not a runner-selected option. Each offer-ledger envelope copies the selected scenario's qops_requires_cross_host_visibility value and commits it into the canonical offer-ledger digest. The verifier returns this value only through the opaque VerifiedOfferLedger; QOPS no longer accepts a caller-supplied cross-host boolean.

A local/offline-first scenario can therefore legitimately report cross-host timing as NotEvaluated when the frozen contract says remote visibility is not required. A DHT convergence, remote-signal, partition/rejoin or other peer-observation scenario must have complete, valid cross-host event-pair evidence before QOPS is eligible. Changing this flag after generating the ledger changes its digest and invalidates the prior commitment.

This protects against accidental or caller-driven policy downgrade inside the reducer. It does not prove that the selected scenario manifest, workload generator or runner is honest; those identities still require independent attestation and artifact custody.
