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
