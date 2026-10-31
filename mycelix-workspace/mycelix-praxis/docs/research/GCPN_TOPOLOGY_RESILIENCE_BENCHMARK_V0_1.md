# GCPN Topology Resilience Benchmark v0.1

**Status:** deterministic synthetic structural comparison; non-operational.  
**Parent:** issue #5605.  
**Package:** `benchmarks/gcpn-topology-resilience`.  
**Architectures:** centralized hub-and-spoke, regional federation, local-first federation.

## 1. Why this benchmark exists

The GCPN record-level conformance suite can test whether evidence, authority, privacy and provenance invariants are preserved. It cannot yet answer a different question: what happens to civilian warning reachability when the network architecture loses a hub, regional service, WAN path or last-mile channel?

This benchmark isolates that structural question. Every architecture receives the same authorized synthetic alert, the same receiver cohorts, and the same fault row. The output is exact deterministic delivery arithmetic.

The model is intentionally abstract. It does not describe any named country's systems, real infrastructure topology, real population distribution, or live channel availability. Receiver counts are synthetic weights, not demographic data.

## 2. Architecture definitions

### A. Centralized hub-and-spoke

Every alert from the origin must reach a central hub. The hub then distributes to origin and remote region delivery paths.

Required dependencies:
- origin-to-central uplink;
- central hub availability;
- local origin last-mile path for local receivers;
- central remote distribution path and remote last-mile paths for remote receivers.

This model has a clear global coordination point and a corresponding central dependency.

### B. Regional federation

The origin region uses its regional service. Local delivery depends on that regional service and the local last mile. Cross-region delivery additionally depends on the interregional federation path, remote regional services and remote last-mile channels.

This model removes the single global hub but keeps regional service dependencies explicit.

### C. Local-first federation

Local authority and the local-first delivery path serve origin-region receivers without depending on WAN or federation availability. Cross-region delivery remains dependent on the origin regional federation service, interregional connectivity, remote regional services and remote last-mile channels.

This model preserves local continuity when cross-border links fail, while making no claim that a region can deliver when its local authority or local last-mile service is unavailable.

## 3. Frozen fault matrix

The 11 synthetic rows in `fixtures/topology-fault-matrix-v0.1.csv` use identical receiver groups:
- origin/local cohort: 1,000 synthetic receivers;
- remote region A: 600;
- remote region B: 400;
- total: 2,000 per scenario.

The fault cases are:
1. all services available;
2. central hub outage;
3. origin-to-central uplink loss;
4. central remote distribution path outage;
5. interregional federation partition;
6. origin regional service outage;
7. local-first local path outage;
8. origin last-mile outage;
9. remote last-mile outage;
10. origin authority unavailable;
11. remote regional services outage.

All booleans are explicit. No fault probability is assumed.

## 4. Reproduction

From the Mycelix repository root:

```bash
cargo test --manifest-path mycelix-workspace/mycelix-praxis/benchmarks/gcpn-topology-resilience/Cargo.toml
cargo run --manifest-path mycelix-workspace/mycelix-praxis/benchmarks/gcpn-topology-resilience/Cargo.toml
```

The executable prints one row per scenario and architecture, including intended receivers, delivered receivers, missed receivers, local/remote delivery, and delivery ratio.

## 5. Expected structural results

Every scenario has 2,000 intended receivers. The table reports local + remote deliveries:

| Scenario | Centralized | Regional federation | Local-first federation |
|---|---:|---:|---:|
| All available | 1,000 + 1,000 | 1,000 + 1,000 | 1,000 + 1,000 |
| Central hub outage | 0 + 0 | 1,000 + 1,000 | 1,000 + 1,000 |
| Origin central uplink loss | 0 + 0 | 1,000 + 1,000 | 1,000 + 1,000 |
| Central remote path outage | 1,000 + 0 | 1,000 + 1,000 | 1,000 + 1,000 |
| Interregional partition | 1,000 + 1,000 | 1,000 + 0 | 1,000 + 0 |
| Origin regional service outage | 1,000 + 1,000 | 0 + 0 | 1,000 + 0 |
| Local-first path outage | 1,000 + 1,000 | 1,000 + 1,000 | 0 + 1,000 |
| Origin last-mile outage | 0 + 1,000 | 0 + 1,000 | 0 + 1,000 |
| Remote last-mile outage | 1,000 + 0 | 1,000 + 0 | 1,000 + 0 |
| Origin authority unavailable | 0 + 0 | 0 + 0 | 0 + 0 |
| Remote regional services outage | 1,000 + 1,000 | 1,000 + 0 | 1,000 + 0 |

These results are implied by the topology rules and fixture states; they are not measured system performance. The model shows trade-offs rather than a universal winner:
- centralization preserves remote reach in the chosen interregional-peer partition, but depends on the hub and origin uplink;
- regional federation preserves local delivery through global partition, but local and remote service depend on the origin regional service;
- local-first federation preserves local reach through WAN/peer partition and regional-service outage, but still loses remote reach when federation dependencies fail;
- every architecture loses reach to a receiver cohort when that cohort's last-mile channel is unavailable;
- no architecture may issue or distribute an alert if the origin authority is unavailable.

## 6. Metrics

Report per scenario and architecture:
- intended receiver count;
- deliveries to origin, remote A and remote B cohorts;
- total delivered and absolute missed receivers;
- local continuity ratio;
- remote reachability;
- delivery ratio by cohort and total;
- topology-specific dependency failures.

Do not combine these into one weighted score. The hard authority gate is non-compensatory. Do not infer latency, reliability probability, forecast accuracy, human comprehension, cost, or real-world availability from this structural model.

## 7. Relation to industry and standards

CAP 1.2 remains the alert interchange standard, not a complete network or service architecture. Early Warnings for All is a four-pillar programme spanning risk knowledge, monitoring/forecasting, warning dissemination and preparedness/response. This simulator only explores a small aspect of the dissemination/federation problem and must not be presented as a replacement for national systems, CAP, EW4All or the Africa Multi-Hazard Early Warning and Early Action System.

The September 2026 Addis Ababa Call to Action highlights last-mile communication, accessible and multi-channel warning, CAP-enabled cell broadcast, national/local ownership, regional cooperation and sustainable operating finance. Those requirements motivate future dimensions beyond this deterministic route model: human-centred receiver trials, channel coverage, accessibility, local governance, early action and cost.

Reference links:
- ITU Addis Ababa Call to Action: https://www.itu.int/en/ITU-D/Emergency-Telecommunications/Pages/EW4ALL/Addis-Ababa-Call-to-Action-on-Early-Warnings-in-Africa.aspx
- UNDRR Africa Early Warnings Forum: https://www.undrr.org/news/africas-early-warning-experts-and-officials-call-sustained-momentum-life-saving-disaster
- UN Early Warnings for All: https://www.un.org/en/climatechange/early-warnings-for-all
- UNDRR CAP: https://www.undrr.org/early-warnings-for-all/common-alerting-protocol

## 8. Next benchmark layers

This v0.1 matrix is a small deterministic kernel. To approach a credible industry benchmark, subsequent versions must add, in order:

1. Explicit temporal events, queueing and delayed reconciliation; do not invent latency before defining the clock, link rates and queue policy.
2. Reproducible multi-seed availability/fault distributions calibrated only from public or voluntarily contributed data.
3. Correlated common-mode failures and shared upstream source dependencies.
4. Member key rotation, revocation, withdrawal, compromised credentials and append-only reconciliation.
5. CAP 1.2 round-trip and update/cancel lifecycle tests.
6. Receiver-side channel delivery by language, accessible format and connectivity cohort.
7. Voluntary receiver comprehension and action trials under ethics/privacy review.
8. Comparative lifecycle cost and operator burden under matched service scope.

Confirmatory improvement claims require sealed scenarios, paired inputs, a second evaluator, confidence intervals and compatible cost/receiver denominators. Named national systems should not be ranked without permissioned adapters and independent evidence.

## 9. Safety and non-claims

This is a deterministic synthetic model of civilian alert reachability only. It does not simulate military networks, weapons, missile trajectories, sensor placement, targeting, fire control, interceptors, engagement, operational alert policies, or any real population or infrastructure. It is not a warning service and must never dispatch real alerts.
