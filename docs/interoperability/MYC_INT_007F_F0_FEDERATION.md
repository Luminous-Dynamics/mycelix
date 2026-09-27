# MYC-INT-007F — F0 Federation Topology and Fault Manifest

Status: design/fixture only. Tracks #3251. Child of MYC-INT-007E / PR #3250.

## Purpose

Freeze the first heterogeneous Integral/Mycelix federation showcase as one machine-readable experiment subject rather than a diagram whose meaning can drift between runs.

Canonical fixture:

`docs/interoperability/fixtures/MYC_INT_007F_F0_FEDERATION.json`

## Experiment identity

F0 contains exactly eight logical nodes with deliberately different roles and provenance classes:

- A — physical H1 productive node;
- B — replay-backed second productive node;
- C — OAD/design-heavy node;
- D — COS/manufacturing-heavy node;
- E — CDS/deliberation-heavy node;
- F — FRS/analysis-heavy node;
- G — degraded old-generation node;
- H — conventional PostgreSQL/HTTP conformer.

The runtime family is fixture metadata, not durable semantic identity.

## Core boundaries

```text
connectivity
!= semantic compatibility
!= policy acceptance
!= authority recognition

message delivered
!= effect authorized

signed artifact
!= locally accepted artifact

runtime ID
!= external semantic ID
```

## Provenance visibility

Every node is labeled by provenance class. The first physical-pilot count is therefore one, not eight.

```text
8 logical nodes
!= 8 physical pilots
```

Synthetic and replay-backed nodes are valid conformance subjects but must remain visibly synthetic/replay-backed in both UI and reports.

## Local-first invariant

Node A is expected to retain admitted local sensor acquisition and local observation history when Mycelix/Holochain/federation peers are unavailable.

This protects the architecture from making the global network load-bearing for basic field instrumentation.

## Demonstration phases

The manifest freezes D1 through D11:

1. local-first normal operation;
2. design propagation;
3. duplicate COS/accounting delivery;
4. FRS recommendation path;
5. foreign-decision authority negative control;
6. partition;
7. schema/version skew;
8. reconnect/reconciliation;
9. contradictory physical outcome;
10. optional Symthaea analysis;
11. runtime heterogeneity through the conventional conformer.

Each phase carries semantic assertions rather than only expected network behavior.

## Fault campaigns

F01–F18 cover node outage, unknown delivery, duplicate/reordered transport, stale summaries, schema skew, clock skew, foreign authority, expired authorization, revoked certification, contradictory observations, false-positive recommendations, analysis outage, Edge-local continuity, local policy rejection, persisted-before-ACK uncertainty, and export/import through a conventional runtime.

The fault IDs become the stable vocabulary for later network-emulation and UI work.

## Network conditions

F0 defines three zones:

- local cluster;
- remote/intermittent cluster;
- compatibility/external edge.

Latency/jitter/bandwidth/packet-loss values in the JSON are synthetic fixture parameters. They are not measured internet conditions or production service-level claims.

## Reconciliation rule

Reconnect does not permit silent last-arrival-wins collapse.

```text
partitioned histories reconnect
-> preserve source ownership and conflict/indeterminacy
-> reconcile only through the exact semantic profile
```

Delivery history and semantic history remain distinct.

## UI contract

Both Story View and Evidence View must consume the same underlying experiment state.

Story View may simplify presentation, but it may not hide a state that changes meaning, including:

- stale;
- unknown;
- conflicting;
- incompatible schema;
- foreign/local authority distinction;
- physical versus synthetic provenance.

Green links never imply successful semantic admission or authorization.

## Scale progression

The machine-readable format is intended to support later F1/F2/F3 generations, but only F0 is currently defined.

```text
F0  8 nodes
F1 32 nodes      gated on F0 evidence
F2 128 nodes     gated on F1 evidence
F3 512+          evidence-gated
```

Later scale runs should add resource/convergence measurements without changing F0 history.

## Execution boundary

007F is not yet an executable federation qualifier. A future harness may translate this exact fixture into Nix/containers/VMs and deterministic network-fault controls, but semantic PASS remains dependent on the exact upstream qualified contracts.

## Nonclaims

007F does not prove production scalability, institutional scalability, network security, governance quality, Holochain superiority, Mycelix adoption suitability, or Integral endorsement.
