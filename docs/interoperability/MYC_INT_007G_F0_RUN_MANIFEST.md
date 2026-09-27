# MYC-INT-007G — F0 Reproducible Run Manifest

Status: design/fixture only. Tracks #3255. Child of MYC-INT-007F / PR #3253.

## Purpose

Bind one exact execution attempt of the F0 federation fixture to exact implementation subjects, workload generations, network/fault controls, and retained evidence.

Canonical fixture:

`docs/interoperability/fixtures/MYC_INT_007G_F0_RUN_MANIFEST.json`

## Core rule

```text
same topology
!= same executed experiment

same topology + exact run manifest
-> comparable run subject
```

The run manifest never changes 007F node, phase, or fault semantics. It only binds one implementation/environment instance to them.

## Execution modes

The profile supports:

- `DeterministicLocal` — local processes/containers with deterministic fault scheduling where possible;
- `NetworkEmulated` — explicit network namespace/container/netem-or-equivalent faults;
- `MixedPhysicalVirtual` — physical H1 Node A plus virtual/replay/conventional peers.

Tool choice is implementation metadata, not durable semantic identity.

## Node binding

Every 007F node receives an implementation binding containing exact build/image/commit identity, runtime mode, storage profile, schema/interface generations, enabled optional services, and provenance.

```text
container restarted
!= semantic node replaced

runtime-native node ID
!= federation semantic node identity
```

## Network/fault binding

Every run binds exact emulation/fault tooling and configuration. Link state remains separate from semantic admission and authority.

```text
packet dropped
!= semantic rejection

ACK missing
!= receiver definitely did not persist
```

## Evidence package

Retain exact run-manifest bytes/commitment, node/build identities, node logs, delivery/ACK traces, schema/translation receipts, authority decisions, conflict/reconciliation evidence, fault-injector log, measured resource telemetry where collected, and final state/export commitments.

Wall-clock log order must not become the semantic event-order theorem.

## Physical-node safety

In `MixedPhysicalVirtual`, the federation harness is read/evidence-only toward H1 by default.

```text
federation membership
!= local actuator authority
```

No fault campaign may bypass H1 local stop/interlock rules.

## Result reporting

Report independently:

- semantic invariant outcomes;
- delivery/ACK outcomes;
- reconciliation outcomes;
- authority outcomes;
- staleness/conflict outcomes;
- runtime/resource measurements;
- indeterminate states;
- operator interventions;
- implementation-specific failures.

No scalar federation score is defined.

## Qualification boundary

007G defines a run subject only. Execution and PASS require future exact harness/tooling plus qualified semantic dependencies.

## Nonclaims

007G does not prove federation scalability, production readiness, Holochain/PostgreSQL/Mycelix correctness, institutional adequacy, or authorization of physical effects.
