# Pulse causal AuthorityCheckpoint — bounded model-check receipt

**Result:** PASS for the stated finite freshness model and exact model-check run  
**Workflow:** [Pulse Authority Lifecycle Model, run #56](https://github.com/Luminous-Dynamics/mycelix/actions/runs/41856892659)  
**Artifact:** [TLC logs and counterexample](https://github.com/Luminous-Dynamics/mycelix/actions/runs/41856892659/artifacts/13337397306), ID `13337397306`  
**PR head checked:** `97efb39400624f2f91429828e6742558961e95b3`  
**Merge checkout:** `3884e107c570707e68afa1ca6ae9c15d83b213f7`  
**Base:** `9e12dd46bdd8371e8ed9626a17bc06f21c141a52`  
**TLC:** 1.8.0 / build `2026.08.11.125311`  
**TLC JAR SHA-256:** `ab323b79802aedc3203b3f9af37c6aca3ed43f4e0225b36f2aa77b26de46c05f`  
**Artifact ZIP SHA-256:** `06234f8a09cf0db565aeafcfe75590aba16c6baa9396d7c64fb9327a676d0ee0`

## Results

| Configuration | Result | States generated | Distinct states | Outcome |
|---|---|---:|---:|---|
| `PulseAuthorityCheckpoint.cfg` | PASS | 27 | 17 | State queue exhausted; no invariant error |
| `PulseAuthorityCheckpoint-bypass-mutant.cfg` | Expected counterexample | 10 | 9 | `NoFalseFreshAuthority` violated |

The parent workflow also reran the authority lifecycle and mailbox models:
- authority safe: 83,941 / 10,249; projection and grant-idempotency mutants caught;
- mailbox safe: 37 / 20; proposed-state and field-mutation mutants caught.

All eight model phases in this run behaved as expected. A mutant phase is counted as successful only when TLC exits non-zero and the named invariant appears in the log.

## Repeat run

The checkpoint safe and bypass configurations reproduced their prior outcomes in [model workflow run #60](https://github.com/Luminous-Dynamics/mycelix/actions/runs/41857220290), on PR head `34b1d695b8ce7d5bc878ee9d3025fa738514f1d2` (merge checkout `8e4943ff`). Safe: 27 generated / 17 distinct states, empty queue; bypass mutant: `NoFalseFreshAuthority`.

Artifact ID `13337303730`, ZIP SHA-256 `17da4b4450375fd6c262aa59f80d680954f297a2a8cd4a55b8a2b542995ca36a`. Same bounds, no production-runtime freshness proof.

## Modeled contract

The model keeps distinct:
- current conductor host-grant state;
- canonical authority epoch and revocation state;
- observer projection epoch/state;
- the minimum checkpoint epoch requested by the caller;
- a host authorization probe and an application-read classification.

A host probe is a historical observation bound to host state at probe time. Later revocation does not rewrite that observation. Similarly, an application-read result is checked against canonical state, observer epoch and required checkpoint **at read time**; later changes do not retroactively rewrite it.

After host revocation commits, the required epoch advances while the observer projection may remain stale-active. The safe constrained reader returns `UNKNOWN` until the observer reaches the required epoch. After convergence it may return a trusted revoked projection. The bypass mutant labels the stale active projection as trusted and TLC catches the false-fresh claim.

## Boundaries

This is a finite abstract model with one grant, one grantee/outsider distinction, one revocation generation and one observer. It does not model Holochain's DHT transport, actual source-chain receipts, multiple peers, partitions, competing checkpoints, garbage collection, network liveness, cryptographic authenticity or production latency. It does not establish that Holochain already exposes a ZedToken-equivalent API.

The proposed AuthorityCheckpoint is a **read-freshness receipt**, not a bearer token, permission, grant or substitute for conductor-enforced CapGrant/domain checks.

**Claim ceiling:** bounded model-check evidence for this exact specification/configuration and TLC build only. Runtime implementation and two-conductor tests remain unqualified.
