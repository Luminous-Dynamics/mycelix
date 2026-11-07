# Pulse current-vs-historical trust adjudication — bounded model-check receipt

**Result:** PASS for the finite abstract trust-adjudication model and exact model-check run  
**Workflow:** [Pulse Authority Lifecycle Model, run #350](https://github.com/Luminous-Dynamics/mycelix/actions/runs/42394313382)  
**Job:** `128468298587` — Safe model and negative-control counterexamples  
**Artifact:** [TLC logs, counterexamples and receipt](https://github.com/Luminous-Dynamics/mycelix/actions/runs/42394313382/artifacts/13582598173)  
**Artifact ID:** `13582598173`  
**Artifact ZIP SHA-256:** `62e6c8fc60667404defe020cd2968e01e913d40f71da6fc13789de8270fb6a78`  
**Exact branch PR head:** `0fda1650b951eae13768a2f53ecf6960a67549bd`  
**TLC:** 1.8.0 / build `2026.08.11.125311`  
**TLC JAR SHA-256:** `ab323b79802aedc3203b3f9af37c6aca3ed43f4e0225b36f2aa77b26de46c05f`

## Results

| Configuration | Result | States generated | Distinct states | Outcome |
|---|---|---:|---:|---|
| `PulseTrustHistoricalAdjudication.cfg` | PASS | 189 | 60 | State queue exhausted; no invariant error |
| `PulseTrustHistoricalAdjudication-current-mutant.cfg` | Expected counterexample | 21 | 16 | `NoCurrentAcceptanceFromRevokedKey` violated |
| `PulseTrustHistoricalAdjudication-time-mutant.cfg` | Expected counterexample | 26 | 20 | `HistoricalAcceptanceRequiresTrustedEvidence` violated |

The overall run #350 completed all model phases successfully. A mutant phase is successful only when TLC exits non-zero and emits the specific invariant named by that phase; an arbitrary parser or configuration error does not count.

## Encoded contract

The model separates four outcomes:

- `CURRENT_ACCEPTED`: the signing key is active under the current policy at decision time and the statement signature/epoch are valid;
- `HISTORICALLY_AUTHENTICATED`: the signature was valid under the retained policy epoch and before the key's modeled revocation epoch, with trusted time and the exact historical policy snapshot available;
- `INDETERMINATE`: trusted time or historical policy evidence is unavailable;
- `REJECTED`: the signature/policy is invalid, the policy epoch is not retained, or the key was already revoked at the statement epoch.

The model's safe transition creates a statement under key A at policy epoch 1, advances the current policy to epoch 2, revokes key A and activates key B. It then verifies the old statement as a historical record while rejecting it for a current decision.

## Negative controls

1. **Historical-to-current privilege confusion:** the mutant permits a historically valid statement from key A to authorize a current decision after key A is revoked. TLC catches the violation of `NoCurrentAcceptanceFromRevokedKey`.
2. **Missing time/policy evidence:** the mutant accepts a historical statement when trusted time or the exact historical policy snapshot is unavailable. TLC catches the violation of `HistoricalAcceptanceRequiresTrustedEvidence`.

## Boundaries

This is a finite abstract state model with two keys and two policy epochs. It does not verify real Ed25519/DSSE signatures, establish trusted wall-clock time, authenticate time-stamping authorities, persist or retrieve an actual policy-history chain, resolve real policy forks, or run the Rust verifier. The model represents trusted time and historical policy availability as explicit Boolean inputs and the statement signing date as a policy epoch; it does not prove that those inputs are independently trustworthy.

The Rust implementation currently provides a static pinned-root policy snapshot path, but does not yet expose a fully qualified historical adjudication API with trusted timestamps, effective revocation times, durable policy-history retrieval and independent verification. Those remain implementation gates under [#5895](https://github.com/Luminous-Dynamics/mycelix/issues/5895).

**Claim ceiling:** bounded model-check evidence for the exact TLA+ model/configurations checked by TLC 1.8.0 at PR head `0fda1650b951eae13768a2f53ecf6960a67549bd`. It is not a cryptographic implementation audit, trusted-time service, production key-compromise adjudication or runtime qualification.
