# Pulse witnessed transparency checkpoint — qualification profile v1

**Profile:** PULSE-TRANSPARENCY-WITNESS-001  
**Status:** Research model; not a deployed witness protocol  
**Parent issue:** https://github.com/Luminous-Dynamics/mycelix/issues/5942  
**Related evidence profile:** https://github.com/Luminous-Dynamics/mycelix/issues/5863

## Problem: inclusion is not global agreement

SCITT RFC 9943 already requires the Transparency Service's Verifiable Data Structure to provide append-only and non-equivocation properties: no fork in the accepted statement sequence, with auditors able to check that everyone sees the same ordered collection. This PR does not weaken or replace that normative requirement.

The research question is whether an independent relying-party/monitor layer can detect and contain a TS that violates its non-equivocation promise. RFC 9162 documents the residual split-view problem: Merkle inclusion and consistency checks against a client's own previous checkpoint can be circumvented by a log that shows different, inconsistent views to different clients. It identifies comparison/gossip between clients and monitors as a way to detect inconsistent signed tree heads.

Two different clients can each receive a valid signature under the same log key, a valid inclusion proof, and a valid consistency proof from the same earlier checkpoint, yet receive different same-size tree roots. Each local proof bundle may look internally consistent. The 3-of-4 witness overlay modeled here is an optional independent-monitor policy for this adversarial case. A real TS must still conform to RFC 9943's VDS non-equivocation requirement; witnesses are defense-in-depth and relying-party evidence, not a substitute for that service guarantee.

## Candidate static witness profile

The finite model uses four witnesses, at most one Byzantine witness, and a three-signature quorum.

For n committee members, at most f Byzantine members, and quorum size q, any two quorums intersect in at least 2q - n members. To guarantee an intersection containing at least one honest witness, require 2q - n > f, equivalently q > (n + f)/2.

For n=4 and f=1, q=3 is the smallest integer quorum satisfying the condition. A two-signature quorum is unsafe in this profile: two 2-of-4 subsets can each contain the one Byzantine witness and a different honest witness.

This arithmetic is only one part of the argument. An honest intersection member prevents split views only if it binds its signature to the exact log ID, tree size and root hash; durably stores its prior signature/lock before acknowledging; refuses conflicting roots for the same checkpoint identity; cannot lose or roll back that lock during restart/recovery; and has a signing key and operation path not controlled by the log operator or another witness.

The committee's administrative, infrastructure, cloud-account, key-custody and software supply-chain independence must be evidenced. Four keys operated by one administrator are not four independent witnesses.

## Checkpoint identity and vote

A real profile should sign a canonical, domain-separated checkpoint statement containing protocol/schema version, transparency-service/log identity, tree size, Merkle root hash, prior-checkpoint reference, policy/profile version, log signing key ID and witness committee epoch.

A witness lock key is the tuple identified by log_id, tree_size and committee_epoch. It records the exact checkpoint digest, not merely a Boolean signed flag. A witness seeing a different digest for an already locked identity must reject it and preserve both observations as equivocation evidence.

A later tree size is a new checkpoint identity; it still requires a valid consistency proof from the prior checkpoint and cannot bypass the prior lock/history.

## Standards boundary

RFC 9943 defines the SCITT VDS requirement as append-only and non-equivocating. The TS implementation is responsible for that property. The witness protocol in this research PR is not a new SCITT requirement and does not make an otherwise non-conformant TS conformant. It models how independent clients/relying parties can refuse to trust divergent same-size roots and preserve evidence of suspected service equivocation.

RFC 9162's Certificate Transparency auditing discussion provides the closest operational analogue: clients/monitors compare signed tree heads and audit inclusion/consistency, because each client's local proofs alone do not necessarily expose every split view. The Pulse overlay investigates a stronger, explicitly thresholded witness-policy option. It must be compared against ordinary gossip/monitoring, not assumed superior without a matched fault-injection benchmark.

## Modeled assumptions

The TLA+ model intentionally abstracts the Merkle and COSE algorithms. It assumes both candidate roots are individually plausible under the log key and have valid inclusion and consistency proofs from the same prior checkpoint. It then asks whether the witness protocol prevents clients from accepting different roots at the same checkpoint identity.

Model artifacts:
- PulseTransparencyWitness.tla
- PulseTransparencyWitness.cfg — safe full-view 3-of-4 policy
- PulseTransparencyWitness-partition.cfg — 2–2 client partition, q=3
- PulseTransparencyWitness-quorum-mutant.cfg — unsafe 2-of-4 quorum
- PulseTransparencyWitness-double-sign-mutant.cfg — honest witness lock disabled
- PulseTransparencyWitness-lock-loss-mutant.cfg — witness lock lost on restart

### Safe behavior expected

- at most one Byzantine witness may sign both roots;
- each honest witness signs at most one root for a fixed checkpoint identity;
- a client accepts only a root with three matching signatures visible to it;
- conflicting roots cannot each obtain a three-signature quorum;
- a 2–2 partition gives neither client a quorum, so each remains unaccepted/UNKNOWN;
- once the partition heals, a fresh valid checkpoint can be assembled under the same policy.

### Negative controls expected

| Mutant | Deliberate weakness | Expected invariant |
|---|---|---|
| Weak quorum | 2-of-4 with one Byzantine witness and 2–2 partition | NoSplitViewAccepted |
| Honest double-sign | Honest witnesses can sign two roots at same identity | NoSplitViewAccepted |
| Lost lock | Honest witness restarts without durable anti-double-sign state | NoSplitViewAccepted |

The workflow treats a mutant as detected only when TLC exits non-zero and the intended invariant name appears in the counterexample log. Parser errors or unrelated invariant failures do not satisfy a mutant check.

## Required runtime interoperability work

The model is not an implementation of SCITT, a COSE receipt parser, or a Merkle VDS. Runtime qualification should select an existing RFC 9943-compatible TS/VDS implementation; verify the TS signature, COSE receipt, statement digest, inclusion proof and consistency proof offline; provision witness keys separately from the TS log key; collect identical checkpoint commitments from each witness; test the real TS under split-view fault injection using at least two independent clients and monitors; restart witnesses between signing attempts and prove durable lock state survives; and capture a contradictory checkpoint as a security incident rather than silently choosing the newest timestamp/root.

Do not introduce a custom Merkle tree or new transparency ledger until an interoperability evaluation demonstrates that existing implementations cannot satisfy the profile.

## Performance and availability benchmark

Measure TS checkpoint creation/signing, witness verification/signing, quorum assembly, inclusion and consistency proof generation/verification, offline client verification, checkpoint gossip/monitor discovery, and time to detect/quarantine contradictory roots separately.

Report signature bytes, proof bytes, witness storage writes, persistent-lock lookup cost, CPU, memory, network bytes and quorum availability. Separate healthy, delayed, one-node unavailable, Byzantine, 2–2 partition and post-restart strata.

Provisional targets, not measured results: zero split-view acceptances in the bounded mutants; 100% UNKNOWN/INCOMPLETE when no matching quorum is available; liveness with exactly three mutually reachable witnesses when one witness is unavailable; no new checkpoint acceptance during a 2–2 split; and independent process-level runs for reported percentiles with raw receipts/environment manifests retained.

## Claim ceiling

A passing finite TLC run proves only that this finite model, under its stated Byzantine and lock assumptions, preserves its invariants and detects the chosen mutants. It does not prove the TS uses the modeled protocol, witnesses are independent or honest, keys are secure, anti-double-sign state is durably persisted by a production service, COSE/Merkle proofs are implemented correctly, all partitions/healing schedules are covered, or the signed statement's underlying event is true.

A transparency receipt never creates a Holochain CapGrant, never overrides a conductor denial, and never substitutes for domain-resource authorization.
