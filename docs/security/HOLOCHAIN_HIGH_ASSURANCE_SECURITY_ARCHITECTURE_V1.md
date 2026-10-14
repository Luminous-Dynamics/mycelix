# Holochain High-Assurance Security Architecture v1

**Status:** Architecture / qualification plan
**Scope:** Holochain security properties for Mycelix identity, evidence, governance, and economic state

## Executive conclusion

Holochain can be made as secure as a particular blockchain for selected application guarantees, and can be more secure for some Mycelix-specific properties. It is not correct to claim that ordinary Holochain provides the same global-consensus guarantees as every blockchain.

Security must therefore be decomposed:

~~~text
Holochain = intrinsic integrity + agent-centric history + DHT validation + capability security
Blockchain = shared ordering + canonical state + consensus/finality + economic or quorum security
~~~

## What Holochain already provides

Holochain source-chain actions are cryptographically signed and hash-linked. DHT operations are validated by peers using deterministic integrity rules; invalid operations can produce warrants. Capability-based zome calls provide explicit access control. These are real security mechanisms, not merely application conventions.

Holochain's own documentation also makes the boundary explicit: it does not maintain consensus over global state, and countersigning is not itself a general double-spend-prevention mechanism.

## Security property matrix

| Property | Holochain native strength | Mycelix strengthening |
|---|---|---|
| Authorship / provenance | Very strong | key rotation/recovery + explicit identity bindings |
| Application data integrity | Very strong | adversarial validation corpus + exact-head qualification |
| Invalid-agent detection | Strong | quarantine policy + retained warrants/evidence |
| Capability authorization | Strong | least privilege + authority-version binding |
| Privacy | Strong architectural fit | purpose-bound disclosure + ZK proofs where needed |
| Global ordering | Not native | resource-specific quorum or external settlement |
| Double-spend prevention | Not globally native | scarce-resource quorum protocol |
| Global singleton state | Not native | canonical authority or quorum |
| Byzantine agreement | Not global | explicit BFT-style application protocol |
| Partition tolerance | Strong | safe conflict state; sacrifice writes for scarce resources |
| Economic finality | Not native | qualified public anchor or quorum certificate |

## Four-layer high-assurance stack

~~~text
Layer 4 — External finality
    Ethereum / qualified L2 / other public anchor

Layer 3 — Scarce-resource coordination
    resource-specific witness/quorum protocol

Layer 2 — Holochain intrinsic integrity
    signed source chains
    deterministic validation
    DHT authorities
    warrants / quarantine
    capability security

Layer 1 — Application semantics
    authority + evidence + provenance + privacy
~~~

A weaker or cheaper layer must never silently substitute for a stronger one.

## 1. Strengthen integrity validation

Every authority-bearing integrity zome should satisfy:

- exact predecessor dependencies are authoritative;
- missing/malformed dependencies fail closed;
- action author is derived from the authenticated action, not caller payload;
- authority-bearing timestamps bind to action timestamps;
- singleton roots are not inferred from mutable link cardinality;
- conflicting roots become explicit conflict, never an implicit winner;
- validation remains deterministic and side-effect free;
- external observations carry source, freshness, revision, and known-at semantics;
- historical evidence cannot silently become current authority.

Holochain's documentation specifically recommends deterministic must_get_* dependency reads in validation and warns against mutable collections such as get_links there.

## 2. Upgrade scarce-resource security

This is the central gap between ordinary Holochain and a consensus blockchain.

For money, finite inventory, unique ownership, voting rights, or single-use credentials:

~~~text
Resource R
  -> allocation intent
  -> deterministic witness set
  -> quorum certificate
  -> committed allocation
~~~

Let n be the witness population and q the required quorum. q > n/2 gives quorum intersection; a Byzantine protocol needs a threshold appropriate to its explicit fault model, commonly q > 2n/3 when the assumption is fewer than n/3 Byzantine witnesses. The exact theorem must be stated and independently tested.

Safety target:

~~~text
Two conflicting allocations for one resource
cannot both obtain valid quorum certificates
under the declared topology and honest-intersection assumptions.
~~~

If the assumptions fail, the system should emit equivocation/conflict evidence instead of inventing a winner.

## 3. Keep balances as projections

The authoritative economic objects should be append-only events:

~~~text
Issuance
Transfer
Reservation
Settlement
Redemption
Correction
Supersession
~~~

A balance is a deterministic projection of valid events through an explicit frontier.

## 4. Add public checkpointing where global finality matters

For high-value state, compute a deterministic snapshot commitment and anchor it publicly:

~~~text
Holochain event frontier
  -> canonical serialization
  -> commitment / Merkle root
  -> Ethereum or another qualified public anchor
~~~

The anchor proves that the commitment existed by the anchor point. It does not prove that application semantics were correct.

## 5. Preserve partition tolerance

Do not turn every Mycelix interaction into a global-consensus transaction.

Personal journals, private evidence, care records, research notes, and local coordination can remain agent-centric.

Use stronger coordination only where conflicting histories create material harm:

~~~text
money
finite inventory
unique ownership
single-use credentials
constitutional authority
cross-domain settlement
~~~

This reduces the amount of the system that needs consensus-like machinery.

## 6. Make Byzantine assumptions explicit

Never infer Byzantine safety from the existence of distributed validators.

~~~text
security claim
 = protocol
 + topology
 + witness-selection
 + Sybil model
 + fault threshold
 + cryptographic assumptions
 + recovery assumptions
~~~

Every high-assurance claim should carry those coordinates.

## 7. Holochain can exceed a generic blockchain on some application properties

A global ledger forces all participants to replicate and agree on a shared state that many applications do not actually need. Holochain can reduce the global consensus surface and preserve direct authorship, capability boundaries, and privacy for independent state.

That can produce stronger application-level security by avoiding unnecessary global state and reducing the number of security-critical transitions.

This is not a claim that Holochain has stronger global consensus than Ethereum.

## 8. Ethereum-style finality remains useful

Ethereum proof-of-stake currently provides explicit checkpoint finality backed by economic penalties. Mycelix should use such a public finality mechanism only for state that genuinely needs globally anchored settlement.

## 9. Define three named security profiles

### Holochain Local Integrity

Guarantees signed authorship, deterministic validation, dependency integrity, and warrant-based bad-actor evidence.

Does not guarantee global order, global singleton state, or double-spend prevention across disconnected partitions.

### Holochain Quorum Economic

Adds deterministic witness selection, quorum certificates, resource-specific allocation, equivocation detection, replay protection, and recovery.

Claim ceiling: safe under the named quorum topology and fault assumption.

### Holochain Anchored Settlement

Adds the quorum economic protocol, economic journal, public checkpoint, external finality receipt, reconciliation, and rail-specific risk policy.

Claim ceiling: settlement final under the exact anchor, adapter, and finality assumptions.

## 10. Formal qualification program

### A. Pure protocol model

Model reservations, transfers, issuance, redemption, witness voting, quorum formation, timeout, retry, equivocation, recovery, and checkpointing as deterministic state machines.

### B. Adversarial property testing

Generate conflicting concurrent allocations, Byzantine witness responses, duplicate votes, stale epochs, forged identities, replay, partitions, delayed messages, malformed dependencies, conflicting roots, key rotation, and interrupted recovery.

### C. Live Holochain 0.7 testing

Run exact Holochain 0.7 / HDK 0.7 / HDI 0.8 profiles with multiple conductors, partitions, crashes, delayed gossip, malicious agents, witness loss, quorum loss, recovery, and reconciliation.

### D. External settlement testing

For each public rail, prove that transaction hashes are not mistaken for finality, stale observations fail, profile/chain/version substitution fails, and public finality is evaluated independently.

### E. Independent verifier

The strongest evidence path is an independent verifier that is not the same implementation as the protocol under test.

~~~text
implementation
  -> receipt
  -> independent verifier
  -> qualified claim
~~~

## 11. Substrate truth is itself a security boundary

Holochain 0.7.0 is currently the recommended general-use release, with HDK 0.7.0 and HDI 0.8.0 in the compatible family. Finance security qualification should therefore be explicitly bound to the reconciled 0.7 substrate rather than silently qualifying an older 0.6-era environment.

Current Mycelix issue AC-123 (#4317) already tracks this reconciliation. AC-142 (#4384) records the substrate mismatch and makes it a qualification blocker.

## 12. Final theorem

Do not make Holochain a blockchain.

Make Mycelix a layered system in which each security property is supplied by the smallest mechanism capable of proving it:

~~~text
Holochain intrinsic integrity
+ scarce-resource quorum safety
+ public checkpoint/finality where required
+ independent verification
~~~

That architecture can be more secure for Mycelix than putting every object into a conventional global ledger.

The claim must always remain bounded by:

- exact protocol;
- exact implementation/version;
- topology;
- attacker model;
- fault threshold;
- protected asset/state;
- recovery assumptions;
- finality semantics.

## Nonclaims

This document does not itself establish Byzantine safety, global consensus, censorship resistance, monetary soundness, or legal authority. Each property requires its own protocol and qualification theorem.

## References

- Holochain validation: https://developer.holochain.org/build/validation/
- Holochain DHT: https://developer.holochain.org/concepts/4_dht/
- Holochain source chain: https://developer.holochain.org/concepts/3_source_chain/
- Holochain countersigning: https://developer.holochain.org/concepts/10_countersigning/
- Holochain capabilities: https://developer.holochain.org/build/capabilities/
- Holochain 0.7 compatibility: https://developer.holochain.org/resources/compatibility/holochain-0.7/
- Ethereum proof-of-stake finality: https://ethereum.org/developers/docs/consensus-mechanisms/pos/