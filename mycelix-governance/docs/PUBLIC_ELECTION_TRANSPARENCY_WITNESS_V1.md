# Mycelix Public Election Transparency + Witnessing v0.1

Status: **ELECT-009 / ELECT-010 foundation; structural contracts only**

Parent stack:

- `mycelix-public-election-v1`
- `mycelix-public-election-anonymous-authority-v1`

Profiles:

- `mycelix-public-election-transparency-v1`
- `mycelix-public-election-witness-quorum-v1`

## Purpose

This tranche establishes the public election record as an **append-only, independently witnessed evidence history**, not merely a collection of Holochain entries.

The governing distinction is:

```text
append-only consistency
        !=
common public view
```

A Merkle-style consistency proof can show that checkpoint B extends checkpoint A. It cannot, by itself, prove that a malicious log operator did not show one valid-looking history to observer X and a different valid-looking history to observer Y.

Therefore Mycelix public elections require two independent assurance layers:

1. **ELECT-009:** cryptographic checkpoint lineage and inclusion/consistency evidence;
2. **ELECT-010:** witness agreement across genuinely independent control domains.

## Research basis

RFC 9162 Certificate Transparency is not an election protocol, but its transparency-log model is directly useful here. It formalizes append-only Merkle logs, inclusion proofs, consistency proofs, signed tree heads, and monitoring. It also explicitly notes the split-view problem: a malicious log can present different inconsistent views unless clients/monitors compare what they observed.

ElectionGuard independently reinforces the value of a public election record: its published election record is intended to contain the artifacts needed to verify an election while excluding secret material.

References:

- https://www.rfc-editor.org/rfc/rfc9162.html
- https://electionguard.vote/spec/
- https://electionguard.vote/develop/Election_Record/

Mycelix does not copy Certificate Transparency or ElectionGuard wholesale. The useful pattern is the separation between a reproducible public record, append-only proofs, and independent observation.

## ELECT-009 — append-only checkpoint lineage

`ElectionTransparencyCheckpointV1` binds:

- public-election profile;
- transparency profile;
- exact election constitution;
- frozen log parameters;
- frozen canonicalization profile;
- checkpoint sequence;
- tree size;
- root digest;
- predecessor checkpoint digest; and
- consistency-proof digest.

Genesis has no predecessor and no consistency proof.

Every non-genesis checkpoint must name both.

`validate_checkpoint_successor(...)` additionally requires:

```text
same election constitution
same log parameters
same canonicalization profile
next.sequence == previous.sequence + 1
next.tree_size > previous.tree_size
next.previous_checkpoint_digest == exact previous checkpoint digest
```

This prevents configuration drift, sequence gaps, rollback, and silent predecessor substitution at the structural boundary.

The crate intentionally does **not** implement a Merkle hash function or verify a consistency proof. The exact hash/serialization/proof algorithm belongs in a later frozen transparency-log profile and independent verifier.

## Inclusion evidence

`InclusionEvidenceRefV1` structurally binds:

- exact checkpoint digest;
- tree size;
- leaf index;
- leaf digest; and
- inclusion-proof digest.

The leaf index must lie inside the named tree.

This is the future bridge from election evidence objects—ballots, manifests, ceremonies, challenges, audits—to a publicly reproducible checkpoint.

## Fork / split-view classification

Two checkpoint references at the same sequence are classified explicitly.

If they are byte-for-byte the same logical checkpoint, they are `SameCheckpoint`.

If they share the same election and predecessor but differ in root/checkpoint identity, they are `EquivocatingSuccessors`.

If they claim the same sequence but derive from different histories, they are `DivergentHistory`.

Neither case may be silently resolved by network arrival order.

## ELECT-010 — independent witness quorum

A witness attestation binds:

- exact checkpoint digest;
- witness key digest;
- **control-domain digest**;
- control-domain credential digest;
- observation-evidence digest; and
- attestation digest.

The critical field is the control domain.

Three keys operated by one organization are not three independent witnesses.

The quorum validator therefore counts both:

```text
unique witness keys
unique control domains
```

A frozen quorum policy specifies minimum total witness keys and minimum distinct control domains. `minimum_total_witnesses` may not be lower than `required_distinct_control_domains`.

## Reuse of Mycelix continuity-witness work

Mycelix Identity PR #753 already defines the right semantic distinction for continuity evidence: a human-facing domain label is not enough; multiple councils, hosts, subsidiaries, or notaries under one administrative root must share a canonical `control_domain_id` for independence counting.

ELECT-010 intentionally mirrors that theorem rather than inventing election-specific Sybil semantics.

This PR does not directly depend on #753 because that identity profile is separately stacked and not yet qualified/merged. Instead, election witness attestations carry a digest binding to a control-domain credential/profile. A later integration tranche should adapt the exact qualified #753 credential identity rather than duplicate its schema.

## Witness equivocation

`WitnessEquivocationEvidenceV1` records evidence that the same witness key/control domain attested to two different checkpoints for the same sequence.

That evidence must bind two distinct checkpoint digests and a separate evidence digest.

A later certification policy should treat unresolved witness/log equivocation as a hard block, not a warning.

## Why Holochain replication is not enough

Holochain remains useful as a replication and evidence-distribution substrate, but:

```text
DHT availability != append-only proof
DHT agreement    != complete public record
many agents      != independent control domains
```

The public election record must be exportable and independently verifiable without trusting Mycelix runtime behavior.

This is also why checkpoint digests and proof artifacts should eventually be mirrored through unrelated channels: civic organizations, universities, observer groups, offline archives, and ordinary HTTPS/static mirrors can all carry the same authenticated checkpoints.

## Connection to ELECT-008

The anonymous-authority tranche requires `NullifierCensusV1.complete_checkpoint_digest` before tally admission.

ELECT-009/010 supplies the missing assurance context for that digest:

```text
complete ballot/evidence census
        -> transparency checkpoint
        -> append-only consistency evidence
        -> independent witness quorum
        -> eligible input to tally verification
```

The current crates do not yet compose these into one runtime capability. ELECT-011 will define the standalone verifier contract that performs those joins without depending on Holochain.

## Deliberate non-claims

This tranche does not yet provide:

- a concrete Merkle/hash algorithm;
- canonical binary encoding;
- consistency-proof verification;
- inclusion-proof verification;
- a signature implementation;
- a gossip protocol;
- a timestamp authority;
- a globally complete log;
- a witness credential verifier;
- proof that a control-domain credential is truthful;
- distributed consensus;
- ballot cryptography; or
- election certification.

It defines the structural theorem that later implementations must qualify.

## Next tranche

Proceed with:

- **ELECT-011** minimal offline verifier contract and evidence-package manifest;
- **ELECT-012** physical ballot, batch, seal, custody, scanner/CVR reconciliation, and audit evidence types.

After those, Mycelix will have enough non-cryptographic election architecture to begin evaluating concrete ballot and anonymous-credential protocols against the complete evidence boundary.
