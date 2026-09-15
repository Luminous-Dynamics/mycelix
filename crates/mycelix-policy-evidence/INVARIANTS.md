# Mycelix Policy Evidence v0.1 Invariants

## Purpose

`mycelix-policy-evidence` is a pure cross-domain waist between a domain's canonical policy semantics and later policy-currentness / authority composition.

It proves only that two independently shaped evidence domains bind the same exact policy identity:

1. authentication of one immutable policy record; and
2. verification of institutional adoption of that exact policy identity.

It does **not** define policy semantics, choose a governing policy, prove generation currentness, interpret law, grant administrative authority, grant execution authority, or create an external effect.

## PE-001 — Semantic identity is upstream

The domain supplies an exact `PolicyIdentityRef { digest, profile }` produced by its own canonical semantic-identity theorem.

This crate never hashes policy semantics and never infers policy identity from a locator, record, institution, timestamp, verifier name, or adoption proof.

A zero digest, empty/oversized/control-bearing profile, or malformed subject fails closed.

## PE-002 — Record authenticity and adoption are different facts

`PolicyRecordVerificationEvidence` and `PolicyAdoptionVerificationEvidence` are distinct wire types with distinct protocol identifiers.

A record-authenticity receipt cannot satisfy the adoption qualifier, and an adoption receipt cannot satisfy the record qualifier.

The qualified join requires both.

## PE-003 — Immutable record evidence binds an exact record

A record subject binds:

- exact policy digest/profile; and
- exact immutable `policy_record_ref`.

The verification evidence additionally carries an exact record-proof ref, verifier ref, verification ref, and bounded `EvidenceLease`.

Moving or mirroring semantically identical policy content to another record is a different record-authenticity subject even though the upstream semantic policy identity may remain unchanged.

## PE-004 — Institutional adoption is typed and exact

An adoption subject binds:

- exact policy digest/profile;
- exact `InstitutionId`;
- optional exact `JurisdictionId`;
- exact `RulebookRef` including digest;
- exact institutional `authority_ref`; and
- exact `adoption_proof_ref`.

A proof for another institution, jurisdiction, rulebook generation/content, authority reference, adoption reference, or policy identity fails closed.

The adoption subject is an expectation supplied by the consuming domain from its policy semantics / institutional context. This crate does not decide which institution or rulebook ought to govern a policy.

## PE-005 — Receipt origin is still external

The transport evidence types are serializable/deserializable. Deserialization is not proof.

Local qualification validates shape, exact subject binding, and evidence lifetime. It does **not** prove that the bytes really came from the named record verifier or adoption verifier.

Live runtimes must obtain these receipts from independently qualified verifier boundaries and must not treat arbitrary caller-provided receipt bytes as sufficient positive authority.

## PE-006 — Proof lifetime reuses the existing monotone lease algebra

This crate does not create another validity-window composition rule.

Every evidence receipt carries `mycelix_authority_evidence_lease::EvidenceLease`.

The joined evidence lease is exactly the existing monotone intersection:

```text
verified_at(join) = max(record.verified_at, adoption.verified_at)
valid_until(join) = min(record.valid_until, adoption.valid_until)
```

Expired, future-verified, inverted, zero-time, or otherwise invalid leases fail through the shared lease theorem.

No qualification may widen either evidence horizon.

## PE-007 — Distinct verifier domains are mandatory at the join

The v0.1 joined theorem requires the record verifier reference and adoption verifier reference to be distinct.

This prevents one verifier identity from silently filling both evidence roles. It is a minimum separation rule, not a complete organizational/fault-domain-diversity theorem.

A higher-assurance profile may additionally require different operators, toolchains, trust roots, evidence sources, or review processes.

## PE-008 — Positive local results are non-deserializable

`QualifiedPolicyRecordEvidence`, `QualifiedPolicyAdoptionEvidence`, and `QualifiedPolicyEvidenceBundle` are process-local typestate results and do not implement `Deserialize`.

Persisted transport receipts must be requalified instead of being reloaded as already-positive objects.

## PE-009 — Currentness remains separate

A valid record plus valid adoption proof does not establish that the policy is the current governing policy.

A policy may be authentic and historically adopted while later revoked, superseded, replaced, or outside the relevant current generation.

Therefore:

```text
canonical policy identity
!= immutable record authenticity
!= institutional adoption
!= generation currentness
!= policy authority
```

Currentness must be established by a separate closed-world / generation-bound theorem appropriate to the domain.

## PE-010 — No authority amplification

Every positive type in this crate permanently reports false for authority amplification.

In particular, policy evidence does not itself grant:

- institutional authority;
- administrative decision authority;
- review/finality authority;
- execution authority; or
- external-effect authority.

## PE-011 — No ambient authority or runtime I/O

The crate is pure Rust. It contains no HDK/Holochain, DHT lookup, database, filesystem, network, process, environment, wall-clock read, randomness, policy selection, current-record/latest heuristic, actuator, or effect path.

Qualification receives an explicit `now_ms` only to validate already-supplied dynamic evidence leases.

## Intended composition

```text
domain policy semantics
        ↓
canonical PolicyIdentityRef
        │
        ├── exact immutable record subject
        │       + record verification evidence
        │       → QualifiedPolicyRecordEvidence
        │
        └── exact typed institutional adoption subject
                + adoption verification evidence
                → QualifiedPolicyAdoptionEvidence
                        ↓
                exact identity equality
                + distinct verifier domains
                + EvidenceLease intersection
                        ↓
                QualifiedPolicyEvidenceBundle
                        ↓
                later domain-specific currentness / authority theorem
```

The bundle is reusable evidence for the next theorem. It is not the next theorem itself.
