# Integral Interop 1 — Source Snapshot Identity Binding

Status: **ReferenceModelOnly**

## Purpose

D6X and D6W carry source_dkg_snapshot_commitment as a semantic dependency identity. The current ReferenceModelOnly boundary does **not** claim that this commitment can reconstruct or independently validate the underlying DKG snapshot object; that object is outside this fixture's source boundary.

The security property implemented here is narrower and testable:

> A source snapshot identity is part of the D6X closure identity and the D6W input identity, and an older closure cannot be paired with a projection naming a different source snapshot.

This distinguishes **snapshot identity** from **runtime retrieval evidence**.

## Identity boundary

The source snapshot commitment participates in the D6X closure identity alongside:

- closure profile commitment;
- semantic environment commitment;
- derivation profile commitment;
- selected roots;
- typed selected node and edge dependencies;
- missing-dependency/resolution state;
- closure status and claim ceiling.

Therefore changing only source_dkg_snapshot_commitment produces a different D6X closure identity.

D6W copies the exact projection source snapshot commitment into InputCommitmentV1. At the stricter D6W qualified-consumption boundary, that source snapshot commitment must also be in the canonical D6S SHA-256 representation; symbolic/opaque snapshot labels are rejected. This is a representation/integrity gate, not proof that the underlying DKG snapshot is authoritative.

D6W additionally requires:

- the semantic environment itself to be structurally valid;
- the environment to explicitly carry a dependency snapshot root;
- that root to equal the projection source snapshot commitment exactly;
- the source snapshot commitment to use the canonical D6S SHA-256 representation;
- the D6X closure to be complete and valid;
- the closure projection commitment to equal the projection commitment;
- the closure source snapshot commitment to equal the projection source snapshot commitment;
- projection node and edge commitments to match their semantic sources.

An old closure cannot therefore be reused with a projection that names another source snapshot.

## Retrieval evidence is different

SemanticDependencyResolutionEvidenceV1 contains runtime/audit information such as retrieval references, observed commitments, and qualification-context commitments. That evidence is intentionally excluded from candidate-independent closure identity.

Changing a retrieval reference can therefore change the audit certificate while leaving the semantic closure identity unchanged, provided the semantic dependency itself has not changed.

This separation prevents a storage URI, resolver address, or runtime lookup record from silently becoming semantic identity.

## Environment binding

Where `SemanticEnvironmentV1.dependency_snapshot_root` is present, D6S requires the
projection's `source_dkg_snapshot_commitment` to equal that environment root.
This does not reconstruct the external DKG snapshot; it prevents a projection
from silently switching snapshot identity while retaining the same semantic
environment commitment.

A caller can therefore distinguish two cases:

- **bound identity:** the environment names the snapshot and D6S checks exact equality;
- **opaque identity:** the environment does not name one, so the external snapshot
  remains an explicit out-of-bound trust boundary.

The canonical receipt constructor uses the same source-binding check, so a
self-consistent projection cannot become a canonical D6S receipt merely by
recomputing its projection and receipt commitments.

D6W goes one step further: canonical source-snapshot representation is required
before the material can cross the downstream qualified-consumption boundary.
This deliberately preserves the D6S ReferenceModelOnly compatibility layer for
older symbolic fixtures while preventing those opaque identifiers from being
treated as qualified downstream source commitments.

## Trust in the environment

The equality check is only as authoritative as the supplied semantic environment.
D6S computes and checks the environment's commitment, but this reference model
does not authenticate who supplied that environment or prove that its
`dependency_snapshot_root` was obtained from a trusted DKG boundary. A caller
must authenticate or independently obtain the environment before treating its
snapshot root as authoritative. If that step is absent, the check establishes
internal consistency only—not source provenance.

## What this does not prove

This tranche does **not** prove that a supplied source snapshot commitment is backed by a particular external DKG snapshot object. That requires an explicit source-snapshot object and a separately specified commitment algorithm at the DKG boundary.

Accordingly, this remains ReferenceModelOnly; no Integral wire-schema or DKG snapshot format is inferred.

## Adversarial coverage

The conformance tests cover:

1. changing the source snapshot changes D6X closure identity;
2. pairing a changed projection with an old closure is rejected by D6W;
3. pairing a changed projection with a freshly derived closure yields a distinct D6X/D6W identity rather than silently retaining the old identity;
4. runtime resolution evidence does not become semantic identity.

The key invariant is **identity propagation without pretending to validate an out-of-bound source object**.

The D6W gate is intentionally stricter than the D6S reference-model boundary:
qualified downstream consumption requires an explicitly bound, canonically represented
snapshot identity. A canonical hash that is merely substituted into the projection
cannot pass by itself; it must agree with the supplied semantic environment.

## D6P qualified-admission boundary

D6W now distinguishes two admission paths for D6P receipt commitments:

- the ordinary D6W constructor verifies the receipt identifiers as canonical commitments and
  binds them into the qualified input identity;
- the authoritative D6P constructor additionally requires the D6X closure to be produced
  through `compute_dependency_closure_from_authoritative_d6p`, then reconstructs every
  D6P receipt named by the projection against a supplied D6P composition.

The second path is the provenance-bearing path when the caller has authoritative D6P
composition material. It is intentionally separate from commitment equality: a receipt
and composition can both be internally self-consistent without establishing that the
composition itself came from an authoritative D6N/D6O boundary.

This follows the same provenance distinction used by established provenance models such as W3C PROV: derivation/identity records describe how artifacts relate, while trust in the provenance record and its source must be established separately.

Consequently, D6W can now make a precise claim at each boundary:

- canonical D6P receipt identity: representation/integrity;
- authoritative-D6P admission: receipt-to-composition provenance reconstruction;
- authoritative D6N/D6O source: still an upstream trust boundary unless separately reconstructed.
