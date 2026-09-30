# Integral Interop 1 — D6X commitment-binding boundary

Status: ReferenceModelOnly.

## Purpose

D6X now treats a selected qualified node or edge commitment as an integrity binding rather than an opaque non-empty string.

For canonical SHA-256 commitments emitted by the D6S commitment helper:

- a node commitment must equal the recomputation over its selected node binding fields;
- an edge commitment must equal the recomputation over its selected edge binding fields;
- a stale commitment paired with changed semantic fields is rejected before the dependency enters D6X closure identity;
- irrelevant graph material remains outside the semantic boundary and therefore does not need to participate in D6X closure binding.

This is an integrity rule only. It does not establish truth, authority, causality, certification, current-finality, authorization, or actuation.

## Node binding

The D6X ReferenceModelOnly binding currently covers:

- node identifier;
- underlying content commitment;
- node kind.

The resulting node binding is domain-separated under `integral-interop-1-node`.

Changes to historical/currentness metadata are handled separately by D6X currentness semantics and are not silently folded into the node binding.

## Edge binding

The D6X ReferenceModelOnly binding currently covers:

- edge identifier;
- source node identifier;
- target node identifier;
- edge kind.

The resulting edge binding is domain-separated under `integral-interop-1-edge`.

An edge selected by the closure profile must pass this binding check before it can enter the selected dependency set.

## Legacy symbolic commitments

Older ReferenceModelOnly unit fixtures use symbolic commitments such as `commit-root` and `edge-e1`. These are retained as compatibility fixtures.

Binding verification is therefore enforced when a commitment has the canonical 64-hex SHA-256 representation produced by the D6S helper. This keeps existing reference fixtures executable while making real canonical commitments fail closed when stale.

A future promotion beyond ReferenceModelOnly should remove this compatibility path and require canonical commitment encoding for every qualified node and edge.

## Adversarial coverage

The Integral cross-layer corpus now includes explicit stale-binding tests:

1. mutate selected node semantic content while retaining its previous canonical commitment;
2. mutate selected edge semantics while retaining its previous canonical commitment;
3. assert the selected object reports a binding mismatch;
4. assert D6X returns no closure certificate.

The positive mutation path recomputes the selected commitment after a semantic change and therefore continues through D6X/D6W propagation.

## Security rationale

A hash commitment is useful for integrity only if verifiers can establish what was committed. NIST describes SHA-256 as a digest mechanism for detecting message changes; the security property is therefore meaningful here only when the digest is bound to a defined semantic serialization. (NIST FIPS 180-4 / SHA-256)

This boundary is intentionally narrower than generic JSON canonicalization. RFC 8785 defines recursive UTF-16 property ordering and deterministic JSON serialization, while D6S-CANON-1 remains its own versioned contract with its integer-only numeric rule. (RFC 8785)

## Interoperability boundary

Integral's public developer guide describes OAD → COS as a data contract in which the Certified Design Package supplies the production-plan inputs. The public technical material remains a development/reference surface rather than a ratified wire schema, so this fixture remains ReferenceModelOnly. (Integral developer guide and public OAD documentation)
