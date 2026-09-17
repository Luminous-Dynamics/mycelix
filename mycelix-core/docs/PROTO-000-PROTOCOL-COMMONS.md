# PROTO-000 — Protocol Commons Constitution

Status: candidate source specification for PROTO-000 through PROTO-003.

## Scope

This tranche defines the constitutional interoperability thin waist only:

- exact protocol-profile identity and content commitments;
- layered capability declarations;
- critical-semantic classification;
- deterministic bounded negotiation;
- unknown-extension handling; and
- explicit downgrade evidence.

It does **not** establish registry legitimacy, standards-body authority, conformance certification, implementation independence, deployment security, legal validity, constitutional legitimacy, or any execution authority.

## Constitutional invariants

1. **Protocol is not authority.** Interoperability, implementation count, registry presence, conformance, market share, or protocol authorship cannot mint political, legal, physical, emergency, or execution authority.
2. **Compatibility is layered.** Wire compatibility does not imply semantic, policy, authority, privacy, evidence, lifecycle, or effect compatibility.
3. **Unknown critical semantics fail closed.** Unknown privacy, rights, authority, safety, or effect semantics cannot silently become absence of constraint.
4. **Downgrade is explicit.** Compatibility pressure cannot silently weaken a locally required protocol floor. Any permitted downgrade binds external authority evidence and records semantic loss.
5. **Popularity is not legitimacy.** Extension adoption, registry allocation, implementation count, stake, reputation, or market share cannot create authority semantics.
6. **Conformance is bounded evidence.** Passing a conformance suite establishes only the exact declared test claim; it does not establish security, rights compatibility, semantic completeness, or authority.
7. **Specification is not implementation.** Reference implementation behavior cannot silently redefine normative semantics.
8. **History is not rewritten.** Negotiation and downgrade receipts remain historical facts even after profiles are superseded.

## PROTO-001 — Exact profile identity

`ProtocolProfileV1` binds the exact core version, capability sets, extension semantics, semantic-module identifiers, authority/privacy/evidence vocabularies, canonicalization profile, security suite, conformance profile, and deprecation horizon.

Its SHA-256 digest is a deterministic content commitment over the serialized profile. The digest is **not** a signature, attestation, registry blessing, independence proof, or authority decision.

## PROTO-002 — Capability negotiation

Peers negotiate only profile pairs whose core, authority/privacy/evidence vocabularies, canonicalization profile, and security suite match exactly in this tranche. Every required capability must be available on the other peer at the exact declared version.

Selection uses explicit peer preference ranks. A higher rank means only “preferred by this peer”; it is not a truth, safety, recency, or legitimacy score.

## PROTO-003 — Unknown semantics and downgrade

Extensions are classified as advisory, feature, evidence, privacy, rights, authority, safety, or effect semantics.

Privacy, rights, authority, safety, and effect semantics are critical. A profile declaring such an extension as safely ignorable is invalid.

Required extensions may not silently degrade to ignore or feature-unavailable behavior.

A local minimum profile preference is an explicit negotiation floor. Crossing that floor requires an externally established `DowngradePermitV1`. This module records the referenced authority decision but does not verify or create that authority.

## Non-claims

A successful `NegotiatedProtocolReceiptV1` does not establish that either peer is trustworthy, that extension declarations are registry-qualified, that the selected security suite is secure, that implementations are independently derived, that legal or policy requirements are satisfied, that a deployment is safe, that any downstream effect is authorized, or that either peer has governance authority over the other.

Those claims belong to their respective authority, assurance, privacy, runtime, and later PROTO tranches.
