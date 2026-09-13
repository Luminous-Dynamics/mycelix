# Procedure Policy Identity v0.1 Invariants

This crate adds canonical semantic identity for ADMIN-002 procedure policies without changing the qualified ADMIN-002 semantic kernel.

## Core theorem

`policy locator != policy semantic content != verified content identity != authoritative current policy != administrative authority`

A successful `QualifiedProceduralPolicyIdentity` proves only that one supplied `ProceduralCompletenessPolicy` carries the registered digest/profile for its own semantic obligations.

It does not prove that the policy is current, legally valid, institutionally adopted, jurisdictionally applicable, or authority-bearing.

## Canonical profile

Profile: `mycelix-administrative-procedure-policy-v1-blake3-framed-semantic`

Domain: `mycelix/administrative-procedure/policy/v1`

The digest length-frames every variable-width value and binds, in fixed order:

1. canonical identity profile;
2. ADMIN-002 protocol version;
3. procedure profile;
4. exact required notice-recipient set in canonical lexical order;
5. exact required response-recipient set in canonical lexical order;
6. response-mode discriminator;
7. minimum response-window milliseconds; and
8. reasons-requirement discriminator.

Recipient vector order has no semantics. Duplicate recipients remain invalid under ADMIN-002 and are rejected before hashing.

## Deliberately excluded

`policy_ref` is provenance/locator metadata and does not alter semantic identity.

`policy_digest` and `policy_digest_profile` are the identity claim being verified. They cannot be inputs to their own digest.

## Qualification

`qualify_procedural_policy_identity` requires the registered profile and exact recomputed semantic digest. A digest copied from one policy cannot qualify altered procedure semantics.

The positive typestate is intentionally not Clone/Serialize/Deserialize and grants neither institutional authority nor external-effect authority.

## Currentness boundary

This crate does not resolve which policy is current. A later authoritative provider must bind at least institution, jurisdiction, rulebook/procedure profile, exact canonical policy identity, generation/currentness evidence, effective interval, and provider proof.

No caller may infer current authority merely from successful semantic identity verification.

## Qualified-parent preservation

This tranche is rooted directly at ADMIN-002 exact head `4584122d271dcc7bd15274361e0f900e7bb37fe6` and CI pins the qualified administrative crate's Cargo manifest, hardened facade, and ADMIN-002 semantic implementation blobs. Canonical identity is additive evidence, not a rewrite of the qualified procedure theorem.
