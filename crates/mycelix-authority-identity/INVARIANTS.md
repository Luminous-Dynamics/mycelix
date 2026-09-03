# Mycelix Authority Identity v0.1

Status: **normative pure canonicalization kernel**

This crate defines shared cryptographic semantic identities for institutional authority objects so revocation, delegation, executor, and audit adapters do not invent incompatible same-ID hashing rules.

v0.1 registers one profile:

`mycelix-authority-grant-v1-blake3-framed-semantic`

## Why this exists

`AuthorityGrant.id` is an identifier, not a content commitment.

Current-authority freshness must therefore never bind only a grant ID. If the same textual ID is reused with a different holder, capability, rulebook, delegation parent, lifetime, source, or proof, it must be a different authority identity.

The canonical identity is the bridge between the wire-neutral `mycelix-institutional-core` model and generation-bound freshness from PR #74.

## Included authority semantics

The grant identity binds:

- institutional-core protocol version;
- grant ID;
- holder principal;
- institution;
- optional jurisdiction;
- complete role set;
- complete capability set;
- exact rulebook ID/version/digest;
- complete authority-source set including source kind/reference/proof reference;
- issued-at time;
- expiry;
- optional delegation parent grant ID; and
- grant proof reference.

Changing any included semantic field changes the canonical digest.

## Set semantics

The institutional evaluator uses membership semantics for roles/capabilities and iterates sources without giving serialized vector position authority meaning.

Therefore:

- role order does not affect identity;
- capability order does not affect identity;
- authority-source order does not affect identity.

All three are canonicalized before hashing.

Duplicate roles, capabilities, or exact authority sources are rejected rather than silently allowing multiple serialized representations to inflate apparent authority or produce ambiguous audit encodings.

## Framing

Every variable-width field uses explicit length framing. Optional fields use a presence frame before their value.

The digest also commits both:

- the fixed domain separator `mycelix/authority-identity/grant/v1`; and
- the registered profile string.

No concatenation-with-delimiters or JSON object ordering is part of the canonical identity.

## Proof references remain identity-bearing

`AuthoritySourceRef.proof_ref` and `AuthorityGrant.grant_proof_ref` are included.

This is deliberately conservative: two otherwise identical grants backed by different exact institutional/proof provenance are distinct authority objects. A higher layer may group them as equivalent policy outcomes, but current execution authority must not silently substitute one proof lineage for another.

## Delegation

`delegated_from` is identity-bearing. Adding/removing/changing the parent grant therefore changes the child grant identity.

This does not itself prove delegation attenuation. The separate delegation-lineage kernel required by issue #73 must still prove parent/child capability, role, institution, rulebook, lifetime, freshness-generation, and cycle/depth constraints.

## Authority separation

Canonicalization does not grant authority. It proves only that independent components are talking about the same exact semantic authority object.

The crate contains no Holochain, policy evaluation, revocation selection, lifecycle claim, external effect, Phi, reputation, stake, or model-score input.
