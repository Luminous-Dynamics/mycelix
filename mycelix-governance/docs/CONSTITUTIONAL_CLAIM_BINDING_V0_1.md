# Constitutional Claim Binding v0.1

Status: experimental, non-activating semantic/refinement tranche.

## Purpose

The qualified constitutional consumption, temporal-provenance, and claim-lifecycle models use a logical `claim_id` as part of their abstract identity. Runtime finality cannot safely infer that an arbitrary identifier commits to the complete semantic claim body.

This tranche introduces an explicit, versioned `ClaimBinding` and a strict bound-authority path without rewriting the already-qualified legacy reference APIs.

## ClaimBinding

A binding contains every field that changes what a consumption claim means:

- schema version;
- claim ID;
- canonical authorization-envelope digest/reference;
- nonce;
- use index;
- jurisdiction/finality domain;
- MatterId namespace + stable ID;
- target digest;
- payload digest;
- shared usage-budget ID.

`ClaimBinding::from_claim()` is structural. `canonical_bytes()` is deterministic, domain-separated, fixed-order, big-endian for integers, and length-prefixes variable UTF-8 fields. It is intentionally not JSON/Serde encoding.

Hash/signature algorithm selection is a runtime policy. The pure semantic layer defines exactly **what bytes** must be committed, not which cryptographic algorithm a deployment must use.

## Additive migration

This tranche does not silently reinterpret legacy qualified objects.

Legacy:
- `FinalityProof`;
- `FinalityEvidence`.

Strict authority wrappers:
- `BoundFinalityProof { proof, claim_binding }`;
- `BoundFinalityEvidence { evidence, claim_binding }`.

Legacy finality evidence may remain retained for historical/reference semantics. Boundness is **non-retroactive**: an evidence ID first observed through the unbound path cannot later acquire a binding registry entry. It must remain unbound historical evidence. **Retained unbound evidence is not lifecycle authority.**

## Authority chain

The high-assurance lifecycle path requires one identical binding through:

```
submitted ConsumptionClaim
      ↓ from_claim()
ClaimBinding
      ↓
BoundFinalityProof
      ↓
BoundFinalityEvidence
      ↓
TemporalEvidenceState.finality_bindings[evidence_id]
      ↓
ClaimLifecycleStatus::Finalized / RejectedConflict
      ↓
ClaimTransitionReceipt
```

Lifecycle finalization fails closed unless:

- accepted temporal evidence exists;
- the temporal binding registry contains that evidence ID;
- temporal binding equals the submitted claim's binding;
- bound proof binding equals the submitted claim's binding;
- proof/evidence claim IDs, proof IDs, profile, and effective sequence agree.

## Canonicalization rule

Domain separator:

`MYCELIX-CONSTITUTIONAL-CLAIM-BINDING\0V1\0`

Encoding order:

1. schema version — u16 big-endian;
2. claim ID — u32 length + bytes;
3. envelope digest — u32 length + bytes;
4. nonce — u32 length + bytes;
5. use index — u32 big-endian;
6. jurisdiction — u32 length + bytes;
7. MatterId namespace — u32 length + bytes;
8. MatterId stable ID — u32 length + bytes;
9. target digest — u32 length + bytes;
10. payload digest — u32 length + bytes;
11. budget ID — u32 length + bytes.

Any future schema change must use a new schema version/domain contract. Do not append silently to V1.

## Required safety properties

1. Copying `claim_id` onto a different claim body cannot reuse a bound proof.
2. Mutating envelope, nonce, use index, jurisdiction, MatterId, target, payload, or budget while retaining `claim_id` invalidates bound proof validation.
3. Temporal evidence with the correct proof ID but wrong binding cannot become lifecycle authority.
4. Duplicate bound evidence is idempotent only when both evidence and binding agree exactly; a previously observed unbound evidence ID cannot be upgraded into bound authority.
5. The same evidence ID cannot be rebound to a different binding.
6. Lifecycle finalized/winner state retains the exact binding.
7. Transition receipts retain the exact binding of the claim whose transition they record.
8. Legacy unbound evidence cannot satisfy the lifecycle's bound-finality requirement.

## Formal/refinement boundary

The bounded TLA+ lifecycle model treats claims as atoms. The concrete refinement obligation is therefore:

```
one formal Claim atom
    -> one unique authenticated Rust ClaimBinding
```

A runtime refinement claim fails if two semantically different accepted concrete claim bindings can map to the same formal claim identity.

`ConstitutionalClaimBinding.als` checks the concrete-identity refinement at a bounded relational scope. It deliberately admits two claims sharing the same legacy `claim_id` while differing in payload, then asserts that an exact full-field binding can authenticate at most one Claim atom. Qualification must also run a mutant that removes one security-relevant field from `Matches`; that mutant must produce a counterexample.

This tranche does not claim that canonical bytes are cryptographically authenticated merely because they exist. Runtime witness/consensus/signature adapters must authenticate the canonical binding or a typed digest of it.

## Evidence boundary

No Holochain/runtime activation, cryptographic-algorithm mandate, distributed consensus proof, or unbounded theorem is established here. This tranche must receive exact-head Rust qualification, including mutation sensitivity, before #1333 or #1191 can treat claim identity as runtime-safe.
