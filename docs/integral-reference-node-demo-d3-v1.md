# D3 — COS observation → ITC projection

## Purpose

D3 extends the Integral reference-node path from COS semantic admission into a bounded ITC accounting projection.

The critical boundary is:

```
COS admission
!= observation
!= ITC valuation
!= ledger issuance
```

Integral's COS owns operational/task facts while ITC owns contribution accounting. The reference seam therefore requires an explicit, source-bound COS **Observation** as its input. A successful OAD/CDS/COS admission or production authorization cannot itself create contribution evidence.

## Executable model

The reference implementation models:

1. a versioned COS observation;
2. source attribution and evidence reference;
3. observed quantity;
4. uncertainty;
5. local/foreign origin;
6. an ITC projection that preserves the source observation identity.

The projection remains a **reference-model accounting projection**. It does not implement Integral's final ITC valuation policy and does not mint ledger authority.

## Fail-closed cases

The model rejects:

- missing observations;
- authorization presented as observation;
- execution intent presented as observation;
- stale observation generation;
- observations without evidence references.

It preserves:

- foreign source origin;
- uncertainty;
- observation identity.

## Why this matters

The eventual cockpit can now distinguish:

**What was admitted? → What was authorized? → What was actually observed? → What accounting projection follows?**

This prevents a successful design/admission path from becoming a fabricated claim that work happened or that contribution credits were earned.

## Claim ceiling

`ReferenceModelOnly`.

This does not establish Integral ratification, real-world work verification, ITC economic policy, credit issuance, ledger correctness, worker compensation, or economic outcomes.

## Source alignment

Integral's public developer guide explicitly describes interfaces as formal contracts between systems and keeps COS task/workflow responsibility distinct from ITC credit-balance responsibility. citeturn0search0
