# D3 — COS observation → ITC projection

## Purpose

D3 extends the Integral reference-node path from COS semantic admission into a bounded ITC accounting projection.

The critical boundary is:

```
COS admission
NaN
NaN
NaN
```

Integral's COS owns operational/task facts while ITC owns contribution accounting. The reference seam therefore requires an explicit, source-bound COS **Observation** as its input. A successful OAD/CDS/COS admission or production authorization cannot itself create contribution evidence.

## Executable model

The reference implementation models:

1. a versioned COS observation;
2. source attribution and evidence reference;
3. observed quantity;
4. uncertainty;
5. local/foreign origin;
6. an ITC projection that preserves the source observation identity;
7. an immutable observation binding carried into the projection;
8. idempotent replay of the same logical observation;
9. fail-closed rejection when the same observation identity is reused with changed payload.

The observation binding includes the observation identity, work, actor, generation, quantity, timestamp, evidence reference, origin, and uncertainty state. This prevents a retry/reconciliation path from silently changing who did what, what was observed, or which evidence supported it.

## Fail-closed cases

The model rejects:

- missing observations;
- authorization presented as observation;
- execution intent presented as observation;
- stale observation generation;
- observations without evidence references;
- empty source/work/actor/accounting identities;
- duplicate logical observation IDs whose payload differs from the existing projection.

It preserves:

- foreign source origin;
- uncertainty;
- observation identity;
- work identity;
- actor identity;
- evidence reference.

A repeated observation with the same complete source payload returns `Replayed` rather than creating a second semantic contribution.

## Why this matters

The eventual cockpit can now distinguish:

**What was admitted? → What was authorized? → What was actually observed? → What accounting projection follows?**

This prevents a successful design/admission path from becoming a fabricated claim that work happened or that contribution credits were earned. It also prevents retry/reconciliation from becoming a hidden mutation channel.

## Claim ceiling

`ReferenceModelOnly`.

This does not establish Integral ratification, real-world work verification, ITC economic policy, credit issuance, ledger correctness, worker compensation, or economic outcomes.

## Source alignment

Integral's public developer guide explicitly describes interfaces as formal contracts between systems and keeps COS task/workflow responsibility distinct from ITC credit-balance responsibility. citeturn0search0