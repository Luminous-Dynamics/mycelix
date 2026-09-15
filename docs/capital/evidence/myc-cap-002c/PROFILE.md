# MYC-CAP-002C — Conserved Claim Registry v1

Status: executable candidate; qualification is exact-head CI evidence, not this document.

## Purpose

This profile proves one narrow property:

> a bounded investor claim may change holder or shape without changing aggregate project liability or touching the underlying commons asset.

It is a single-priority, single-currency claim registry bound to one exact MYC-CAP-002A parent transition receipt.

## Parent binding

The profile freezes:

- exact project and unit;
- exact Return Envelope subject SHA;
- exact parent financial profile SHA-256;
- exact semantic SHA-256 of the parent transition receipt;
- exact `remaining_claim_units`.

The registry is stale if the parent financial receipt changes. v1 does not contain a retirement operation and cannot mutate the Return Envelope.

## UTXO-like claim slices

Every registry operation consumes current claim slices and creates new slices.

The allowed operation kinds are:

```text
GenesisIssue
Transfer
Split
Merge
```

`GenesisIssue` occurs exactly once and creates total face value equal to the parent remaining claim.

All later operations conserve exact integer face value:

```text
sum(consumed face units) == sum(created face units)
```

The aggregate active registry must always equal the frozen parent remaining claim.

## Single-purpose operation semantics

`Transfer` is the only operation that may change holder.

`Split` preserves the current holder while dividing one slice.

`Merge` requires all consumed slices to have one current holder and preserves that holder.

This prevents split/merge from becoming hidden assignment mechanisms.

Consumed slices become stale immediately. Slice IDs and operation IDs are never reusable.

## Secondary price isolation

`transaction_price_units` records the private transfer price but has no effect on project liability.

Therefore:

```text
sale price
!= face value
!= new project principal
!= new Return Envelope
```

A 73M-unit claim sold for 60M or 100M remains a 73M project claim.

## Asset boundary

The registry contains no operation that transfers, mortgages, removes the lock from, or otherwise disposes of the protected commons asset.

```text
claim liquidity != asset liquidity
```

Claim ownership also does not imply stewardship authority, operator authority, or legal title to public infrastructure.

## Current holder semantics

Holder state is reconstructed from the full operation lineage. Possession of an old serialized slice does not make that slice current after it has been consumed.

The authority/evidence references are bounded pointers only. v1 does not authenticate the external identities or signatures they reference.

## Deliberate exclusions

Separate future profiles should own:

- claim retirement against a newer parent financial receipt;
- multiple seniority classes;
- default interest or lender priority;
- token/bearer instruments;
- securities-law/transfer-restriction logic;
- cross-currency claims;
- securitization/tranching;
- legal negotiability;
- collateral/step-in rights under MYC-CAP-002J.

## Nonclaims

PASS proves only exact face-value conservation and current holder/slice reconstruction under this frozen profile and supplied evidence. It does not establish securities-law compliance, legal ownership or negotiability, market price/fair value, tax treatment, lender priority, external identity authenticity, asset title, community governance authority, or infrastructure performance.
