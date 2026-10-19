# ECON-001 — Canonical Instrument Constitution

**Status:** Draft constitutional/economic semantics. This tranche changes documentation and machine-readable declarations only; it does not change runtime balances, fees, minting, staking, bridge behavior, or governance authority.

## Purpose

Mycelix currently contains several historical generations of economic terminology. Some older material describes MYCEL as a transferable commons currency or governance stake and places demurrage on TEND. The current Finance implementation is already closer to a safer separation: SAP carries demurrage, TEND is zero-sum mutual credit, and MYCEL is non-transferable.

ECON-001 freezes the canonical semantic model before further economic hardening.

## Constitutional model: three lanes, not three interchangeable currencies

| Instrument | Canonical role | Transfer semantics | Balance semantics | Demurrage | Civic/governance effect |
| --- | --- | --- | --- | --- | --- |
| **SAP** | settlement / accounting / exchange | transferable value | non-negative monetary balance | policy-eligible | no automatic civic power |
| **TEND** | reciprocity / mutual credit / time exchange | reciprocal ledger transfer | signed, network-zero-sum credit/debit | forbidden | no automatic civic power |
| **MYCEL** | contextual standing / credentials | non-transferable | not money | not applicable | bounded role eligibility only; never fundamental civic sovereignty |

The canonical protocol symbol is **MYCEL**. `MYCL` may appear as informal shorthand in discussion, but it is not a second wire denomination, asset, or ledger identity.

## Separation invariants

1. **Transferable wealth is not standing.** SAP ownership, spending volume, gifts, investment success, or fees paid SHALL NOT automatically mint or increase MYCEL.
2. **Reciprocity is not standing.** TEND balances, exchange volume, or service ratings SHALL NOT automatically mint or increase MYCEL. A service event may become independently reviewed evidence relevant to a bounded domain credential, but no fixed conversion rate exists.
3. **Standing is not money.** MYCEL SHALL NOT be transferable, sold, collateralized, redeemed, or converted into SAP or TEND.
4. **TEND is mutual credit, not savings money.** TEND SHALL remain signed and zero-sum across its clearing domain; positive balances are matched by negative balances. TEND does not implement demurrage.
5. **SAP is the settlement lane.** SAP is the Mycelix-native transferable value/accounting instrument. Demurrage, where ratified, acts on eligible SAP balances rather than changing the meaning of one SAP unit of account.
6. **No implicit cross-lane conversion.** No payment, gift, reciprocal exchange, stake, score, or standing update may silently convert one lane into another.
7. **No tokenized human worth.** MYCEL is not a measure of intrinsic human value, moral worth, universal competence, or entitlement to fundamental rights.
8. **Equal civic standing remains prior.** Fundamental civic rights and constitutional standing are not purchasable with SAP, earned through TEND, or rank-ordered by MYCEL.

## Legacy semantics explicitly superseded as design targets

The following historical ideas are not part of the canonical target architecture:

- MYCEL as a transferable commons currency;
- MYCEL staking as a direct source of political/governance weight;
- TEND demurrage;
- fixed SAP↔TEND↔MYCEL exchange rates;
- automatic SAP donation/payment → MYCEL standing;
- automatic TEND activity/quality score → MYCEL standing;
- universal reputation scoring as a proxy for human worth or general competence.

This declaration does **not** claim all existing runtime code already satisfies these rules. Any implementation that still contains one of these couplings is migration debt to be handled explicitly in a later, separately reviewable tranche.

## Instrument admission rule

Mycelix SHOULD resist adding a fourth foundational economic instrument.

A new monetary instrument is admissible only when all of the following are established:

1. the required semantics cannot be represented safely with SAP, TEND, MYCEL, or a typed claim/right/pool/receipt;
2. it has genuinely distinct settlement semantics rather than a new brand or incentive wrapper;
3. its safety and capture model is explicit;
4. its interoperability benefit exceeds fragmentation and liquidity costs;
5. it cannot become a hidden route from wealth to civic sovereignty.

The default representation for new mechanisms SHOULD be, in order of preference: a typed right, claim, obligation, pool, attestation, receipt, or contract. Tokenization requires additional justification.

## Relationship integrity

The economic lane used to record value must not erase the social/legal relationship:

- exchange remains exchange;
- reciprocity remains reciprocity;
- gift remains gift;
- care must not automatically create recipient debt;
- stewardship remains a bundle of rights and duties rather than ownership-by-score.

In particular: **need is not debt**. Care Commons or humanitarian support may compensate providers in SAP or record voluntary reciprocity without debiting the recipient merely because support was received.

## Authority boundary

ECON-001 is declarative. It does not:

- change any current balance;
- migrate any ledger;
- change the 2% SAP demurrage implementation;
- change TEND credit limits;
- remove current fee tiers or staking behavior;
- decide how SAP reserves/redemption work;
- define a universal MYCEL score;
- authorize automated governance decisions.

Those changes require separate implementation/evidence tranches.

## Planned follow-on sequence

- **ECON-002 — Conversion Firewall:** prevent automatic SAP/TEND/MYCEL conversion and automatic economic-activity→standing promotion.
- **ECON-003 — SAP Unit/Balance Separation:** distinguish the SAP numeraire from demurrage-bearing balances; specify issuance/redemption/reserve/liquidity invariants.
- **ECON-004 — TEND Reciprocity Kernel:** strengthen exact zero-sum accounting, bounded counter-cyclical credit, federation clearing, and care/reciprocity separation.
- **ECON-005 — MYCEL Credential Fabric:** make domain credentials/evidence primary and any aggregate score explicitly derived, bounded, inspectable, and appealable.
- **ECON-006 — Typed Claims Kernel:** represent equity, debt, bonds, insurance, escrow, stewardship rights, subscriptions, grants, pensions, and infrastructure claims without new currencies.

## Qualification semantics

Because ECON-001 does not change runtime code, its qualification target is semantic consistency, reviewability, and machine-readable agreement with `instrument-constitution-v1.json`. No runtime PASS is claimed by this document.
