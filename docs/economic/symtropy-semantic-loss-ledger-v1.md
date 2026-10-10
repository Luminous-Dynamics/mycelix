# Symtropy Cross-Ontology Semantic-Loss Ledger v1

Status: bounded interoperability reference; not a financial/accounting/legal qualification.

## Purpose

Every projection from a Symtropy simulation event into Mycelix, Valueflows, Integral COS, TEND, or accounting must carry an explicit semantic-loss record.

The ledger prevents a target representation from being mistaken for the source simulation truth and prevents adapters from silently widening claims.

## Required trace

```text
source_event_id
-> source_origin
-> simulation/world/episode/build/replay identity
-> adapter_profile
-> target_id
-> target_type
-> semantic disposition
-> target recognition/settlement record (if any)
```

Every target must remain reversible to the source event identity.

## Dispositions

| Disposition | Meaning | Allowed as clean projection? |
|---|---|---|
| Preserved | Exact source semantics survive | yes |
| Derived | Target meaning is computed under an explicit target policy | yes, as derived |
| BoundedLoss | Some source semantics cannot be represented and loss is explicit | only with bounded claim |
| Unmapped | Target concept intentionally not created | yes, if the omission is explicit |
| Conflict | Source or mapping is stale, conflicting, indeterminate, or contradictory | no |

## Canonical mappings

### Valueflows

Valueflows distinguishes Intent (desired/planned), Commitment (promised), Claim (future reciprocal claim), and EconomicEvent (actual economic flow). Therefore:

- Symtropy OfferCreated -> Valueflows Intent.
- Symtropy OfferAccepted -> Valueflows Commitment only when the simulation event is being represented as a promise/commitment.
- Actual simulated flows may project to a Valueflows EconomicEvent representation, but the result remains simulation-scoped.

Do not map planning or acceptance automatically to actual settlement.

### Integral

Symtropy -> Integral COS is an observation/interoperability projection.

```text
COS observation != ITC entitlement
COS observation != governance authorization
simulation production != physical production
```

Any ITC or governance effect requires the separate exact Integral policy/authority path.

### TEND

Symtropy -> TEND is an instrument-specific candidate/projection boundary.

```text
TEND transaction candidate != authorization
authorization != settlement
settlement != world-state outcome
```

TEND identity, unit, issuer/steward, issuance rules, validity, source world, and transaction identity must remain explicit.

### Accounting

Accounting output is a derived projection.

```text
simulation event -> accounting projection
!= physical evidence
!= accounting compliance
!= settlement finality
```

## Anti-collapse invariants

1. simulation event != physical observation;
2. simulation ownership != legal ownership;
3. simulated currency != external currency;
4. replay equality != independent verification;
5. recognition != origin rewrite;
6. adapter projection != source mutation;
7. market acceptance != settlement;
8. transport acknowledgment != settlement receipt;
9. correction adds lineage instead of rewriting history;
10. stale/conflicting/indeterminate evidence cannot become a clean projection;
11. production observation does not itself create economic entitlement;
12. accounting projection cannot become physical evidence.

## Why this matters

The current Valueflows specification explicitly models actual EconomicEvent flows separately from Intent and Commitment, and defines corrections/reversals as part of the event model. The Mycelix adapter should preserve those distinctions rather than flattening them into one generic transaction type. citeturn0search0turn0search6

## Claim ceiling

A passing semantic-loss ledger can establish only:

```text
provenance-preserving, bounded cross-ontology semantic projection
```

It does not establish economic performance, physical-world occurrence, legal title, accounting compliance, currency equivalence, settlement finality, or Integral policy adoption.