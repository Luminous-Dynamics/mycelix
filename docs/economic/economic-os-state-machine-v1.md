# Economic OS State Machine v1

Status: bounded executable reference.

## Design principle

The Economic Fabric must not model settlement as a single boolean or direct transition from evidence. It is a typed state machine whose transitions require explicit policy, evidence validity, authority and outcome conditions.

## States

| State | Meaning |
|---|---|
| Observed | source activity has been observed |
| Evidenced | observation has sufficient bound evidence for the current profile |
| Valuated | a versioned valuation/policy has been applied |
| Entitled | an explicit policy has created an entitlement/obligation |
| Authorized | an authorized actor/process has approved the scoped effect |
| Submitted | settlement/effect has been submitted with an idempotency identity |
| SettlementEligible | all current pre-settlement predicates have been satisfied |
| Settled | recipient/provider settlement receipt exists |
| Disputed | settlement or underlying meaning is under dispute |
| Indeterminate | outcome cannot yet be established |
| Corrected | a new record corrects prior meaning without mutating history |
| Reversed | a compensating/reversal event has been recorded |

## Critical improvement

Settlement eligibility is now a distinct state.

Therefore:

```text
Authorized -> Submitted -> SettlementEligible -> Settled
```

is not equivalent to:

```text
Authorized -> Settled
```

`SettlementEligible` requires:

- current source evidence;
- versioned policy;
- explicit authority;
- resolved dispute state;
- preserved claim ceiling.

## Integral profile

For Integral ITC, the generic state machine does not decide the ITC formula. It only requires that the external Integral policy be explicit and versioned.

```text
COS evidence
    ↓
Integral ITC policy
    ↓
ITC entitlement
    ↓
explicit authority/effect path
```

ITC remains source-owned and instrument-specific. Integral's current public ITC description states that ITC is a contribution accounting system rather than a currency and describes non-transferability and decay as structural properties. citeturn0search6

## Valueflows profile

Valueflows provides an important external check: Intent is planned/proposed, Commitment is promised, Claim represents future reciprocity, and EconomicEvent represents actual economic flow. Its correction relation explicitly leaves the initial event unchanged. citeturn0search2turn0search4

The Economic Fabric therefore cannot collapse:

```text
Intent -> EconomicEvent
Commitment -> Settlement
Claim -> Settlement receipt
Correction -> historical mutation
```

## Integral five-system trace

The current Integral Development Guide defines cross-system contracts including OAD→COS Certified Design Package, COS→ITC Labor and Materials Record, COS→FRS Operational Signal, ITC→FRS Credit and Access Signal, FRS→CDS Sensemaking Artifact, CDS→OAD Design Mandate, and related flows. citeturn0search24

The Economic Fabric should therefore preserve the originating contract rather than replace it with a generic transaction.

## Claim ceiling

This state machine is a semantic/conformance reference. It does not prove economic performance, physical occurrence, legal validity, accounting compliance, ITC correctness, or settlement finality.