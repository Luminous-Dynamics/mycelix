# Integral ITC ↔ Economic Fabric Crosswalk v1

**Scope:** MYC-INT / MYC-ECO integration  
**Status:** implementation contract candidate; not Integral-ratified

## Purpose

Connect Integral's ITC boundary to the Economic Fabric without turning ITC into SAP, TEND, MYCEL, or a universal currency.

Central rule:

~~~text
COS source observation
    ↓
ITC-owned assessment / policy
    ↓
ITC accounting entry
    ↓
Economic-Fabric projection
~~~

The Economic Fabric transports and reconciles economic meaning; it does not replace Integral's ITC semantics.

## Ownership boundaries

| Concept | Source owner | Economic-Fabric treatment |
|---|---|---|
| performed labor | COS/manufacturing | source observation |
| verified labor | COS/evidence | qualified observation |
| labor weighting | ITC | policy/assessment |
| ITC credit issuance | ITC | issuance/accounting event |
| ITC account balance | ITC | position projection |
| ITC access rule | ITC | policy reference |
| SAP balance | Finance/SAP | separate instrument position |
| TEND obligation | Finance/TEND | separate mutual-credit instrument |
| MYCEL reputation | identity/reputation | non-monetary observation |
| market/fiat valuation | valuation/bridge profile | valuation observation, not quantity identity |

Therefore:

~~~text
LaborEvent != ITC credit
ITC credit != SAP
ITC credit != TEND
ITC credit != MYCEL
ITC credit != fiat
~~~

## Integral five-system loop

~~~text
OAD
 │
 ├── design intelligence ──────┐
 │                             ↓
COS ── labor/material facts ─→ ITC
 │                             │
 └── operational signals ─→ FRS
                               │
ITC ── credit/access signal ─→ FRS
                               │
                               ↓
                              CDS
                               │
                    bounded decision/mandate
                               │
                    explicit domain authorization
                               ↓
                         OAD / COS / ITC
~~~

The arrows are data/semantic interfaces, not automatic authority transfer.

## ITC ledger mapping

A candidate ITCLedgerEntry projection should preserve at minimum:

- external schema identity and revision;
- entry identity;
- account identity;
- participant identity;
- source event references;
- credit/debit direction;
- quantity;
- unit and scale;
- weighting/assessment reference;
- policy/profile identity;
- validity interval;
- issuance/adjustment reason;
- correction/supersession lineage;
- provenance;
- privacy classification.

A balance is a projection:

~~~text
ledger entries
   ↓
ITC account projection
~~~

It is not a replacement for the append-only event lineage.

## Labor boundary

Integral's combined convenience representation must not cause internal semantic collapse.

Internally preserve:

~~~text
COSWorkObservation
    ↓
ITCWeightingAssessment
    ↓
ITCCreditIssuance
~~~

Each derived field in an external combined LaborEvent MUST carry a provenance reference identifying whether it originated from COS observation, ITC assessment, or ITC issuance.

Examples:

~~~text
hours_performed = COS fact
weight = ITC assessment
credits_issued = ITC issuance
~~~

Changing the ITC weighting policy MUST NOT rewrite the historical COS observation.

## Material boundary

Likewise:

~~~text
planned material
!= reserved material
!= consumed material
!= ITC-assessed contribution
~~~

A MaterialConsumptionEvent should preserve:

- material identity;
- quantity + unit;
- work/design generation;
- actual consumption timestamp;
- source evidence;
- ecological classification provenance;
- correction lineage.

Any ITC-derived value is a downstream assessment and must not become the source-of-truth material quantity.

## Economic-Fabric event categories

### Observation

A source fact about work, material, design, or account state.

### Assessment

An ITC-owned calculation or policy application.

### AccountingEvent

A durable ITC ledger mutation.

### Position

A current account projection derived from accounting history.

### Obligation / Entitlement

Only when the ITC source semantics explicitly establish one.

### Authorization

Never inferred from an ITC balance or credit amount alone.

### Settlement

Never inferred from an accounting entry alone.

### EconomicOutcome

Only after the relevant domain effect has been observed and reconciled.

## Anti-collapse controls

The adapter MUST reject or quarantine:

1. raw labor hours presented as already-issued ITC;
2. ITC balance presented as SAP;
3. ITC balance presented as TEND credit;
4. ITC balance presented as MYCEL reputation;
5. ITC credit limit presented as ownership;
6. ITC account state presented as authorization;
7. accounting acknowledgement presented as settlement;
8. stale ITC projection presented as current;
9. corrected source observation implemented by mutating the historical entry;
10. foreign ITC recognition presented as local issuance;
11. ITC valuation presented as universal price;
12. ITC access eligibility presented as legal ownership.

## Crosswalk to Economic-Fabric lifecycle

~~~text
COS observation
  → Economic Intent / Event reference

ITC policy
  → Assessment / policy reference

ITC issuance
  → AccountingEvent

ITC account projection
  → PositionObservation

Cross-system access claim
  → Entitlement candidate

Explicit authorization
  → Authorization

Physical/service effect
  → Execution + EffectReceipt

Observed result
  → OutcomeObservation

Correction
  → appended correction lineage
~~~

No arrow is implicit authority.

## Fiat and Web3 compatibility

ITC can coexist with external instruments without adopting their denomination.

~~~text
ITC 10 units
    +
USD valuation observation
    ≠
10 USD
~~~

and:

~~~text
ITC entitlement
    +
external Web3 settlement rail
    ≠
native Web3 asset
~~~

An adapter must retain both the ITC semantic identity and the external instrument identity.

## Qualification ladder

1. **SourceProfile** — external ITC schemas/statuses frozen;
2. **ReferenceModel** — deterministic mapping with provenance;
3. **Conformance** — positive/negative corpus passes;
4. **RuntimeAdapter** — real Finance/ITC objects translated;
5. **FederatedConformance** — cross-node recognition/retry tested;
6. **PilotObservation** — real deployment evidence, if pursued.

A later stage must not retroactively promote an earlier stage's evidence.

## Required executable corpus

Positive cases:

- verified labor → ITC assessment;
- assessment → credit issuance;
- ledger → account projection;
- material consumption → ITC-derived contribution;
- ITC access signal → FRS;
- corrected labor → correction lineage;
- cross-node ITC recognition preserving origin;
- ITC + external valuation;
- ITC + external settlement rail.

Negative cases:

- labor → money;
- ITC → SAP;
- ITC → TEND;
- ITC → MYCEL;
- account balance → authorization;
- accounting entry → settlement;
- foreign recognition → local issuance;
- correction → historical rewrite;
- valuation → unit substitution.

## Claim ceiling

This crosswalk establishes a candidate interoperability contract only.

It does not establish Integral ratification, correctness of Integral's economic policy, legal ownership, legal tender, value equivalence between ITC and other instruments, real-world economic effectiveness, safety/ecological validity, authorization to execute physical actions, or external settlement finality.
