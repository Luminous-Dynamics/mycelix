# Economic Fabric Conformance Profile v1

**Scope:** MYC-ECO-005 / #3346  
**Purpose:** define the executable state machine that Finance adapters must satisfy before native Economic-Fabric integration.

## 1. Typed lifecycle

Every translated economic action is represented as a lifecycle, not a single boolean success:

```text
Intent
  -> Authorization
  -> Execution
  -> RailEvidence
  -> Finality
  -> Reconciliation
  -> OutcomeProjection
```

Each stage is independently qualified.

### Intent

Identifies:

- action kind;
- instrument profile;
- quantity + unit scale;
- source subject;
- destination subject;
- correlation/idempotency key;
- timestamp;
- originating system.

Intent does not authorize execution.

### Authorization

Binds an intent to an explicit authority:

- authorizing subject;
- policy/profile;
- scope;
- expiry;
- nonce/idempotency constraint;
- evidence/reference.

Authorization does not prove execution.

### Execution

Records the attempt:

- execution identifier;
- intent identifier;
- executor;
- requested quantity;
- accepted/rejected state;
- failure code where applicable.

Execution does not prove settlement.

### Rail Evidence

Records what the selected rail actually reported:

- rail identifier;
- provider/network;
- external reference;
- observed quantity;
- observed asset/instrument;
- provider state;
- observation timestamp.

Provider acknowledgement is not universal finality.

### Finality

Finality is a typed claim:

- finality profile;
- evidence references;
- confidence/qualification state;
- observed finality timestamp;
- optional expiry/reorg/challenge window.

No adapter may emit `final=true` merely because execution returned successfully.

### Reconciliation

Compares intended and observed effects:

- quantity equality;
- unit equality;
- instrument/profile equality;
- source/destination equality;
- fee treatment;
- duplicate detection;
- correction lineage.

A mismatch produces a reconciliation exception rather than silently normalizing the result.

### Outcome Projection

Only after reconciliation may the adapter project an economic outcome.

An outcome is a projection of evidence, not a replacement for it.

## 2. Instrument identity

Every quantity crossing the Fabric boundary carries an instrument profile reference.

Minimum identity:

```text
namespace
instrument_id
profile_version
unit
scale
issuer_or_steward
origin
```

For bridged instruments, origin MUST remain explicit.

For SAP:

```text
namespace = mycelix
instrument_id = SAP
profile_version = explicit
unit = micro-SAP
scale = 10^-6 SAP
```

For TEND, the profile MUST additionally preserve mutual-credit semantics and obligation direction.

For MYCEL reputation/trust observations, the profile MUST identify the observation domain and MUST NOT expose the observation as a transferable monetary instrument.

## 3. Semantic firewall

Adapters MUST reject implicit promotions:

| From | To | Default |
|---|---|---|
| reputation observation | monetary issuance | reject |
| SAP position | TEND position | reject |
| credit limit | asset ownership | reject |
| entitlement | authorization | reject |
| authorization | settlement | reject |
| settlement acknowledgement | finality | reject |
| accounting projection | source event | reject |
| valuation rate | quantity conversion | reject |
| simulation instrument | physical instrument | reject |
| bridged asset | native asset | reject |

A promotion can only occur through an explicit, versioned profile/policy that names the transformation and its evidence requirements.

## 4. Conservation invariants

For a conservation-preserving transfer:

```text
source_before - debited - fees = source_after
destination_before + credited = destination_after
```

subject to the instrument's explicit lifecycle rules.

For SAP specifically, demurrage is a distinct lifecycle adjustment. It MUST NOT be represented as an unexplained transfer between participants.

Issuance, retirement, compost redistribution, bridge conversion, and redemption MUST each have explicit source/sink semantics.

## 5. Idempotency

Every mutating intent MUST carry an idempotency key.

Repeated delivery of the same intent must produce:

- the same lifecycle identity;
- no second economic effect;
- an auditable duplicate observation.

A different intent with the same payload MUST not be assumed to be a duplicate.

## 6. Corrections

Corrections append to lineage:

```text
original event
    |
    +--> correction
    |
    +--> reversal
    |
    +--> replacement projection
```

Historical source events are not rewritten to make reconciliation pass.

## 7. Failure taxonomy

The executable harness should distinguish at minimum:

```text
PROFILE_MISMATCH
UNIT_MISMATCH
SCALE_MISMATCH
ISSUER_MISMATCH
ORIGIN_MISMATCH
AUTHORIZATION_MISSING
AUTHORIZATION_EXPIRED
EXECUTION_REJECTED
RAIL_MISMATCH
FINALITY_UNQUALIFIED
RECONCILIATION_MISMATCH
DUPLICATE_INTENT
CORRECTION_LINEAGE_BROKEN
SEMANTIC_PROMOTION_FORBIDDEN
```

The distinction matters because a rejected authorization is not equivalent to a failed settlement, and a reconciliation mismatch is not equivalent to a transport failure.

## 8. Compatibility facade rule

Legacy APIs such as a one-shot payment function may remain for compatibility.

The facade MUST internally construct the lifecycle above and return a result that identifies the strongest state actually established.

Example:

```text
send_payment()
  -> Intent created
  -> Authorization verified
  -> Execution accepted
  -> Rail observed
  -> Finality pending
  -> Reconciliation pending
```

The facade MUST NOT collapse this to "settled" unless the rail's finality profile and reconciliation evidence establish settlement.

## 9. Test vectors

The existing negative corpus in `finance-fabric-negative-corpus-v1.json` becomes the minimum semantic firewall suite.

The executable harness should add positive vectors for:

1. SAP position observation;
2. authorized SAP issuance;
3. conservation-preserving SAP transfer;
4. SAP demurrage;
5. TEND mutual-credit obligation;
6. explicit external-fiat bridge observation;
7. Web3 bridged instrument with origin preservation;
8. authorization followed by pending settlement;
9. confirmed settlement with qualified finality;
10. correction/reversal with preserved lineage.

## 10. Cross-system interoperability

The Fabric boundary is intentionally system-neutral:

```text
SAP / TEND / MYCEL
      |
Mycelix Finance
      |
Economic Fabric
      |
Valueflows / accounting / ERP / Web3 / fiat / external rails
```

External systems may map into the Fabric, but no external representation automatically becomes a Mycelix-native semantic fact.

This allows future SAP/ERP, Valueflows, Integral ITC, Web3, fiat, and conventional accounting adapters to share one conformance contract without forcing them into one economic ontology.

## 11. Acceptance criterion

An adapter is conformant only when:

- all required identity fields survive round-trip translation;
- all quantities preserve unit and scale;
- lifecycle stages remain independently observable;
- negative semantic-promotion vectors fail closed;
- idempotency prevents duplicate effects;
- corrections preserve lineage;
- finality claims are backed by the declared rail profile;
- reconciliation mismatches cannot silently become success;
- the adapter can expose a machine-readable receipt.

This profile is deliberately stricter than a conventional "payment succeeded" API because interoperability is useful only when semantic meaning survives translation.
