# Settlement Rail Profile V1

## Purpose

A monetary/instrument identity does not determine the rail used to move, book, reconcile or settle it.

```
InstrumentProfile
  × SettlementRailProfile
  × EvidenceProfile
  × FinalityProfile
  × AuthorizationScope
```

## State algebra

```
Observed
 -> Validated
 -> Authorized
 -> Executed
 -> IncludedOrBooked
 -> Confirmed
 -> FinalityQualified
 -> Reconciled
 -> OutcomeObserved
```

The states are semantic states, not a universal promise that every rail exposes every state.

### Critical separations

- message != instruction
- instruction != acceptance
- acceptance != booking
- booking != execution
- execution != confirmation
- confirmation != finality
- finality != legal discharge
- transfer != market fill
- market fill != settlement
- provider acknowledgement != finality
- accounting projection != source economic event

## Rail profile

A profile binds:

- rail_id and revision;
- compatible instrument profiles;
- observation/evidence requirements;
- execution semantics;
- inclusion/booking semantics;
- confirmation semantics;
- finality model;
- reversal/reorg model;
- timeout and indeterminate behavior;
- reconciliation identity;
- correction lineage;
- claim ceiling.

## Finality profiles

### Probabilistic

A rail may express increasing confirmation depth without an absolute protocol finality state. This is materially different from consensus-finalized state.

### Consensus-finalized

A rail may expose an explicit finalized state under its consensus rules. Ethereum's proof-of-stake documentation distinguishes justified and finalized checkpoints and describes finalized blocks as not ordinarily reversible without a consensus failure. citeturn0search3turn0search6

### Provider-defined

An intermediary may publish a completion/status state. That state remains provider-specific unless the rail profile explicitly defines how it maps to stronger evidence.

### Mutual-credit / ledger-finality

A mutual-credit system may define finality through its own ledger/authority rules. It must not inherit blockchain finality semantics merely because both systems move quantities.

### Unknown / indeterminate

Timeout, conflicting observations, missing evidence, reorg, provider outage or unresolved reconciliation must remain explicitly indeterminate rather than being promoted to success.

## Web3 identity

Chain identity should be explicit. CAIP-2 defines chain identifiers, while CAIP-19 composes chain identity with asset namespace/reference for asset identity. citeturn0search2turn0search0

Therefore an ERC-20 rail observation should retain:

- chain_id;
- token contract/mint;
- transaction identity;
- block/inclusion evidence;
- confirmation/finality evidence;
- quantity/unit scale;
- source event identity.

A token transfer event is not automatically a market fill, and a market fill is not automatically a final settlement.

## Fiat/payment rails

For bank/payment adapters, preserve the source message/instruction identity and the provider/rail evidence. Do not collapse message delivery, acceptance, booking, settlement and legal discharge.

## Bridge rails

Bridge transfer is its own rail profile.

```
source asset
 -> bridge lock/burn/escrow evidence
 -> bridge message/proof
 -> destination mint/release
 -> destination observation
 -> bridge reconciliation
```

Destination representation does not become native-asset identity merely because a bridge completed.

## Settlement receipt algebra

A settlement receipt must contain:

```
receipt_id
source_event_identity
instrument_identity
rail_identity
execution_evidence
inclusion_or_booking_evidence
confirmation_state
finality_profile
authorization_reference
settlement_identity
reconciliation_identity
correction_lineage
claim_ceiling
```

A receipt proves only the states and evidence explicitly qualified by its rail profile.

## Reconciliation

Reconciliation is not merely "balance matches."

It should establish correspondence between:

```
source event
↔ rail execution
↔ external observation
↔ settlement identity
↔ target economic event
```

Mismatches become Conflict, Indeterminate or CorrectionRequired; they do not disappear through retries.

## Claim ceiling

This specification provides semantic settlement-rail interoperability only. It does not establish legal discharge, asset legitimacy, solvency, custody, investment value, regulatory compliance or universal finality.
