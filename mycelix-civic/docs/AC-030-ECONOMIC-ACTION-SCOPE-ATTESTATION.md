# AC-030 — Economic Action Scope Attestation

## Purpose

AC-025 closed the composition gap between substrate integrity and impact reciprocity.

AC-030 closes the next bypass surface: a caller can otherwise choose the dimensions passed into the gate at the call site.

The new `EconomicActionScope` makes that choice an explicit policy artifact.

## Scope declaration

An action scope contains:

- stable scope identifier;
- action reference;
- action purpose;
- required substrate dimensions;
- governing policy reference;
- responsible authority reference;
- attestation/proof reference;
- supporting evidence references;
- declaration timestamp.

The reference model does not prescribe a cryptographic signature format. `attestation_ref` can point to whatever externally governed signing or authorization mechanism a deployment uses.

## Canonical declaration

Required dimensions must be strictly sorted and unique.

This is deliberate. The SDK does not silently sort or deduplicate an authority declaration because doing so could conceal contradictory input from an upstream policy engine.

## Known-impact coverage

AC-030 also adds an action-scope check to the impact ledger.

Every impact already recorded for the action must have its substrate dimension included in the declared scope.

For example, when an action already has an ecological depletion record, a scope containing only financial integrity cannot qualify that action.

This is a bounded guarantee: it detects omission against known evidence. It does not claim that an impact ledger can prove the absence of impacts that have never been observed.

## Scoped assessment

`EconomicIntegrityGate::assess_scoped`:

1. validates the scope declaration;
2. verifies known impact coverage;
3. evaluates the AC-017 substrate gate;
4. evaluates the AC-018 impact gate;
5. returns the AC-025 combined decision;
6. preserves the scope ID, action reference, and covered impact IDs.

The pre-existing `assess` API remains available for compatibility. The scoped path is the stronger execution boundary for integrations that require explicit policy provenance.

## Invariants

### No silent scope narrowing

Known affected dimensions cannot disappear from the declaration.

### No implicit canonicalization

Duplicate or unordered declarations are rejected rather than rewritten.

### Policy provenance remains visible

The scope object preserves the declared scope identity, policy, authority, attestation, and evidence references.

### No claim of omniscient impact discovery

Unknown impacts remain an evidence/discovery problem. AC-030 does not pretend the ledger can prove that no unrecorded impact exists.

### Existing composition remains fail closed

Scope validation happens before AC-025 assessment, and AC-025 still prevents either integrity layer from compensating for the other.

## Research basis

The Open Contracting Data Standard treats publication policy as important context for understanding what information is covered, where it comes from, how it is generated, and how it changes over time. OCDS releases are immutable, with later changes represented by new releases so that the history remains inspectable.

References:

- https://standard.open-contracting.org/latest/en/guidance/publish/
- https://standard.open-contracting.org/latest/en/primer/releases_and_records/
- https://standard.open-contracting.org/latest/en/schema/reference/

AC-030 borrows only the engineering principle: make coverage explicit, bind decisions to policy/provenance, and preserve the history of what was asserted. It does not attempt to turn OCDS into an economic-control standard.

## Validation added

- valid scoped assessment preserves action and known impact coverage;
- known impacted dimension omitted from scope is rejected;
- missing attestation reference is rejected;
- unordered dimensions are rejected;
- duplicate dimensions are rejected.

## Security contribution

AC-030 moves the boundary from:

`caller chooses inputs -> gate evaluates`

to:

`policy-scoped action -> validated coverage -> integrity gate -> auditable assessment`

That matters because an integrity check can only constrain what it was actually asked to inspect.
