# IG-007T1 — Treasury/Credit downstream counterexamples

## Parent

IG-007T1 is a deterministic child of IG-007T0 / #1091 / draft #1105.

It does not change product Rust and does not execute a live transfer or DHT mutation.

## Bound profile

```text
id        mycelix-treasury-credit-downstream-observed-fca2c107-v1
SHA-256   77db81e40ebe7b0ec9278f8c9aad7584faa08b6784cde4779393d1d6b88cf58d
authority ObservedSourceBound
```

## Corpus

```text
schema    mycelix-treasury-credit-downstream-counterexamples-v1
authority MeasurementOnly
SHA-256   89b795cea150c1d0f18aa63bc6adfe2ffd31389ef9396bbde810f647540695de
issues    #1085, #1086
```

## Fixtures

### CE-TC-01 — legacy dispatch containment

The legacy governance action targets `governance_bridge::transfer_credits`, while T0 observes zero such exported entrypoints in the exact eight-module bridge coordinator census.

The execution caller fails the action on call error.

Result:

`LegacyTransferCreditsDispatchFailClosedByMissingTarget`.

This is positive containment, not an unauthorized-transfer claim.

### CE-TC-02 — bridge Finance dispatch containment

`execute_approved_transfer` targets `finance/treasury::execute_governance_transfer`, while T0 observes zero such exported entrypoints in the exact Treasury coordinator.

The exact bound `governance_utils::call_role` helper returns `Err` on transport errors, network errors and unexpected responses.

Result:

`ApprovedTransferBridgeDispatchFailClosedByMissingTreasuryEntrypoint`.

Again, this is positive containment.

### CE-TC-03 — coordinator approval policy is not an integrity theorem

The ordinary coordinator path requires a Treasury manager and uses manager-majority approval.

The frozen Allocation-update integrity validator reconstructs neither that manager theorem nor a status-transition graph.

Result:

`CoordinatorApprovalPolicyNotReconstructedByIntegrity`.

### CE-TC-04 — Treasury shape validity is not mutation authority

A pure fixture varies balance and manager fields while retaining a finite unit-interval reserve ratio, the only frozen update checks represented by T0.

Result:

`TreasuryShapeValidityDoesNotEstablishMutationAuthority`.

No Treasury publication is performed.

### CE-TC-05 — Allocation shape validity is not authorized transition

A pure fixture retains positive amount while varying status, recipient and `approved_by`. T0 records no observed transition graph or immutable-field theorem in `validate_update_allocation`.

Result:

`AllocationShapeValidityDoesNotEstablishAuthorizedTransition`.

No Allocation publication is performed.

### CE-TC-06 — proposal linkage is optional and unreconstructed

The observed `propose_allocation` surface accepts `proposal_id: Option<String>` and T0 observes no proposal-authority reconstruction on that path.

Result:

`ObservedAllocationProposalLinkageIsOptionalAndUnreconstructed`.

This does **not** assert that every Treasury allocation must originate in a governance proposal.

## Positive controls

T1 retains separately:

- legacy target absence is fail-closed;
- bridge Finance target absence is fail-closed through the bound helper;
- manager-majority coordinator approval exists;
- Treasury debit uses checked subtraction.

## Successor semantics

A correct #1085 successor should establish one content-authorized governance→Treasury route rather than merely adding function aliases.

A correct #1086 successor should make the relevant Treasury/Allocation authority and transition predicates integrity-enforced, not coordinator-only.

The historical T0/T1 evidence must remain unchanged after repairs; successor evidence should intentionally stop reproducing the applicable authority-gap fixtures while preserving appropriate positive controls.

## Non-claims

No live unauthorized transfer, forged Treasury/Allocation publication, stolen funds, deployment exploit, deployment currentness, or governance-safety claim is made.
