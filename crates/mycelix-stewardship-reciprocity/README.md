# mycelix-stewardship-reciprocity

STEW-010 records authority-neutral reciprocity obligations over an exact stewarded subject representation.

Core separation:

```text
obligation recorded
!= asserting party authoritative
!= beneficiary identity verified
!= trigger occurred
!= benefit delivered
!= obligation satisfied
!= runtime authorization
```

## Compile-time semantic-role separation

The v1 API uses distinct reference types for roles that were previously easy to swap accidentally:

```text
ReciprocityAssertorRefV1
ReciprocityBeneficiaryRefV1
ReciprocityTriggerRefV1
ReciprocityRequirementRefV1
ReciprocityBasisRefV1
```

This means an assertor reference cannot be supplied as a beneficiary or trigger reference merely because all are ultimately opaque protocol identifiers.

```text
typed role separation
!= identity proof
!= authority proof
!= trigger truth
!= obligation legitimacy
```

Benefit clauses retain their exact `ReciprocityBenefitKindV1` plus typed requirement reference. Obligation basis references are non-empty, bounded, and duplicate-rejecting but remain asserted basis material rather than verified mandate/authority.

The theorem contains no satisfaction state, no latest-wins reducer, no permission API, no capability issuance, and no enforcement semantics.
