# mycelix-stewardship-reciprocity-satisfaction

STEW-011A adds a profile-relative admission layer over STEW-011 receipts.

Core separation:

```text
receipt exists
!= performance verified
!= beneficiary acceptance
!= obligation satisfaction
!= legal discharge
!= runtime authorization
```

The closed v1 disposition vocabulary is:

```text
AdmittedSatisfiedUnderProfile
RejectedUnderProfile
DisputedUnderProfile
Indeterminate
```

The strongest positive state is deliberately named `AdmittedSatisfiedUnderProfile`, never bare `Satisfied`. `ReportedPartiallyFulfilled` is retained as evidence but cannot by itself be promoted into a satisfied admission.

Currentness is independent of disposition:

```text
AssertedCurrent
AssertedRevoked
AssertedSuperseded
AssertedExpired
Indeterminate
```

Only `AssertedCurrent` may accompany a positive admission.

Positive admission requires receipts for the same exact obligation, at least one `ReportedFulfilled` receipt, evaluation evidence, currentness evidence, and beneficiary acceptance when the evaluation profile says acceptance is required.

Conflicting receipts are first-class. Divergent receipt outcomes plus a positive satisfaction admission require explicit conflict-resolution evidence. Without that separate evidence plane, construction fails closed. Even with conflict-resolution evidence, `AdmittedSatisfiedUnderProfile(A)` does not imply universal satisfaction, legal discharge, or admission under profile B.

This crate implements no latest-wins, majority voting, reputation weighting, token/stake weighting, or universal beneficiary veto/waiver rule.

`AdmittedSatisfiedUnderProfile` remains process-relative evidence for later runtime evaluation. It is not a payment oracle, cultural authority grant, legal discharge, capability, enforcement decision, or runtime authorization.
