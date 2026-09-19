# mycelix-stewardship-policy-authority

STEW-012A is a fail-closed **structural policy-authority candidate** theorem composed from STEW-003 policy and STEW-008 stewardship admission.

```text
policy + admitted AccessPolicyParticipation claim
+ mandate/delegation evidence
+ currentness evidence
+ binding evidence
!= runtime authorization
```

The crate checks only structural prerequisites that can be proven from the supplied records:

- every admission is `AdmittedUnderProfile`;
- every admission is for `AccessPolicyParticipation`;
- every admission target structurally covers the exact policy target;
- duplicate admission records are rejected;
- mandate/delegation, currentness, and binding evidence references are present and bounded.

Exact-representation, revision-wide, and subject-wide stewardship claims have explicit structural coverage rules. Structural coverage does **not** establish authority scope, and no authority propagates automatically to translations, remixes, restorations, derivatives, or future revisions.

The asserted issuer, admission decider/profile, mandate/delegation, currentness, and evidence references remain unevaluated. This crate deliberately exposes no `is_authorized()` or runtime capability conversion.
