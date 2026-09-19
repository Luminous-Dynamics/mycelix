# mycelix-stewardship-policy

STEW-003 is the fail-closed knowledge-use policy theorem for Mycelix stewardship.

The initial profile binds every policy to one exact `StewardedSubjectIdentityV1`. This is intentionally narrower than work-wide or lineage-wide licensing: permission for one exact representation does not silently propagate to translations, remixes, migrations, performances, restorations, later revisions, or other derivatives.

## Core separations

```text
permit(View) != permit(TrainAi)
permit(Research) != permit(Commercialize)
permit(Retrieve) != permit(Disclose)
permit(GenerateDerivative) != permit(Redistribute)
```

Unspecified actions fail closed.

## Permission candidates, not authorization

A permission rule may carry opaque references to constraints and duties. This crate does not evaluate those referenced objects, so its result is a `PermissionCandidate`, not final authorization.

```text
permission rule present
+ referenced constraints/duties
!= constraints satisfied
!= duties satisfied
!= runtime authorization
```

An explicit prohibition dominates a permission for the same action.

## No implicit inheritance

The v1 policy target is exact identity only. Subject-wide, revision-wide, community-wide, or derivative inheritance is deliberately out of scope until a later theorem can define explicit propagation and conflict rules.

## ODRL direction

ODRL remains the intended interoperability surface for richer permissions, prohibitions, constraints, and duties. STEW-003 is a small internal theorem, not a replacement for ODRL and not a claim of ODRL conformance.

## Non-claims

This crate establishes no identity of the policy issuer, stewardship legitimacy, copyright ownership, community consent, legal enforceability, constraint satisfaction, duty satisfaction, payment, cultural authority, confidentiality, or AI-training legality.
