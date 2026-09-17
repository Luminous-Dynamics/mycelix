# mycelix-evidence-assurance

Pure `#![no_std]` AMSAP-002B1 companion crate for evidence assurance above qualified axis-bound claims.

This crate keeps four scientific properties independent:

```text
evidence magnitude
!= counterevidence magnitude
!= scientific contestation
!= diagnosticity
```

It does not reinterpret the underlying `AxisEvidenceClaim`. Instead it binds an assurance profile to the exact claim ID and claim-payload commitment.

`Diagnosticity::Informative` must be earned. A claim that says it is informative requires a matching diagnosticity receipt with a qualified sensitivity assessment and expected positive/sham-control behavior. A failed or unassessed control produces an inconclusive diagnosticity result rather than an informative null/challenge.

`CounterevidenceStrength::NoneDetected` likewise requires an adequate, claim-bound counterevidence search receipt. An unsearched claim cannot become `NoneDetected` by convention.

Strong evidence may coexist with high contestation; strong counterevidence may coexist with low contestation. None of these assurance properties grant consciousness, valence, responsibility, welfare protection, legal responsibility, currentness, deployment authority, governance authority, or external-effect authority.

Final append-only lifecycle transitions remain AMSAP-005E scope; this crate does not mutate scientific claims or erase historical observations.
