# Mycelix Authenticated Hospitality Qualification

`mycelix-business-authenticated-hospitality-qualification` is an additive evidence envelope above the controlled hospitality qualification and full control-document authenticity coverage.

It does not rewrite either prior envelope.

## Composition

Before emitting stronger evidence, the crate:

1. revalidates the complete controlled hospitality envelope against the original registration, target plan, transaction projection, immutable transaction report, control plan and control coverage;
2. re-runs the complete control-authenticity theorem against the original control entries, external authenticity receipts and current verifier/credential/revocation contexts; and
3. verifies the resulting authenticity transition is exactly the expected scope/interval transition.

A digest-valid summary alone is not sufficient.

## Limitation evolution

The controlled envelope already narrows export completeness and aggregation semantics while retaining:

`limitation:control-source-authenticity-unverified:v1`

Complete live authenticity coverage replaces only that limitation with:

`limitation:control-source-issuer-authority-unverified:v1`

All other limitations remain unchanged, including external-reality, metric-semantic-authority and time-rule provenance limitations.

This distinction is intentional: authenticated authorship does not establish that the author was institutionally authorized to issue the control statement.

## Safety

The envelope retains the original report digest and the prior controlled-envelope digest. It cannot change the model verdict, grant execution authority, verify cryptography, or mutate any provider/business system.
