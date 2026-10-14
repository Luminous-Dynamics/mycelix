# AeroEvidenceV1 Test Vectors

These vectors are deliberately small. They test lineage semantics, not aircraft safety.

## Vector 1: measurement

Subject: fixture-A
Configuration: fixture-A/v1
Claim: measured hole diameter is 10.02 mm
Claim kind: measurement
Method: caliper-procedure/v1
Toolchain: caliper-17/calibration-2026-08
Uncertainty: ±0.02 mm
Validity domain: 20 ± 2 °C
Expected: a verifier can reproduce the claim's identity from canonical fields and trace it to the exact configuration.

## Vector 2: prediction versus observation

Prediction: finite-element run predicts 12.0 kN failure load.
Observation: physical test records 11.6 kN.
Expected: both records remain immutable; a PredictionError links them and records the 0.4 kN difference.

## Vector 3: design fork

Parent configuration: fixture-A/v1
Change: hole diameter changed from 10.0 mm nominal to 10.5 mm nominal.
Expected invalidation: only evidence whose validity domain depends on the changed geometry is invalidated; unrelated material traceability evidence remains valid.

## Vector 4: failed test

Test: fixture-A/v2 load test
Result: FAIL
Expected: failed result remains addressable after a later PASS result. The later result does not erase the failed evidence.

## Vector 5: restricted evidence

Claim: material certificate verified.
Basis: restricted supplier certificate reference.
Expected: public metadata can prove dependency on restricted evidence without exposing the certificate payload.

## Vector 6: supersession

Evidence E1 is active.
Evidence E2 supersedes E1.
Expected: E1 remains resolvable, lifecycle becomes superseded, and E2 points to E1 as its predecessor.

## Vector 7: dispute

Party B disputes claim E2.
Expected: a dispute relation is added without mutating E2's original payload.

## Vector 8: reproduction

Independent engineer reproduces analysis E3.
Expected: reproduction R3 references E3, records its environment and result, and preserves any discrepancy instead of rewriting E3.