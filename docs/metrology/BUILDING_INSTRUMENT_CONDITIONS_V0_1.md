# Building instrument condition applicability v0.1

This additive slice binds actual sample-time environmental/influence conditions to a versioned capability profile. It does not change the v0.1 passport schema and does not authorize a real sensor or measurement.

## Synthetic profile

The only condition profile that can reach review eligibility is a synthetic air-temperature instrument profile with two measured local conditions: ambient temperature and relative humidity. Its limits are fixture values only, not engineering tolerances or comfort/safety limits.

Three physical profile families are declared for future work (thermal, optical PM and fluid flow), but they remain blocked until their exact instrument, method, calibration scope and condition variables are reviewed.

## Applicability rules

- Profile ID/revision/digest and instrument/configuration/firmware must match exactly.
- Every required condition is a source observation with digest, current evidence, calibration, uncertainty, time window, clock domain/epoch, location and rights.
- Condition observations must cover the entire target sample interval and use the same clock/time unit; ambient conditions from another room/plant zone do not silently apply.
- Provider assertions, assumed values and `not_applicable` cannot satisfy a mandatory measured condition.
- Out-of-envelope values, stale/expired observations, rights revocation, correction mismatch or mid-window changes remain explicit blocked states.
- A correction requires an exact method/revision/artifact/parameter digest and one application only.
- The result is at most `ELIGIBLE_FOR_INDEPENDENT_CONDITION_REVIEW_ONLY`, never instrument admission.

## Claim ceiling

All values and profiles are synthetic. This does not prove an actual instrument operated within its capability envelope, validate an environmental correction, qualify sensor performance, or establish any building/comfort/IAQ/water/energy claim.
