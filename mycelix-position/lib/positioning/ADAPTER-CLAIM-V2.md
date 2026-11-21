# Position adapter claim contract v2 (draft)

This additive module is the first implementation slice for Mycelix Position #6404. It introduces versioned, Rust-native observation/estimate/prediction claim types without changing v1 serialized fields.

## Ownership composition

- Reference-system/body/frame identity remains with #416.
- Time scale, clock domain and correlation semantics remain with #435.
- Evidence-class meaning remains with #419.
- Source provenance remains with #5899.
- Failure-domain/common-mode relationships remain with #5904.
- Fusion and covariance intersection remain with #417.
- Geometry/solver diagnostics remain with #433/#447.
- Operational health and service state remain with #458.

The new module carries opaque references to those owners. It is not a second CRS catalogue, clock service, trust authority, evidence ledger, fusion engine or health state machine.

## Core invariants

- A time record can preserve an absent raw reading explicitly (raw_value=None, unit=None) or preserve a supplied numeric reading and unit. A supplied number without a qualified time-basis remains unqualified; no UTC or common clock is invented.
- Time intervals can be compared only when both endpoints have numeric readings, the same qualified basis, the same declared unit and the same optional normalization commitment; #435 owns actual conversion/correlation. Mixed normalization commitments are rejected. Missing readings cannot be replaced with zero or receipt time.
- Frame identity is an opaque qualified reference owned by #416; bare coordinate numbers cannot create a frame.
- Observations, estimates and predictions are distinct Rust types and remain discriminated in serialized claims.
- Estimates bind parent evidence commitments and estimator profile identity.
- Predictions bind a parent estimate, propagation model and qualified interval.
- Source evidence carries provenance/calibration/configuration/failure-domain references but no trust score, admitted flag, LOS assertion or inferred independence.
- Uncertainty is explicitly represented and validated; this small v2 slice supports diagonal variance or a committed external representation, not general covariance math.

## Tests and exact-head validation

The v2 contract contains fourteen unit tests covering legacy/unqualified source-time preservation, source-vs-ingestion time separation, missing source timestamps, qualified interval basis/unit/order/normalization, JSON round-trip, estimate lineage, prediction lineage/time, finite uncertainty, duplicate failure-domain references, claim discrimination and empty references.

On exact API head `98ec2d2dccaf90d6fe52731818954dc98e4ae20e`, the dedicated Position workflow completed successfully with **82 tests passed, 0 failed in each mirrored positioning workspace**. This validates the package-level contract on that commit; it does not validate provider devices, time-source truth, calibration, navigation integrity, physical accuracy or safety.

## Downstream integration

Symthaea SOMA #10824 now has a reviewable consumer candidate in [PR #13052](https://github.com/Luminous-Dynamics/symthaea/pull/13052). Its isolated compatibility workflow pins this exact Mycelix API commit and has passed six adapter tests against the real Position package source. The harness is deliberately narrower than a full Symthaea workspace build and does not validate Android/provider driver paths or physical navigation.

This module is not an operational navigation qualification or an actuator authority.
