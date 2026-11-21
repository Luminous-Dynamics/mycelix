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
- Time intervals can be compared only when both endpoints have numeric readings, the same qualified basis and the same declared unit; #435 owns actual conversion/correlation. Missing readings cannot be replaced with zero or receipt time.
- Frame identity is an opaque qualified reference owned by #416; bare coordinate numbers cannot create a frame.
- Observations, estimates and predictions are distinct Rust types and remain discriminated in serialized claims.
- Estimates bind parent evidence commitments and estimator profile identity.
- Predictions bind a parent estimate, propagation model and qualified interval.
- Source evidence carries provenance/calibration/configuration/failure-domain references but no trust score, admitted flag, LOS assertion or inferred independence.
- Uncertainty is explicitly represented and validated; this small v2 slice supports diagonal variance or a committed external representation, not general covariance math.

## Tests

Nine unit tests cover legacy preservation, time-basis mismatch, interval ordering/unit mismatch, estimate lineage, prediction lineage/time, uncertainty validity, duplicate failure domains, claim discrimination and empty references. These tests are authored but have not yet been compiled or run in this environment.

## Downstream integration

Symthaea SOMA #10824 should consume these types only after the exact Mycelix API is reviewed and a real dependency feature-build is added. This module is not yet wired to mobile provider adapters and does not authorize physical navigation use.
