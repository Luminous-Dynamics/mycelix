# mycelix-content-planner

Deterministic, authority-free placement proposal generation for Mycelix Content Fabric.

The crate accepts only a CF-06A `PolicyQualifiedPoolV1` plus the exact validated `StorageIntentV1` that produced that pool. Soft telemetry may influence ranking, but it cannot make an ineligible provider eligible and cannot bypass CF-06A subset validation.

## v1 properties

- integer/fixed-point scoring only;
- explicit normalization profile with stable ID;
- stale/future/malformed/conflicting telemetry fails to the conservative maximum penalty;
- missing weighted metrics receive the maximum penalty;
- deterministic tie-breaking by availability action;
- policy-preserving elimination rather than naive top-N selection;
- final subset is validated again through CF-06A;
- stable input/proposal commitments for replay and audit;
- recommendation-only authority.

The crate has no Holochain, Iroh, filesystem, marketplace, payment, lease, executor, or Symthaea runtime dependency.
