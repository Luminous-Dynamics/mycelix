# Observation stream timebase v0.1

Implementation target for Mycelix #6054. The stream contract is separate from passport v0.1's individual sample checks.

- Scope every stream by instrument, channel, firmware/configuration digest, stream ID, clock domain and boot epoch.
- Keep event, acquisition and collector receipt times distinct.
- Use explicit timestamp basis, origin, unit and clock synchronization uncertainty.
- Only compare monotonic clocks inside one exact boot epoch/domain.
- Exact replay duplicates are idempotent; conflicting reuse is rejected.
- Out-of-order delivery remains explicit and only reconciles inside a declared bounded window.
- Sequence gaps remain gaps; no missing-as-zero or interpolation.
- Versioned clock mappings require validity window, offset/drift, uncertainty and a verification receipt.

All 40 corpus cases are synthetic. Results qualify only deterministic stream-ordering semantics, not real clock accuracy or physical measurement.
