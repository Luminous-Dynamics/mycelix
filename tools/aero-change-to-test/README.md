# AeroCommons change-to-test adapter

A small Rust adapter that consumes canonical AeroEvidence impact dispositions and separates mandatory requalification from optional decision-value testing.

## Run

From the repository root:

    cargo test --manifest-path tools/aero-change-to-test/Cargo.toml

## Ordering and authority

The adapter does not implement its own dependency classifier. It consumes the impact class and dependency-coverage state supplied by the canonical impact owner. It recomputes the ChangeSet SHA-256 over a versioned canonical JSON payload with all reference arrays sorted, so array permutation does not alter change identity.

- INVALIDATED → mandatory, non-demotable requalification obligation.
- CONDITIONAL_REVIEW / REQUIRES_REVIEW → review obligation.
- UNKNOWN or incomplete dependency coverage → explicit unknown obligation.
- UNAFFECTED is accepted only with complete dependency coverage.

Each obligation has an explicit OPEN/CLOSED state. A CLOSED obligation requires a typed requalification_closure_receipt. An UNKNOWN obligation cannot be closed by this generic receipt; it needs a fresh canonical impact assessment.

**Optional tests are blocked until every mandatory and conditional obligation has a closure receipt and there are no unknown obligations.** After closure, candidates still pass separate authority, method, envelope, independent-reference and rights gates. Uncalibrated decision-value estimates are visible but not numerically ranked. Calibrated eligible candidates are ordered by conservative lower bound divided by declared total cost.

This receipt reference is not signature or authority verification. A deployment must authenticate and qualify the closure issuer independently; this crate only checks the typed link and state. The ChangeSet digest is recomputed from the canonical ChangeSet body, but does not authenticate its producer.

## Claim ceiling

The report is a research planning artifact. It does not approve manufacturing, inspections, repairs, maintenance release, part acceptance, certification, airworthiness or flight.
