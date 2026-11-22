# Energy Assurance Core

A dependency-free Rust policy kernel for the Mycelix P2P energy grid.

## Scope

- Fixed-point Wh meter interval validation with time, calibration, clock and device-integrity checks.
- Rated-capacity/time plausibility bound with explicit tolerance.
- Separate producer claims and purpose-scoped independent verifier attestations.
- Offer-fill conservation, expected-version checking and exact-payload idempotency.
- Settlement states that distinguish rail-final payment, rejected payment, unknown outcome and separately attested manual process.

## Run

From `mycelix-energy/`:

`cargo test --locked -p energy-assurance-core`

## Limits

This crate does not verify signatures, accreditation, calibration certificates, legal title, Holochain authorship, grid-code compliance or external banking finality. It does not replace integrity callbacks or a distributed atomic reservation protocol. The caller must authenticate evidence and execute the invariants at the authoritative mutation boundary. The crate is a pure reference kernel and its passing tests are not proof that the existing grid coordinator/integrity zomes are fixed.
