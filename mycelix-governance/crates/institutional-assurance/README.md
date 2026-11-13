# LISE Institutional Assurance Fixtures

This crate is the first executable slice of the Luminous Institutional Systems
Engineering (LISE) and Institutional Performance Benchmark (IPB) programme.

## Current scope

The crate now contains two deterministic benchmark slices.

IPB-S2 (conflict of interest) checks:

- six positive controls covering independent approval, exact amount limits, inclusive start and pre-expiry boundaries, substitute approval after proposer recusal, and resource-specific authority;
- beneficiary and proposer self-approval as negative controls;
- an independently seeded conflict of interest;
- fail-closed behavior when conflict screening is incomplete;
- missing, expired, future-dated and malformed authority windows;
- wrong grant holder, scope and resource;
- proposal substitution and zero-value requests;
- expenditure limits attached to an authority grant.

S2 emits machine-readable receipts with SHA-256 digests of the policy, input
and receipt material. It uses fixed fixture time and no RNG, network calls or
wall-clock time.

IPB-S3 (adversarial institutional capture) adds nine deterministic
known-answer scenarios. It distinguishes formal control quorums from a
coalition's effective ability to exercise agenda, appointments, authorization
and treasury custody, then checks independent audit/appeal capacity. Its
outcomes distinguish capture detected, no capture observed within the fixture,
and execution unavailable. Missing/malformed domains and invalid controller
sets return typed input errors.

## Run

From the repository root:

    cd mycelix-governance
    cargo fmt --all -- --check
    cargo test -p institutional-assurance
    cargo run -p institutional-assurance --bin ipb-run
    cargo clippy -p institutional-assurance --all-targets -- -D warnings

The governance workspace Cargo.lock includes the crate and pins the resolved
dependency graph. Use --locked in CI and preserve the exact-head run manifest.

The runner prints a JSON suite report. Its process exits non-zero when one or
more expected outcomes do not match.

## Evidence boundary

This is a synthetic assurance fixture. A passing S2 suite means only that the specified policy behavior matches its
twenty fixture expectations. A passing S3 suite means only that the bounded
capture predicate matches its eight synthetic known-answer cases on the tested
revision. It does not demonstrate that:

- the production Holochain zomes enforce the same policy;
- conflict screening discovers undisclosed or hidden interests;
- a legal or constitutional requirement has been interpreted correctly;
- the institution is legitimate, fair or safe in the field;
- the benchmark represents the full threat space.

Production integration requires an explicit policy mapping, tests against the
actual zome boundary, independent review, and real-world validation. The
fixture must not silently become an authorization service.

## Next extensions

1. Add a versioned authority graph and delegation-chain tests.
2. Add separation-of-duties, quorum and appeal fixtures.
3. Add consume-once authorization and unknown-outcome reconciliation tests.
4. Expand S3 with explicit delegation-chain, appointment-control and temporal
   coalition profiles, consuming Symtropy #3684 and Symthaea #3054/#1872 outputs.
5. Add cross-organization evidence and dispute-resolution scenarios.
6. Compare each implementation with matched manual and conventional workflows.

## Validation ledger

**2026-11-13:** The first exact-PR CI formatter job identified rustfmt-only differences in
the runner, fixture and test source. Those differences were applied from the
hosted formatter output. The initial formatter failure is retained in CI
history; it is not relabeled as a pass.

A later exact-head governance workspace run reached the new crate and
reported Rust E0505 borrow/move errors. The receipt construction was corrected
to compute the receipt digest before moving the policy/input digest strings.

**S2 exact-head validation:** Mycelix CI run #13845 on
`c9a366095ad204f2561ac2c76ff7de8a4dd5b73c` completed successfully:
change detection, formatting, governance workspace tests and aggregate
`ci-pass` all succeeded. Unrelated path-filtered suites were skipped, not
tested. The branch does not claim full-repository validation.

**S3 first exact-head CI:** run #13863 on the first integrated S3 head found
the module declaration before crate-level documentation (E0753) and rustfmt
differences in the new test file. Both were corrected. A review of the predicate
also found a false-positive path when both coalition and independent groups
could independently satisfy the oversight quorum; the predicate and a dual-
quorum control were corrected. The follow-up exact-head run #13864 was
superseded by this correction.

**S3 validation:** source and fixtures are authored on
`research/ipb-s3-institutional-capture-v0`; exact-head hosted formatting,
compilation and tests are pending. No S3 pass is claimed.
