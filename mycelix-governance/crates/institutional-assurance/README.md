# LISE Institutional Assurance Fixtures

This crate is the first executable slice of the Luminous Institutional Systems
Engineering (LISE) and Institutional Performance Benchmark (IPB) programme.

## Current scope

The initial suite implements IPB scenario S2 (conflict of interest) as a
deterministic policy fixture. It checks:

- independent, authorized approval as a positive control;
- beneficiary and proposer self-approval as negative controls;
- an independently seeded conflict of interest;
- fail-closed behavior when conflict screening is incomplete;
- missing, expired, future-dated and malformed authority windows;
- wrong grant holder, scope and resource;
- proposal substitution and zero-value requests;
- expenditure limits attached to an authority grant.

Every case emits a machine-readable receipt with SHA-256 digests of the policy,
input and receipt material. The suite uses fixed fixture time and no RNG,
network calls or wall-clock time.

## Run

From the mycelix-governance directory:

    cargo test -p institutional-assurance
    cargo run -p institutional-assurance --bin ipb-run

The runner prints a JSON suite report. Its process exits non-zero when one or
more expected outcomes do not match.

## Evidence boundary

This is a synthetic assurance fixture. A passing suite means only that this
crate's specified policy behavior matches its fifteen fixture expectations on the
tested revision. It does not demonstrate that:

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
4. Add capture and collusion scenarios with controlled attacker budgets.
5. Add cross-organization evidence and dispute-resolution scenarios.
6. Compare each implementation with matched manual and conventional workflows.
