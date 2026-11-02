# GCPN subgroup reach and equity assurance v0.1

This Rust crate evaluates population-level warning reach by declared subgroup using typed delivery evidence from `gcpn-civil-evidence-contracts`.

## Paradigm shift

A high overall delivery percentage is not proof of equitable reach. GCPN reports one outcome per predeclared subgroup and evaluates the weakest group separately.

The estimand is:

- Single channel: device-delivered count for the declared cohort/channel.
- Multiple channels: the unique union count from an externally governed, privacy-preserving deduplication attestation.

It never adds channel counts and labels the sum "people reached". The attestation must bind the exact channel set, source/release, method/version, as-of time, channel total, unique union and overlap removed.

## Conservative coverage bounds

For unique reached count (R) and eligible population denominator interval ([N_L,N_U]):

- Lower coverage bound: (R/N_U).
- Upper coverage bound: (min(1,R/N_L)).

Unknown or suppressed denominators produce Unavailable and no denominator or reach count is emitted. A missing or unverified multi-channel deduplication attestation cannot support a unique-reach claim.

## Provisional research targets

The CLI demonstrates configurable policy values of:

- Minimum subgroup lower-bound coverage: 95%.
- Maximum upper bound on the cross-group coverage gap: 5 percentage points.

These are research targets, not universal legal standards or claims of operational feasibility. They must be set with local authorities and affected communities. The implementation reports raw bounds alongside each disposition so policymakers can change targets without rewriting the evidence.

## Report semantics

Each subgroup is reported separately as Supports, Refutes, Indeterminate or Unavailable. The equity disposition is non-compensatory:
- A group that fails the minimum coverage target cannot be hidden by a stronger group's result.
- Unknown or suppressed groups prevent overall equity Support.
- An insufficient number of qualified groups cannot establish a coverage gap.
- No overall composite warning-success score is produced.

The report contains coarse group/category labels only. It does not contain recipient-level IDs, device tokens or phone numbers.

## Run

    cargo fmt --all -- --check
    cargo test --locked --offline
    cargo clippy --locked --offline --all-targets -- -D warnings
    cargo run --locked --offline

Rust 1.98.1 is pinned. Dependencies are local Rust crates only.

## Qualification boundary

All fixtures are synthetic. This is not a telecom subscriber census, recipient identity system, legal authority determination, comprehension measure, proof of action, or MHEWS effectiveness result. Real deployment needs locally governed denominator sources, group taxonomies, privacy impact assessment, suppression rules, deduplication authority and community review. It must not create persistent individual recipient identifiers in GCPN.
