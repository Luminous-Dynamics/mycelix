# GCPN civil-evidence fault adapter v0.1

This crate executes mutations against `gcpn-civil-evidence-contracts::audit_civil_evidence()`. It exists to finish the five fault families that the six-stage compositional-assurance adapter could not represent.

## Executed fault families

- Authority registry unknown / mismatch.
- Provider acceptance without device-delivery receipt.
- Missing accessibility evidence.
- Missing action execution receipts.
- Duplicate case-frame key.
- Wrong delivery-to-accessibility binding.
- Wrong accessibility-to-comprehension binding.
- CAP-lifecycle claim not supporting.
- Decision claim not supporting.
- Post-cutoff device-delivery receipt.

The tests exercise each individual fault and cross-stage interactions. The clean baseline also verifies that authority, delivery, accessibility, comprehension, action and case-frame claims remain separately visible.

## Run

    cargo fmt --all -- --check
    cargo test --locked --offline
    cargo clippy --locked --offline --all-targets -- -D warnings
    cargo run --locked --offline

Rust 1.98.1 is pinned. The only dependencies are local GCPN contract crates.

## Relationship to earlier adapters

- PR #5682 exercises 12 of the original 17 cross-stage fault atoms directly against the compositional-assurance engine.
- PR #5701 adds five missing civil dimensions to a separate typed contract layer.
- This adapter executes the missing five against the new auditor and adds cross-stage linkage controls.

Together these create a coverage claim across the original 17 fault families, but not a claim that a single integrated production engine or live MHEWS platform has been validated. The exact package revisions, adapters and fixtures remain separately versioned and must be composed under an exact manifest before an integrated qualification claim is made.

## Boundary

Synthetic inputs only. A signature check is not a legal authority determination; provider acceptance is not device delivery; accessibility is not comprehension; comprehension is not action; and channel counts do not reveal unique people across channels. No personal recipient identifiers, live alert channels or operational endpoints are used.
