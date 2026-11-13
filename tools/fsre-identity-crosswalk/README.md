# FSRE temporal identity crosswalk reference

A dependency-free Rust validator for source-backed identity-transition events between versioned asset, service and system registers.

## Why this exists

A change in system counts does not identify which systems persisted, changed name, transferred owner, merged, split, were commissioned or were decommissioned. A one-to-one name join can create false continuity; a missing row can mean decommissioning, non-reporting, scope exclusion or a source omission. These are different claims and must remain distinct.

The South African Green Drop comparison demonstrates the problem. The 2022 report's full audit is described by WRO as 995 works across municipal, Department of Public Works and private ownership. The 2022 municipal comparison used 850 municipal systems. The 2025 full audit covers 848 municipal systems across 144 WSAs. SAnews attributes the two-system municipal count change to decommissioning or merging. No system-level identity linkage has been performed in this work. WRO says the 2025 report's system-level tables are not machine-readable and its individual PDF resource currently says "No License Provided"; this reference therefore uses only synthetic transitions.

A third-party comparator, Unmapped Dataworks UM-006, reports 3,103 records across Green Drop 2024, Blue Drop 2023/2024 and No Drop 2025, with Demarcation Board codes on municipality records. Its public notes say cross-assessment system links remain future work. It is recorded as a comparator lead only; FSRE does not import or redistribute its early-access records.

## Supported transition relations

- CONTINUITY: one source record to one target record; same entity asserted across revisions.
- RENAMED: one source to one target with a changed source-native label or code.
- TRANSFERRED: one source to one target where ownership or stewardship changes.
- MERGED_INTO: two or more distinct source records to one target record.
- SPLIT_INTO: one source record to two or more distinct target records.
- DECOMMISSIONED: source record only, no successor asserted.
- CREATED: target record only, no predecessor asserted.
- UNRESOLVED: source record exists but no successor is accepted.

Every grouped event binds one effective date, source/target system namespaces, evidence reference/class and review state. Merge and split events are explicit many-to-one and one-to-many groups, not lossy pseudo one-to-one rows. Conflicting same-date source or target assignments are rejected. Unresolved/disputed/provisional identity is never promoted to accepted continuity.

## Evidence and review semantics

Accepted transitions require OFFICIAL_CROSSWALK, SOURCE_NATIVE_CHANGE or INDEPENDENT_FIELD evidence classes. Operator-only assertions are not enough for an accepted crosswalk without separate qualifying support. Synthetic evidence can only have the SYNTHETIC_ONLY review state. This is a structural conformance boundary; it does not authenticate evidence or establish legal ownership.

## Run

From the repository root:

    cargo test --locked --offline --manifest-path tools/fsre-identity-crosswalk/Cargo.toml
    cargo run --locked --offline --manifest-path tools/fsre-identity-crosswalk/Cargo.toml -- tools/fsre-identity-crosswalk/fixtures/synthetic-transitions.psv

The synthetic fixture covers eight event groups and ten transition rows: continuity, rename, transfer, a two-to-one merge, a one-to-two split, decommissioning, creation and unresolved identity. Tests exercise group cardinality, duplicate/conflicting assignments, event dates, evidence/review promotion and row-order invariance.

The output always states real_system_identity_linkage=NOT_ESTABLISHED, source_authenticity=NOT_ASSESSED and operational_authority=NONE for this synthetic fixture. This is a contract validator, not a real Green Drop crosswalk or entity-resolution algorithm.
