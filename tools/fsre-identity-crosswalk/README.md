# FSRE temporal identity crosswalk reference

A dependency-free Rust validator for source-backed identity-transition events between versioned asset, service and system registers.

## Why this exists

A change in system counts does not identify which systems persisted, changed name, transferred owner, merged, split, were commissioned or were decommissioned. A one-to-one name join can create false continuity; a missing row can mean decommissioning, non-reporting, scope exclusion or a source omission. These are different claims and must remain distinct.

The South African Green Drop comparison demonstrates the problem. The 2022 report's full audit is described by WRO as 995 works across municipal, Department of Public Works and private ownership. The 2022 municipal comparison used 850 municipal systems. The 2025 full audit covers 848 municipal systems across 144 WSAs. SAnews attributes the two-system municipal count change to decommissioning or merging. No system-level identity linkage has been performed in this work. WRO says the 2025 report's system-level tables are not machine-readable. Its dataset page has conflicting licence metadata (prominent field: "No License Provided"; lower metadata: CC BY 4.0), and the individual PDF resource says "No License Provided". This reference therefore uses only synthetic transitions.

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

Every grouped event binds source and target system namespaces, record IDs, population scope, owner class, effective date, evidence reference/class and review state. The population scope distinguishes MUNICIPAL_SYSTEMS from ALL_SECTOR_WORKS and the owner class distinguishes MUNICIPAL, DPWI, PRIVATE and OTHER_PUBLIC. An identity may persist while its population scope changes (for example, a transfer from DPWI/all-sector scope into municipal scope); that event is reported separately and cannot be treated as a fixed-cohort comparison without a separate comparability review. Merge and split events are explicit many-to-one and one-to-many groups, not lossy pseudo one-to-one rows. Merges cannot silently combine source population scopes. Conflicting same-date source or target assignments are rejected. Unresolved/disputed/provisional identity is never promoted to accepted continuity.

## Evidence and review semantics

Accepted transitions require OFFICIAL_CROSSWALK, SOURCE_NATIVE_CHANGE or INDEPENDENT_FIELD evidence classes and known population scope and owner class on both endpoints. Operator-only assertions are not enough for an accepted crosswalk without separate qualifying support. Synthetic evidence can only have the SYNTHETIC_ONLY review state. This is a structural conformance boundary; it does not authenticate evidence or establish legal ownership.

## Run

From the repository root:

    cargo test --locked --offline --manifest-path tools/fsre-identity-crosswalk/Cargo.toml
    cargo run --locked --offline --manifest-path tools/fsre-identity-crosswalk/Cargo.toml -- tools/fsre-identity-crosswalk/fixtures/synthetic-transitions.psv

The committed fixture covers eight event groups and ten transition rows: continuity, rename, transfer, a two-to-one merge, a one-to-two split, decommissioning, creation and unresolved identity. The synthetic transfer deliberately changes from ALL_SECTOR_WORKS/DPWI to MUNICIPAL_SYSTEMS/MUNICIPAL; the CLI reports one population-scope change and one owner-class change and requires separate cohort comparability review. A separate bounded test generates 50 synthetic groups / 62 transition rows across all eight event types. The current library has 22 tests covering relation cardinality, duplicate/conflicting assignments, event dates, endpoint scope/owner-class completeness, merge-scope consistency, evidence-reference and namespace consistency within grouped events, relation consistency, evidence/review promotion, source-backed/synthetic separation and row-order invariance. The generated sweep is deterministic conformance coverage, not an independent sealed challenge set or a real matching-accuracy measurement.

The output always states real_system_identity_linkage=NOT_ESTABLISHED, source_authenticity=NOT_ASSESSED and operational_authority=NONE for this synthetic fixture. This is a contract validator, not a real Green Drop crosswalk or entity-resolution algorithm.
