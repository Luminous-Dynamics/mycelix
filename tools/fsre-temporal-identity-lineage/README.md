# FSRE temporal identity lineage v0.1

**Status:** Dependency-free reference implementation for validating typed, evidence-bound identity events and replaying a bitemporal history. It does not authenticate source owners, prove physical identity, or establish population completeness.

## Run

    cargo test --locked --offline --manifest-path tools/fsre-temporal-identity-lineage/Cargo.toml
    cargo run --quiet --locked --offline --manifest-path tools/fsre-temporal-identity-lineage/Cargo.toml -- tools/fsre-temporal-identity-lineage/examples/water-works.psv 40 40

The CLI arguments are the recorded-time cutoff and effective-time cutoff, respectively. Both are explicit: a late-arriving event cannot rewrite what was knowable at an earlier decision cutoff.

## Event contract

Pipe-delimited rows use this exact header:

    event_id|event_type|predecessors|successors|effective_at|recorded_at|source_snapshot|evidence_owner|evidence_ref|adjudicator|rule_version

Predecessor/successor lists are comma-separated immutable entity IDs; `-` denotes an empty side. Supported event types: RENAME, IDENTIFIER_REASSIGNMENT, RELOCATION, OWNERSHIP_TRANSFER, OPERATOR_TRANSFER, SPLIT, MERGE, RETIRE, RECOMMISSION and SERVICE_BOUNDARY_CHANGE.

Rename, relocation, ownership/operator transfer and service-boundary changes preserve the entity ID. Identifier reassignment changes identity. Split is 1:N; merge is N:1; retirement is 1:0; recommissioning is 0:1. Every event is bound to a source snapshot, evidence owner/reference, adjudicator and rule version.

## Claim boundary

The replay is deterministic and read-only. A successful replay means only that the supplied event log passes the implemented structural checks for the selected bitemporal view. It does not authenticate evidence, resolve contested real-world identity, allocate service entitlements across splits/merges, or prove register completeness. Unknown or disputed identity must remain unresolved in downstream FSRE qualification.
