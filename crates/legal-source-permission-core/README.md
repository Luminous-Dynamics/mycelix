# Legal Source Permission Core (synthetic-only v0.1)

This standalone Rust crate evaluates whether a narrowly described source operation is admitted by a versioned, reviewed source-use profile. It is an **admission-control mechanism**, not a legal determination.

It is intentionally separate from source ingestion, legal interpretation, Holochain persistence, model training, and any production legal database. The included profile and requests are fictional fixtures.

## Core principle

Permissions are operation-scoped, purpose-scoped, actor-scoped, territory-scoped, time-scoped, and retention-scoped:

- permission to read does not imply permission to retrieve automatically;
- permission to retrieve does not imply permission to cache, index, embed, train, evaluate, display or redistribute;
- a source terms change or digest mismatch invalidates admission until reviewed;
- an unknown operation or unknown permission does not become an implicit grant;
- a source component/version mismatch does not inherit another snapshot's grant;
- a revoked profile is denied; draft/expired/unreviewed profiles are not admitted;
- overlapping applicable denied grants take precedence; overlapping allowed/unknown grants are treated as ambiguous and not admitted.

The evaluator has no network access and does not fetch, retain, or redistribute source materials.

## Run

\`\`\`sh
cargo test --manifest-path crates/legal-source-permission-core/Cargo.toml
cargo run --manifest-path crates/legal-source-permission-core/Cargo.toml -- validate crates/legal-source-permission-core/fixtures/fictional-source-profile-v0.1.json
cargo run --manifest-path crates/legal-source-permission-core/Cargo.toml -- check crates/legal-source-permission-core/fixtures/fictional-source-profile-v0.1.json crates/legal-source-permission-core/fixtures/benchmark-evaluation-allowed-v0.1.json
\`\`\`

The \`check\` command emits JSON for \`admitted\`, \`denied\`, or \`unknown\`. A valid denial/unknown result is a successful evaluation and exits 0; malformed profiles/requests fail with a nonzero code. CI asserts the expected fixture decisions explicitly.

## Evidence receipts

Every CLI admission receipt includes:
- SHA-256 of the exact profile file bytes and request file bytes;
- profile and evaluator versions;
- requested component/version, operation, purpose, actor, territory, as-of timestamp and retention;
- expected and observed terms digests;
- selected grant (when one applies), permission decision, admission decision, and reason code.

A digest establishes identity of the bytes that were hashed. It does not authenticate the publisher, establish the truth of a source, prove the legal validity of terms, or establish that a source is authoritative.

## Permission semantics

- \`allowed\`: the operation is explicitly allowed in a matching, current grant.
- \`denied\`: the profile explicitly denies the operation, the request exceeds a permitted scope/retention limit, or the source operation cannot be admitted within the declared profile.
- \`unknown\`: terms/currentness/reviewer approval are unverified, the terms digest is missing or mismatched, the source component/version differs, or applicable grants conflict between allowed and unknown.

The receipt distinguishes \`permission_decision\` from \`admission_decision\`: an operation may be allowed in principle while a request is denied because its requested retention exceeds the grant limit.

Timestamps must be normalized UTC strings in \`YYYY-MM-DDTHH:MM:SSZ\`; intervals use inclusive starts and exclusive ends. Profile and request source-version equality is exact.

## License and source rights

The root Mycelix repository documents that shared crates under \`crates/\` are AGPL-3.0-or-later; this crate therefore declares that license explicitly. See the repository's [LICENSING.md](https://github.com/Luminous-Dynamics/mycelix/blob/main/LICENSING.md).

SAFLII's published terms explicitly restrict a range of AI/automated uses of its collection and associated metadata absent separately documented permission: https://www.saflii.org/content/terms.html. This crate must encode source-specific terms and never infer that public readability is permission for automated retrieval, indexing, evaluation, training or redistribution. The included fictional profile says nothing about real legal materials.

## Limits / next steps

This v0.1 profile contract does not prove that a represented agreement is legally valid, that a reviewer is qualified, that every relevant rights-holder has granted permission, or that an operation is lawful under every applicable law. Deployment requires source-specific review and evidence for each permission record.

Follow-on work should integrate durable Mycelix provenance, term-change monitoring, revocation/takedown propagation and independently verified profile-review records. Keep the operation evaluator deterministic, fail-closed and independently testable.
