# Maritime Commons Handoff Verifier

Standalone, effects-disabled Rust validation for the cross-stack evidence handoff manifest proposed in Mycelix #5968 / PR #5969.

## What it verifies

- Strict JSON deserialization: unknown fields and duplicate struct keys are rejected.
- Absent optional fields remain distinct from explicit JSON null.
- Artifact owner/repository pairing, commit shape and SHA-256 digest shape.
- Unique artifact and edge IDs.
- Handoff source/output/transformation/verifier references resolve locally.
- Produced artifact owner matches the declared destination owner.
- Preserved versus lossy-declared and unresolved handoff semantics.
- Contract-template, synthetic-unexecuted and executed-frozen state consistency.
- Exact-input-byte SHA-256 and a stable, sorted validation report.

## What it does not verify

The verifier does not fetch referenced files, compare their digests against remote bytes, authenticate issuers, validate credentials, establish local legal recognition, check source rights, calculate physical feasibility, authorize providers, or perform external effects.

A repository commit and content digest identify a claimed artifact; they are not proof of source authenticity or truth. Domain owners retain their own verification and authority boundaries.

The four owner/repository mappings are a Luminous cross-stack profile v0.1 allow-list, not a universal Maritime Commons rule.

## Run

```sh
cargo test --all-targets
cargo fmt -- --check
cargo clippy --all-targets -- -D warnings
cargo run -- path/to/manifest.json
```

The CLI reads one local JSON file and writes a JSON report to stdout. Exit codes: 0 for Pass, 1 for Reject/Conflict, 2 for Unknown/NotEvaluable, 64 for usage, 65 for invalid input, 66 for file read failure, and 74 for output failure.

## Lockfile status

This initial review branch does not yet contain a committed Cargo.lock. Its pull-request workflow generates a provisional lockfile on the exact review head, runs all Cargo checks with that lockfile, and uploads it as a review artifact. Before merge, the generated lockfile must be inspected and committed; until then dependency resolution is not yet pinned in version control.

## Qualification boundary

A PASS establishes only the local structural and handoff-reference checks for the exact manifest bytes and verifier build. It does not establish the truth, authenticity, currentness, lawfulness or operational authority of referenced maritime claims.
