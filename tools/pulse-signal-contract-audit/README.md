# Pulse signal contract auditor

This standalone Rust source-contract checker covers the four Holochain signal enums currently consumed by the Pulse client.

It parses the Rust enums with syn, extracts Serde enum representation, enumerates named fields, reads the matching TypeScript discriminated unions and both SignalHub category-filter passes. It emits deterministic JSON and exits non-zero when the client is missing a Rust variant, has stale variants/fields, omits a filter or assumes an incompatible serialized shape.

Usage:

    cargo test --manifest-path tools/pulse-signal-contract-audit/Cargo.toml
    cargo run --manifest-path tools/pulse-signal-contract-audit/Cargo.toml -- \
      --root mycelix-workspace/mycelix-pulse \
      --output target/pulse-signal-contract-report.json

    # Preserve the FAIL status and findings in JSON while returning exit code 0
    # for an initial baseline inventory:
    cargo run --manifest-path tools/pulse-signal-contract-audit/Cargo.toml -- \
      --root mycelix-workspace/mycelix-pulse \
      --output target/pulse-signal-contract-report.json \
      --report-only

Run from the repository root. A non-zero result is expected on the current migration source: this is a fail-closed inventory, not a parser that silently normalizes incompatible encodings.

Scope limitations:
- This first version inventories the four active zome enums and the current browser client only.
- It does not prove remote sender authenticity, validate DHT records, or establish that a notification may be treated as trusted state.
- The TypeScript extractor is deliberately limited to the current type-discriminant union convention. Unsupported syntax must be treated as an audit failure, not a clean pass.
- It does not yet inventory every local-only or remote signal construction site; that is the next extension.
