# Pulse signal contract auditor

This standalone Rust source-contract checker enforces parity across four Holochain signal producers, the browser SignalHub decoder, TypeScript discriminated unions, and the v1 JSON Schema.

It parses the Rust enums with syn, extracts Serde representation and named fields, reads the matching TypeScript unions, inventories both SignalHub category-filter passes, and compares each variant's field set against the schema. It also verifies that SignalHub explicitly decodes the adjacent-tag { type, data } envelope, rejects malformed hash/timestamp/enum/vector-clock values, and normalizes only known variants. The JSON Schema audit rejects unconstrained properties and recursively checks nested definitions.

## Current v1 wire contract

- Rust: all four enums use #[serde(tag = "type", content = "data")].
- Schema: docs/security/pulse-zome-signal-wire-v1.schema.json; exactly 28 variants, typed field definitions, closed top-level/per-variant/nested objects, bounded arrays and enum constraints. Portable JSON encodes Holochain hashes as base64url strings and byte vectors as base64; the decoded browser API uses Uint8Array values.
- TypeScript: normalized discriminated unions that exactly match the producer fields. SyncOpType is aligned to every Rust variant, including FolderMoved, DraftDeleted, filters, trust attestations and contacts.
- Browser decoder: validates envelope shape, exact field sets, 39-byte ActionHash/AgentPubKey values, Uint8Array payloads, safe integer timestamps/counters, nullable thread IDs, nested EmailStateUpdate/folder hashes, vector clocks, exact enum variants and finite/ranged score values; unknown variants are rejected.
- Trust notifications are advisory wake-up hints. Consumers must fetch and validate exact backing records before promoting any claimed state.

## Run

From the repository root:

    cargo test --manifest-path tools/pulse-signal-contract-audit/Cargo.toml
    cargo generate-lockfile --manifest-path tools/pulse-signal-contract-audit/Cargo.toml
    cargo run --manifest-path tools/pulse-signal-contract-audit/Cargo.toml --locked -- \
      --root mycelix-workspace/mycelix-pulse \
      --output target/pulse-signal-contract-report.json

The qualification gate requires zero missing/extra variants, exact Rust/TypeScript/schema field parity, no unconstrained JSON Schema properties, exact recursive definitions, exactly two complete filter lists per category (parse and dispatch), adjacent type/data serialization for every producer, runtime field-type validation, and exactly 28 schema variants.

The CI workflow uploads the source-derived report even on failure. Do not use --report-only for qualification; it is reserved for a temporary baseline inventory and intentionally does not fail when findings exist.

## Security and scope limits

A type-correct signal is not an authenticated sender, a valid attestation, a current trust score, a revoked capability, or an authorization grant. A remote signal cannot authorize a protected operation or mutate trusted state by itself. SignalHub does not infer transport sender identity from payload fields.

The auditor does not prove sender authenticity, DHT record validity, source-chain authorship, secret non-disclosure or two-conductor behavior. Those remain runtime qualification requirements in #5772, #5777 and #5778.

The current auditor scopes four active zome enums and the main browser client. It does not yet inventory every local-only/remote construction site across legacy and SDK packages. That is a separate discovery tranche. Unknown or unparsed constructs must produce an incomplete/failing result rather than a clean pass.
