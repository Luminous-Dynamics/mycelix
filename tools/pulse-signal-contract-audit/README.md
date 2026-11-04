# Pulse signal contract auditor

This standalone Rust source-contract checker enforces parity across four Holochain signal producers, the browser SignalHub decoder, TypeScript discriminated unions, and the v1 JSON Schema.

It parses the Rust enums with syn, extracts Serde representation and named fields, reads the matching TypeScript unions, inventories both SignalHub category-filter passes, and compares each variant's field set against the schema. It also verifies that SignalHub explicitly decodes the adjacent-tag { type, data } envelope and normalizes only known variants.

## Current v1 wire contract

- Rust: all four enums use #[serde(tag = "type", content = "data")].
- Schema: docs/security/pulse-zome-signal-wire-v1.schema.json; exactly 28 variants, closed top-level and per-variant objects.
- TypeScript: normalized discriminated unions that exactly match the producer fields.
- Browser decoder: validates envelope shape, exact field sets, nested EmailStateUpdate, finite numeric values and known variants; unknown variants are rejected.
- Trust notifications are advisory wake-up hints. Consumers must fetch and validate exact backing records before promoting any claimed state.

## Run

From the repository root:

    cargo test --manifest-path tools/pulse-signal-contract-audit/Cargo.toml
    cargo generate-lockfile --manifest-path tools/pulse-signal-contract-audit/Cargo.toml
    cargo run --manifest-path tools/pulse-signal-contract-audit/Cargo.toml --locked -- \
      --root mycelix-workspace/mycelix-pulse \
      --output target/pulse-signal-contract-report.json

The qualification gate requires zero missing/extra variants, exact Rust/TypeScript/schema field parity, exactly two complete filter lists per category (parse and dispatch), adjacent type/data serialization for every producer, a strict envelope decoder, and exactly 28 schema variants.

The CI workflow uploads the source-derived report even on failure. Do not use --report-only for qualification; it is reserved for a temporary baseline inventory and intentionally does not fail when findings exist.

## Security and scope limits

A type-correct signal is not an authenticated sender, a valid attestation, a current trust score, a revoked capability, or an authorization grant. A remote signal cannot authorize a protected operation or mutate trusted state by itself. SignalHub does not infer transport sender identity from payload fields.

The auditor does not prove sender authenticity, DHT record validity, source-chain authorship, secret non-disclosure or two-conductor behavior. Those remain runtime qualification requirements in #5772, #5777 and #5778.

The current auditor scopes four active zome enums and the main browser client. It does not yet inventory every local-only/remote construction site across legacy and SDK packages. That is a separate discovery tranche. Unknown or unparsed constructs must produce an incomplete/failing result rather than a clean pass.
