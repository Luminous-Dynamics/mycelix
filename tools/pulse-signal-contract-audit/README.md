# Pulse signal contract auditor

This standalone Rust source-contract checker enforces parity across the four Holochain signal producers, the browser SignalHub decoder, TypeScript discriminated unions, and the v1 JSON Schema.

It parses the Rust enums with syn, extracts Serde representation and named fields, reads the matching TypeScript unions, inventories both SignalHub category-filter passes, and compares each variant's field set against the schema. It also verifies that SignalHub explicitly decodes the adjacent-tag `{ type, data }` envelope and normalizes only known variants.

The current v1 wire contract is:
- Rust: `#[serde(tag = "type", content = "data")]` on all four zome signal enums.
- Schema: `docs/security/pulse-zome-signal-wire-v1.schema.json` (28 variants, closed top-level and per-variant objects).
- TypeScript: normalized flat discriminated unions for existing UI consumers.
- Runtime decoding: rejects unknown variants, missing fields, extra fields and malformed state/numeric payloads before dispatch.

## Run

From the repository root:

    cargo test --manifest-path tools/pulse-signal-contract-audit/Cargo.toml
    cargo generate-lockfile --manifest-path tools/pulse-signal-contract-audit/Cargo.toml
    cargo run --manifest-path tools/pulse-signal-contract-audit/Cargo.toml --locked -- \
      --root mycelix-workspace/mycelix-pulse \
      --output target/pulse-signal-contract-report.json

The CI gate requires:
- zero missing or extra Rust/TypeScript variants;
- exact field-set equality among Rust, TypeScript and JSON Schema;
- exactly two complete SignalHub filter lists per category (parse and dispatch);
- all producers use the same adjacent-tag type/data encoding;
- SignalHub validates and normalizes that envelope;
- exactly 28 schema variants and zero findings.

A failing report is retained as a workflow artifact. Do not use `--report-only` for qualification; it exists only for baseline inventory and intentionally returns success even when the report status is FAIL.

## Security boundaries

Trust notifications remain unverified wake-up hints. A type-correct payload is not an authenticated sender, a valid attestation, a current trust score, a revoked capability, or an authorization grant. Consumers must fetch and validate the exact backing record before changing trusted state. SignalHub does not infer transport sender identity from payload fields.

The audit does not prove sender authenticity, DHT record validity, source-chain authorship, secret non-disclosure, or two-conductor behavior. Those remain runtime qualification work in #5772, #5777 and #5778.

The current auditor scopes the four active zome enums and the main browser client. It does not yet inventory every local-only or remote signal construction site across every legacy/SDK package; that is a separate discovery tranche. Unknown/unparsed constructs must be reported as incomplete rather than treated as clean.
