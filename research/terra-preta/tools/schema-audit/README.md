# Terra Preta JSON Schema audit

This standalone Rust utility goes beyond parsing JSON syntax: it checks the schema document against its declared JSON Schema draft's meta-schema, builds a validator, and can validate a report instance against the schema.

## Run

```sh
cargo test --manifest-path research/terra-preta/tools/schema-audit/Cargo.toml
cargo run --manifest-path research/terra-preta/tools/schema-audit/Cargo.toml -- \
  research/terra-preta/tools/split-audit/split-audit-report.schema.json
cargo run --manifest-path research/terra-preta/tools/schema-audit/Cargo.toml -- \
  research/terra-preta/tools/split-audit/split-audit-report.schema.json \
  /path/to/split-audit-report.json
```

The tool has no default HTTP-resolution features enabled. Terra Preta schemas use local references; introducing remotely resolved schemas would make validation dependent on external network state and is not part of this contract.

The eleven unit tests meta-validate all eight committed contract schemas, validate the checked-in source register and synthetic prediction/dependency-audit report instances, reject invalid date/nominal-coverage values and negative interval scores, and exercise split-report negative controls. Instance validation explicitly enables URI/date format checks rather than relying on the library default. These are schema conformance tests, not scientific-data or efficacy tests.

The older `jq empty` syntax checks are not sufficient to establish JSON Schema validity. This tool is intended to make that distinction enforceable in the focused research CI. As with all software dependencies, pin/update and review the validator dependency using the repository's normal supply-chain policy.
