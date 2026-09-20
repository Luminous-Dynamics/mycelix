# PSI-002B2AQ — runner-neutral exact-source qualifier

Status: **PREPARED / NOT EXECUTED / NOT A PASS**

Exact product: `d63eb7c2dec46c3b0b6faeee3062bdfded0ba3b6`.

This qualifier binds the exact three-file PSI-002B2A product, its B1 parent, exact 10-case source corpus, and an offline isolated Rust execution surface.

A successful execution may establish only exact source compilation and registered structural tests. It cannot establish provider cryptography, registry authentication/currentness, trust-policy authority, composition qualification, or production admission.

Required execution:

```text
cargo fmt --check --all
cargo test --offline --workspace
cargo clippy --offline --workspace --all-targets -- -D warnings
```

No hosted workflow and no network fallback are included.
