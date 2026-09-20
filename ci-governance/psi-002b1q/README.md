# PSI-002B1Q — runner-neutral exact-source qualifier

Status: **PREPARED / NOT EXECUTED / NOT A PASS**

Exact product:

```text
c5c892ab096c5454f8e0bcad87940e0e7ac1a849
```

This qualifier adds no hosted workflow. It binds the exact PSI-002B1 product, its PSI-002A parent, the inherited PEC semantic core, the 13-case source corpus, and an offline isolated Rust command vector.

A successful execution may establish only that the exact structural source compiled and its registered tests passed in that bound environment.

It may not establish query-token unlinkability, abuse resistance, OHTTP privacy, registry authenticity/currentness, contact-discovery privacy, composition qualification, production admission, or application authority.

The isolated workspace contains:

```text
privacy-computation-core
psi-contact-discovery-policy
```

and requires:

```text
cargo fmt --check --all
cargo test --offline --workspace
cargo clippy --offline --workspace --all-targets -- -D warnings
```

Missing cached dependencies are a truthful FAIL. The receipt must be written outside the checkout.
