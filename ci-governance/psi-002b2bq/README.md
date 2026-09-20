# PSI-002B2BQ — runner-neutral exact-source qualifier

Status: **PREPARED / NOT EXECUTED / NOT A PASS**

Exact product subject:

```text
d6c8acf5ad2e88f7454fe161f75e2446df5e2c26
```

This qualifier adds no hosted workflow. It materializes the exact B2A registry-evidence dependency and B2B Xenia provider consumer from canonical Git blobs into a temporary sibling-crate layout, then runs from the B2B standalone workspace.

## Execution gate

```text
cargo fmt --check --all
cargo test --offline --all-targets
cargo clippy --offline --all-targets --all-features -- -D warnings
```

No network fallback is permitted.

## Static invariants

The qualifier also binds and probes:

- exact B2A parent commit/tree and dependency blobs;
- exact three B2B product blobs;
- Ed25519 + ML-DSA-65 dependency/profile;
- registry-specific provider-attestation domain;
- registry-specific verifier-identity domain;
- fixed authenticity-only claim marker;
- absence of a signed currentness field/profile;
- exact structural rebind before provider crypto;
- independently trusted verifier identity comparison;
- both provider-signature verification paths;
- private/non-deserializable final positive type;
- explicit `registry_current() == false` claim ceiling;
- exact committed Rust test count.

## Success ceiling

A real PASS may establish only:

```text
exact B2B source compiled
registered B2B tests passed
consumer-side hybrid verification path executed
B2A structural join consumed exactly
```

It does not establish:

```text
Xenia producer type-gating qualified
registry currentness
PSI cryptographic security
query-credit privacy
OHTTP transport privacy
full contact-discovery composition qualification
production admission
application authority
```

The receipt must be written outside the checkout.
