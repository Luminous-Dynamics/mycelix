# PSI-002B3A3AQ — Exact-Source Qualifier

Status: **PREPARED / NOT EXECUTED / NOT A PASS**

Exact product subject:

```text
b4985d90813a3cb303eb5d253531b7fc981a3c13
```

This runner-neutral qualifier reconstructs only corrected B3A r2 plus the A3A issuer-directory structural crate from canonical Git objects.

## Structural ratchets

Before Cargo execution it requires:

- exact RFC issuer-directory media type;
- full key identity as SHA-256 of exact supplied key bytes;
- one-byte issuance ID derived separately from the final byte of the full ID;
- ordered key-list commitment;
- exact B3A `issuer_configuration_sha256` → observed response-body digest join;
- exact full token-key lookup, never truncated-ID lookup;
- deterministic truncated-ID collision regression;
- raw `not-before` retention with no currentness promotion;
- false authority methods for directory authentication/freshness, provider trust, SPKI profile verification, issuer-key admission/currentness, token verification, query credit and application authority;
- absence of HTTP/network/Holochain dependencies;
- absence of ambient clock APIs such as `SystemTime`, `Utc::now`, or `Instant::now`.

## Offline execution

The exact two-crate workspace is run with:

```text
cargo fmt --check --all
cargo test --offline --all-targets
cargo clippy --offline --all-targets --all-features -- -D warnings
```

Missing cached dependencies are a truthful FAIL. No network fallback is permitted.

## Claim ceiling

Even a successful execution may establish only exact structural source execution. It cannot establish directory payload authenticity, retrieval-provider trust, HTTP freshness, trusted-clock `not-before` satisfaction, issuer-key admission/currentness, token verification, query credit, production admission or application authority.
