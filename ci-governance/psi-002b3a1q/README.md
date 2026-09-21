# PSI-002B3A1Q — exact-source qualifier for RFC 9577 challenge binding

Status: **PREPARED / NOT EXECUTED / NOT A PASS**

Product: `31713561f743294c3ff0d4b3b33aa0ac62478c34`.

The qualifier binds the exact B3A1 crate plus corrected B3A r2 dependency blobs and requires the nine-case source corpus.

Static ratchets require the RFC 9577 default challenge field widths, Appendix A vector-1 67-byte canary and SHA-256 `8e1d5518ec82964255526efd8f9db88205a8ddd3ffb1db298fcc3ad36c42388f`, exact B3A semantic-challenge re-derivation, conservative one-Origin server-name profile, and explicit non-authority methods.

Offline execution:

```text
cargo fmt --check --all
cargo test --offline --all-targets
cargo clippy --offline --all-targets --all-features -- -D warnings
```

No Privacy Pass implementation, HTTP client/server, Holochain, Xenia or network dependency is admitted in this crate.

Even a PASS establishes only exact RFC 9577 challenge encoding under this profile; it does not verify a token or grant a query credit.
