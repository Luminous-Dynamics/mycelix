# SIF v0.1 Hardening Gate

Before expanding the protocol surface, the current accountability slice must pass these gates:

- core formatting, tests, and clippy on the supported Rust toolchain;
- Civic consumer compilation;
- provider-neutral verifier formatting, tests, and clippy;
- canonical commitment encoding has no runtime dependency on `bincode` or another language-specific serializer;
- reordering set-semantic fields does not change commitments;
- semantic mutations do change commitments;
- proof references remain excluded from the frozen receipt statement;
- provider verification receives only commitment-level cross-stack context;
- Xenia and Symthaea can bind the same frozen statement and live-operation nonce without receiving citizen/case plaintext.

Only after these gates are green should v0.1 add verifier-key/trust-domain threshold policy or persistence state machines. This ordering keeps CI failures attributable to a small surface and prevents security semantics from accumulating on an unqualified commitment boundary.
