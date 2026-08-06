# mycelix-zkp-core capability matrix

This ledger separates protocol types, structural validation, cryptographic
verification, and runtime evidence. A feature flag or serializable proof type is
not evidence that a verifier is operational in a Holochain zome.

| Capability | Source status | Cryptographic status | Runtime evidence in this archive |
|---|---|---|---|
| Authenticated proof envelope v2 | Implemented | Backend, proof hash, public-input hash, nonce, timestamp, identity, domain, and integer energy claim are signature-bound | Source tests only |
| Envelope policy validation | Implemented | Structural and policy validation only | Source tests only |
| Dilithium5 authentication | Feature-gated implementation | Signs and verifies canonical envelope digests | Requires `cargo test --features dilithium` outside this review environment |
| Winterfell range/XOR circuits | Feature-gated circuit code | Circuit-specific STARK proving and verification | No fresh build or Holochain Wasmer evidence in this archive |
| Miden consciousness circuit | Feature-gated circuit code pinned to 0.23.5 | Intended zero-knowledge circuit-specific proving and verification | No fresh build or Holochain Wasmer evidence in this archive |
| RISC Zero backend | Structural adapter only | No RISC Zero dependency, prover, receipt parser, or image-ID verifier is linked | None |
| Binius backend | Reserved identifier only | Not implemented | None |
| Merkle membership envelope | Implemented | Format validation and ordinary Merkle utilities only; no ZK verifier in this module | Source tests only |
| Nullifier envelope | Implemented | Format validation and in-memory replay helper only; no ZK membership verifier in this module | Source tests only |
| Supply proof statements | Implemented | Domain-separated public-input commitments and exact envelope binding | Source tests only; proof circuits remain to be implemented per statement kind |
| Supply verification records | Implemented | Canonical append-only verifier attestation format; verifier signature algorithm is application-selected | Source tests only |

## Acceptance terminology

- **Structure-valid** means lengths, versions, timestamps, identifiers, and
  commitments satisfy a local policy.
- **Signature-valid** means the authenticated envelope or verifier record was
  checked against the expected public key.
- **Proof-valid** means a circuit-specific verifier accepted the proof against
  the exact public inputs and expected program/AIR identifier.
- **Operational in Holochain** requires conductor/Wasmer tests in addition to
  native Rust tests.

Applications should store explicit outcomes (`valid`, `invalid`, `unsupported`,
`expired`) together with verifier identity, implementation version, policy hash,
and timestamp. They should not persist a mutable `verified: bool` detached from
that evidence.
