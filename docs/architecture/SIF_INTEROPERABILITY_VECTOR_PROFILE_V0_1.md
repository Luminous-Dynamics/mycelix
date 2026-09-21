# SIF v0.1 Interoperability Vector Profile

Status: draft conformance contract.

Every implementation of SIF v0.1 accountability commitments MUST reproduce the same canonical bytes and BLAKE3-256 commitments from the same semantic fixture. Mycelix owns the canonical semantic encoding; Xenia and Symthaea consume the resulting 32-byte statement and supporting commitment-only context as opaque public inputs.

## Fixture semantics

The reference fixture is the synthetic receipt exercised by `mycelix-accountability-core::canonical` tests. It deliberately includes fields whose insertion order must not affect commitments:

- disclosure classes `z-class`, `a-class`;
- subject rights `Contest`, `Know`, `Inspect`;
- inference provenance `receipt-z`, `receipt-a`;
- policy delay reasons `PublicSafety`, `ActiveInvestigation`, `ProtectVictimOrWitness`;
- policy evidence roles `ComputationProof`, `ExecutionBinding`.

Canonical encoding sorts and deduplicates those semantic sets. Attestation references are excluded from the pre-attestation receipt body.

## Conformance rules

1. A non-Rust implementation MUST derive the receipt and policy canonical bytes directly from `SIF_CANONICAL_COMMITMENTS_V0_1.md`, not from Rust serialization behavior.
2. The hash input is `domain || 0x00 || codec || 0x00 || canonical_body`, using BLAKE3-256.
3. Adding, removing, or reordering an attestation MUST NOT change the pre-attestation receipt commitment.
4. Reordering any set-semantic field MUST NOT change its canonical bytes or commitment.
5. A semantic mutation such as changing purpose text, query commitment, policy requirement, result commitment, or requester source MUST change the relevant commitment.
6. Xenia and Symthaea MUST NOT reserialize a full Mycelix receipt. They consume the frozen receipt statement and other commitment-only fields supplied by the verifier boundary.
7. A future incompatible encoding requires a new codec/domain version. Existing v0.1 vectors are immutable once promoted from draft to qualified.

## Qualification path

The next promotion step is to publish machine-readable byte/digest vectors generated from the fixture and have at least two independent implementations consume them. The vectors are not considered qualified merely because two Rust crates share the same implementation. Target consumers are:

- Mycelix canonical encoder;
- Xenia adapter/vector checker with no Mycelix serialization dependency;
- Symthaea adapter/vector checker or zkVM public-input fixture;
- one tiny reference implementation in another language for auditability.

This profile intentionally separates **semantic interoperability** from **proof-system interoperability**. A RISC Zero proof, Xenia signature, external witness, or future proof backend may have its own encoding internally; each must still bind the same SIF public statement and commitment-only context.
