# SSF Canonical Replay Evidence Encoding Profile v0.1

Deterministic, non-authoritative wire-profile contract for the future canonical encoding of `CanonicalReplayEvidenceDecisionSubjectV1`.

This tranche freezes the byte-level rules before any cryptographic adapter is allowed to bind the decision subject:

- fixed domain prefix `MYCELIX-SSF/REPLAY-EVIDENCE/V1\0`;
- explicit encoding-profile version;
- unsigned integers use big-endian bytes;
- enum discriminants use explicit one-byte tags;
- optional values use an explicit `0` / `1` tag before any payload;
- fixed 32-byte commitments are encoded as their exact 32 bytes;
- fields use deterministic `field_id:u16_be || length:u32_be || value` framing;
- fields within each structure must be encoded exactly once in strictly increasing field-id order;
- Rust memory layout, `Debug`, JSON, maps, platform-native integers, and implicit enum discriminants are not canonical encodings.

The crate also freezes reviewed field-ID tables for the three stable outer structures:

1. `CanonicalReplayEvidenceDecisionSubjectV1`;
2. `CanonicalReplayEvidenceQualificationRequestSubjectV1`;
3. `CanonicalReplayEvidenceSubjectV1`.

Those tables establish field order now rather than letting a future crypto-facing encoder invent numbering later. Deeper history/effect structures remain explicitly outside completed coverage until their own canonical field tables and traversal are reviewed.

The crate provides a `no_std` sink interface plus primitive framing helpers so a future exhaustive encoder can stream directly into a cryptographic implementation without allocation.

## Explicit incomplete-coverage boundary

This profile deliberately does **not** claim that the full decision subject has been exhaustively traversed yet. `CanonicalReplayEvidenceEncodingPlanV1` is created only with `ProfileOnly` coverage and therefore reports `may_enter_crypto_binding() == false`.

A later exhaustive-encoding tranche must cover every security-relevant semantic group listed in `REQUIRED_COVERAGE_V1` before any output may be called the canonical bytes of the decision subject.

This avoids the dangerous intermediate state where a stable-looking encoding silently hashes/signs only a convenient subset of the subject.

## Authority boundary

This crate performs no hashing, signature generation, signature verification, evidence qualification, replay authorization, effect authorization, or actuator operation.

The profile is a wire-contract prerequisite only.
