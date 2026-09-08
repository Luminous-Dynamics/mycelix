# SSF Canonical Replay Wire Tags v0.1

Stable leaf-level wire tags for the canonical replay-evidence decision graph.

This crate deliberately assigns explicit byte tags to closed replay semantics instead of serializing Rust enum layout or declaration order. It also encodes completed-invocation record commitments with an explicit provenance tag so `LegacyReplay(record X)` and `CanonicalReplay(record X)` remain distinct wire identities even when their underlying 32-byte record commitment matches.

Covered leaf semantics in v0.1:

- decision purpose domain;
- replay-evidence time basis;
- pre-invocation time basis;
- execution-generation time basis;
- source-owned provider time basis;
- initial replay-evidence basis;
- completed invocation provenance + record commitment;
- completed outcome terminal state;
- actuator recovery mode;
- effect recovery policy.

Equal numeric tags in different semantic domains are not interchangeable: the enclosing encoder function/profile supplies the domain. Composite encoders must therefore preserve explicit field order and must not concatenate tag values without their surrounding semantic structure.

This is **not** a complete canonical encoding of `CanonicalReplayEvidenceDecisionSubjectV1`. Composite history, evidence, attempt, receipt, time, actuator, and request structures still require their own explicit encoders before full decision-subject coverage can be claimed.

The crate performs no hashing, signing, verification, evidence qualification, replay authorization, or effect authorization.
