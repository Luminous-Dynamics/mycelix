# Ignored Sweettest classification

Ignored tests are not a quality metric by count. Each test must be classified
before it is enabled, redesigned, or removed.

## Alpha-critical

- `phase0_two_conductor_harness_smoke`: repaired; subsumed by the V2 delivery
  test in the mandatory alpha command.
- `phase0_alice_sends_bob_receives`: V1 compatibility transport; keep runnable
  separately but do not make the classical send profile the alpha gate.
- `phase0_v2_hybrid_pqc_transport`: require after a fresh DNA pack; proves
  separate-conductor V2 transport, agent-bound hybrid key bundles, the
  conductor-owned agent signature, gossip, and exact opaque field recovery.
- Tests for signature forgery, timestamp bounds, inbox-link ownership,
  negative sender trust, receipt authorship, and duplicate message IDs.

## Valid post-alpha

- Attachments, scheduling, federation, advanced capability, multi-recipient,
  SMTP, ratcheting, cross-device recovery, and cross-cluster scenarios.

Hybrid PQC is part of the alpha path. The primitive behavior is mandatory in
the shared crypto crate; the Holochain transport behavior is mandatory in the
V2 Sweettest. They are separate executables because Holochain 0.6.1's release-
candidate crypto dependency graph cannot currently link with RustCrypto's
ML-KEM/ML-DSA graph in the same Cargo process.

## Classification procedure for remaining tests

For every remaining `#[ignore]`, record one of: alpha-critical, valid
post-alpha, superseded, obsolete architecture, nondeterministic/redesign, or
dead/remove. A test becomes mandatory only when it supports an alpha claim and
runs deterministically from fresh artifacts.
