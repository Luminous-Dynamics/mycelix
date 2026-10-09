# CIV-013 Witness Conformance v1

**Status:** reference vectors / experimental; not a production qualification.  
**Protocol version:** `civ-013-conformance-v1`  
**Source model observed:** Symthaea `5c4b0c8b2c8ea2f66b97e9741c32b01c7a294ce7`  
**Fixture manifest:** [civ-013-witness-conformance-v1-manifest.json](civ-013-witness-conformance-v1-manifest.json)  
**Golden vectors:** [civ-013-witness-conformance-v1.json](civ-013-witness-conformance-v1.json)  
**Independent reference checker:** [verify_civ013_witness_conformance_v1.py](../../scripts/integral/verify_civ013_witness_conformance_v1.py)

## Purpose

These deterministic vectors establish a shared reference for canonical witness-record encoding and selected protocol outcomes across Symthaea and Mycelix. Version 1 now covers 19 vectors, including cross-log successor rejection, monotonic receipt-tail history, invalid candidate-tail shape, and explicit bootstrap-anchor binding. The checker is intentionally implemented separately from the Rust producer model, using only Python's standard library. It consumes serialized fixture bytes/fields and expected results; it does not import or execute the Rust reducer.

This is a compatibility starting point. Before treating v1 as normative production wire format, both projects must review and accept its field semantics, compatibility policy, and trust assumptions. No production storage implementation should be assumed from this fixture suite alone.

## Canonical encoding

All integers are unsigned and big-endian. Variable byte strings are prefixed by an unsigned 64-bit byte length (not character count). UTF-8 strings must be non-empty. Digests are exactly 32 raw bytes represented as 64 hexadecimal characters in JSON.

A record is encoded as the concatenation of:

1. Domain separator `mycelix-civ013-witness-record-v1\0`.
2. Protocol version as u16.
3. Generation as u64.
4. Length-prefixed UTF-8 log ID.
5. Length-prefixed UTF-8 policy version.
6. The 32-byte anchor digest.
7. Receipt sequence as u64.
8. Optional receipt digest: byte `0x00` for absent, or `0x01` followed by a 32-byte digest.
9. Optional previous-record digest using the same optional-digest encoding.

The record digest is SHA-256 over those bytes. The commit-marker digest is SHA-256 over domain separator `mycelix-civ013-commit-marker-v1\0`, generation as u64, and the 32-byte record digest. Fork-evidence digest uses domain separator `mycelix-civ013-fork-evidence-v1\0`, generation as u64, first record digest, conflicting record digest, and optional prior evidence digest.

The canonical record digest binds the accepted-record fields. It is not a digital signature and does not authenticate the issuer.

## Vector inventory

| Vector | Expected outcome | Property exercised |
|---|---|---|
| V001 | ACCEPT | Trusted bootstrap shape and record/marker digests |
| V002 | ACCEPT | Exact predecessor link, generation successor, record and marker digests |
| V003 | REJECT_STALE_PREDECESSOR | Candidate names the wrong accepted predecessor |
| V004 | REJECT_GENERATION_GAP | Generation skips the current successor |
| V005 | REJECT_ROLLBACK_DETECTED | Independently retained anchor is ahead of restored local state |
| V006 | REJECT_ANCHOR_UNAVAILABLE | No external anchor; fail closed |
| V007 | REJECT_BAD_MARKER | Commit-marker digest mismatch |
| V008 | REJECT_BAD_RECORD_DIGEST | Record field changed without matching digest |
| V009 | ACCEPT_EVIDENCE_ONLY | Valid fork evidence is retained as evidence, not acceptance |
| V010 | REJECT_REORDERED_FORK_CHAIN | Reordered evidence breaks the predecessor-evidence chain |
| V011 | IDEMPOTENT_SAME_RECEIPT | Exact duplicate transition returns the same terminal outcome |
| V012 | REJECT_CAS_LOST | A competing successor already won the same generation |
| V013 | REJECT_RECEIPT_ROLLBACK | A candidate receipt sequence regresses |
| V014 | REJECT_RECEIPT_TAIL_EQUIVOCATION | Same sequence with a different digest is an equivocation |
| V015 | REJECT_LOG_ID_MISMATCH | A successor attempts to cross into another log identity |
| V016 | REJECT_RECEIPT_TAIL_HISTORY_INVALID | A rehashed committed chain regresses its receipt sequence |
| V017 | REJECT_RECEIPT_TAIL_HISTORY_INVALID | A rehashed committed chain changes the digest at the same sequence |
| V018 | REJECT_BAD_RECEIPT_TAIL | Sequence zero cannot carry a tail digest |
| V019 | REJECT_UNTRUSTED_BOOTSTRAP | A valid record/marker cannot override a separately supplied trust anchor |

## Required interpretation rules

- Bootstrap must bind to a separately supplied trusted anchor commitment. A syntactically valid record and marker cannot establish their own trust by being the first item observed.
- Receipt-tail shape is canonical: sequence zero means no tail digest; a positive sequence requires a digest. The accepted sequence cannot regress, and equal-sequence/different-digest evidence is equivocation.
- Identical retries are idempotent only when they match the exact committed transition and receipt identity.
- A competing writer's result must not be overwritten by a stale candidate.
- A generation gap or wrong predecessor does not get repaired by guessing.
- The absence of an independent anchor means rollback protection is unavailable; it must not silently degrade into a local hash-chain claim.
- Fork evidence must remain separate from accepted state and must not advance, replace, or rewrite the accepted checkpoint.
- An unresolved commit outcome is not success; real backend work must define how it becomes reconciled or remains indeterminate.

## Conformance and evidence record

The manifest freezes the required vector IDs and source-model reference. The workflow must bind each run to the exact checked-out commit, verifier and fixture paths, toolchain/runtime versions, manifest digest, fixture digest, and a per-vector PASS/FAIL result. Missing, stale, cancelled, skipped, or queued runs do not pass.

The reference checker must be used as an independent implementation, not imported as production authority. If a production implementation intentionally differs, update the protocol version or add an explicit compatibility rule plus reviewed vectors; do not silently change the fixture expectations.

## Qualification limits

These vectors do **not** establish:

- actual filesystem or SQLite crash consistency;
- real `fsync`, atomic rename, directory synchronization, or cross-process locking behavior;
- hardware-backed or otherwise independently durable monotonic anchors;
- authentic signature verification, key custody/rotation, operational witness independence, quorum safety, or public gossip/distribution;
- global non-equivocation, governance legitimacy, the truth of the underlying physical claim, or legal/regulatory compliance.

Those are separate proof obligations. The next backend stage must run fault injection against the actual persistence adapter, not just against a simulated snapshot model. SQLite's own crash-test model is a useful reference for varying incomplete and reordered writes and checking recovery after reopening: https://www.sqlite.org/atomiccommit.html. For transparency witnesses, see https://github.com/transparency-dev/witness and the split-view discussion in https://github.com/sigstore/architecture-docs/blob/main/rekor-spec.md.
