# Threat Model and Current Assurance Boundary — `mycelix-lawful-identity`

**Status:** Reconciled to the public source reviewed on 2 November 2026.  
**Review type:** Source and test inspection only; not an independent cryptographic audit, privacy proof, Holochain deployment test, or security certification.

> **Important correction:** Earlier versions of this document described cross-DID unlinkability as mathematically neutralized by STARK proofs. That statement is not supported by the current implementation. The current `cross-did-zkp` coordinator explicitly does not generate or verify a cryptographic proof; it stores caller-supplied proof strings and performs limited nonce-link bookkeeping. The `link_resistance.rs` and `unlinkability.rs` tests synthesize SHA-256-derived placeholder records; they do not generate or verify STARK proofs. Treat cross-DID cryptographic unlinkability as **not established**.

This document distinguishes implemented checks, design intent, test-model behavior, and future work. Do not rely on a status label without checking the exact deployed code and independent evidence.

---

## Adversary profile

A well-funded nation-state actor (intelligence agency, tax authority, or law-enforcement unit) with:

- Read access to the Mycelix Holochain DHT.
- ISP-level network surveillance.
- Knowledge of a target's `did:mycelix:legal` from a filing, KYC process, or lawful disclosure.

**Adversary's goal:** link the known legal identity to the target's pseudonymous primary identity and correlate governance, reputation, balances, social relationships, or published activity.

## Current implementation summary

| Surface | What the inspected source does | What it does not establish |
|---|---|---|
| `cross-did-zkp` coordinator | Issues a 32-byte nonce, stores a nonce request, accepts a caller-supplied `proof_value`, stores a record, and checks whether a nonce-hash anchor has a prior local link | It does not generate or cryptographically verify a STARK/ZKP; it does not bind the submitted nonce hash to a verifier-issued nonce entry; its `GetStrategy::Local` lookup is not a global replay-proof guarantee |
| `cross-did-zkp` integrity | Defines the stored entry and explicitly marks `proof_value` as unverified opaque data | Entry shape and Holochain validation do not prove the caller's predicate |
| `link_resistance.rs` | Synthesizes `CrossDidProof`-shaped records from deterministic hash inputs and scans fields for literal / hash substrings | It does not invoke a prover, real proof verifier, coordinator, or DHT |
| `unlinkability.rs` | Synthesizes placeholder records from SHA-256 outputs and checks byte distinctness plus a rough byte histogram | Distinct hashes and a histogram are not a privacy proof, cryptanalysis, or statistical unlinkability test |
| `jurisdiction_proof.rs` | With `backend-winterfell`, generates and verifies two range proofs that a supplied coordinate is within a selected bounding box | It does not prove the coordinate was truthfully measured, attested by a trusted device, or belongs to the person presenting it |
| `location_attestation.rs` | T0 checks the tier; T1 checks tier, optional public-key allowlist, and non-empty signature bytes; T2–T4 return `NotImplemented` | T1 does not cryptographically verify the device signature here; `max_age_seconds` is not enforced by `PhoneGps::verify`; no tier currently establishes hardware-backed location truth in this core crate |

## Vector 1 — Cryptographic leakage / proof validity — **NOT ESTABLISHED**

**Threat:** A proof construction or its public inputs expose deterministic identifiers, or the verifier accepts a false claim because it never verifies the proof.

**Current source boundary:**

- The coordinator's module-level honesty note states that no function in the crate invokes proof-verification logic.
- `submit_proof` accepts caller-supplied `proof_value` and persists it without decoding or validating a cryptographic proof.
- The synthetic link-resistance test creates a `CrossDidProof` struct with a SHA-256-derived placeholder `proof_value`. It does not call a STARK prover or verifier.
- The integrity zome's entry constraints validate structure, not the truth of the predicate.

The current test can be retained as a **synthetic serialization smoke test** for the fields it constructs. It must not be described as evidence that the real cross-DID architecture is unlinkable or that a STARK is correctly implemented.

**Required implementation before a cryptographic claim:**

1. Specify the exact statement, public inputs, private witness, proof system, transcript, and verification-key identity.
2. Implement proof generation and verification in a trusted component.
3. Bind issuer identity, predicate, verifier audience, fresh nonce, credential digest, and policy context to the proof.
4. Make the verifier reject malformed, unsupported, stale, or invalid proofs; never treat storage success as proof validity.
5. Independently reproduce proof verification using a second implementation.
6. Test adversarial witness, altered public-input, verifier-key, and proof-byte mutations.

## Vector 2 — Replay / session correlation — **PARTIAL BOOKKEEPING ONLY**

**Threat:** A previously presented proof is replayed, or records are correlated across sessions.

**Current source boundary:**

- `request_nonce` creates a random nonce entry and a link under a verifier-specific anchor.
- `submit_proof` receives only a caller-supplied `nonce_hash`; it does not receive the verifier DID or a reference to the issued nonce entry.
- The duplicate check uses `GetStrategy::Local` on the nonce-hash anchor and reports `nonce_was_fresh`.
- The coordinator does not cryptographically bind the proof to the nonce, enforce the stated five-minute expiry, verify that the nonce was issued by the intended verifier, or provide a global replay guarantee.
- The synthetic test varies deterministic nonce inputs and confirms placeholder hashes differ. It does not exercise the coordinator or real proof bytes.

**Required implementation:**

1. Bind proof submission to the exact verifier-issued nonce record and verifier identity.
2. Verify nonce freshness, expiry, audience, and single-use policy.
3. Define global / DHT consistency semantics under concurrent submissions and partitions.
4. Reject duplicate, unknown, expired, wrong-verifier, or malformed nonce references at the authoritative validation layer.
5. Make replay tests run through real multi-agent Holochain conductors, including concurrent race and restart cases.
6. Treat timestamps, issuer hashes, claim hashes, and other stable fields as potentially linkable unless a formal privacy analysis establishes otherwise.

A fresh nonce can make one commitment differ from another. That fact alone does not establish unlinkability.

## Vector 3 — Network metadata (IP correlation) — **NOT MITIGATED**

**Threat:** Primary and legal identities gossip from the same host or network and are correlated through timing, IP addresses, peer connections, or traffic volume.

No network-layer anonymity or metadata protection is established by the cross-DID coordinator. Separate agents, private relay designs, or onion routing would need their own threat model and deployment tests. These measures also do not replace proof verification.

## Vector 4 — Device compulsion / endpoint compromise — **NOT MITIGATED**

**Threat:** An attacker gains access to the device, keystore, session, wallet, or operator endpoint.

The current dual-DID architecture does not establish protection against a compromised endpoint or compelled disclosure. Key isolation, recovery, device-bound keys, backups, and duress mechanisms require separate designs and independent review. Do not market a duress-wipe or hardware-TEE capability until implemented and tested.

## Vector 5 — Location truth and jurisdiction inference — **NOT ESTABLISHED BY THE STARK ALONE**

The Winterfell-backed jurisdiction circuit can prove that a submitted numeric coordinate lies within a selected bounding box without revealing the coordinate. It cannot prove that the coordinate was truthfully measured or that the person presenting it was physically there.

The current location-attestation adapter boundary is limited:

- T0 is self-attested and accepts matching tier metadata.
- T1 checks the tier, optional public-key allowlist, and non-empty signature bytes. It does not cryptographically verify those bytes in this crate.
- The `PhoneGps.max_age_seconds` field is not enforced by `PhoneGps::verify`; callers must not assume freshness checking occurred.
- T2 civic, T3 hardware-TEE, and T4 notary implementations return `NotImplemented`.

A deployment that relies on location must validate the attester signature, key binding, time freshness, sensor/device attestation, jurisdiction dataset provenance and version, and policy tier independently. The bounding-box proof is a predicate proof about its private coordinate input—not a proof of provenance or personhood.

The jurisdiction proof also publishes the box ID, attestation tier, nonce hash, commitment, and two proof byte strings; the verifier supplies the attester-key hash, timestamp, box, and nonce to verify. The attester-key hash and other stable metadata can be correlation surfaces. A fresh nonce makes commitments distinct but does not, by itself, guarantee unlinkability.

## What the current tests prove — and do not prove

**The cross-DID tests prove only properties of their synthetic fixtures:** selected strings / hashes are absent from the constructed records, deterministic fields behave as coded, and placeholder values vary with seeded inputs.

They do **not** prove:

- a real ZKP is generated or cryptographically verified;
- an external verifier accepts only true predicates;
- the actual DHT / conductor implementation prevents replay;
- the actual system has cryptographic unlinkability;
- the legal DID cannot be correlated with the primary DID;
- network-layer anonymity or endpoint compromise resistance.

**The jurisdiction circuit tests** exercise a separate range-proof implementation when its Winterfell feature is enabled. They do not validate the location-attester signature or the real-world truth of the input coordinate.

## Promotion gates for privacy and assurance claims

| Gate | Required evidence | Claim allowed |
|---|---|---|
| G0 — synthetic model | Fixture and serialization checks only | "Synthetic proof-record model tests" |
| G1 — proof implementation | Real prover, verifier, pinned keys, mutation and negative controls | "The specified predicate proof verifies under the tested implementation" |
| G2 — protocol integration | Verifier-issued nonce binding, audience, expiry, DHT/concurrency tests | "Replay controls tested in the named Holochain configuration" |
| G3 — privacy analysis | Public-input review, correlation analysis, repeated-session evaluation, independent cryptanalysis | Only the scoped privacy property reviewed |
| G4 — deployment review | Network, endpoint, key lifecycle, operational and incident review | Only the reviewed deployment scope |

Do not describe the current cross-DID coordinator as ZKP-backed or unlinkable until it passes these gates.

## Review cadence

Update this document whenever proof generation/verification, nonce handling, key management, location attestation, or privacy-relevant public fields change.

**Last reconciled to public source:** 2 November 2026.
