# Pulse V2 hybrid-PQC envelope specification

Status: working-alpha implementation contract, 2026-07-13. This is not an
independent cryptographic review or a production-security claim.

## Suite

The exact suite identifier is:

`x25519+ml-kem-768-hkdf-sha256-aes256gcm-agent-ed25519+ml-dsa-65`

New live messages use V2 only after the implementation-evidence gate passes.
V1 remains readable and may be selected explicitly for diagnostic isolation.
Failure to create or verify V2 never causes an automatic V1 retry.

The suite uses ML-KEM-768 as standardized by FIPS 203 and ML-DSA-65 as
standardized by FIPS 204. The current Rust implementation is feature-gated and
uses pre-stable RustCrypto crates, so the crate versions and vectors are part
of the evidence record and must not be silently upgraded.

## Integer and byte encoding

All unsigned integers are big-endian. `bytes(x)` is `u32(len(x)) || x`.
Optional 32-byte values are `0x00` when absent and `0x01 || value` when
present. Text is UTF-8 without normalization. Implementations reject lengths
that cannot fit in `u32` and reject unknown versions or suites before parsing
their bodies.

## Plaintext

One AES-GCM invocation protects the complete message payload. The plaintext is:

```text
"mycelix-pulse/plaintext/v2\0"
|| bytes(subject_utf8)
|| bytes(body_utf8)
|| bytes(content_type_utf8)
```

Subject and body are not encrypted separately. This avoids reusing one nonce
for multiple fields and makes authentication/decryption atomic.

## Hybrid key establishment

The recipient publishes one agent-bound bundle containing an X25519 public
key, an ML-KEM-768 encapsulation key, an ML-DSA-65 verifying key, key state,
and suite. Its key ID is:

```text
SHA-256(
  "mycelix-pulse/key-bundle/v2\0"
  || bytes(suite_utf8)
  || x25519_public
  || bytes(ml_kem_768_public)
  || bytes(ml_dsa_65_public)
)
```

The active Holochain agent signs the complete bundle transcript. A client must
verify that signature and the action author before trusting the PQ keys.

For each message the sender generates an ephemeral X25519 keypair and performs
ML-KEM-768 encapsulation. Let `ss_c` be the X25519 shared secret and `ss_pq` the
ML-KEM shared secret. Derive the 32-byte AEAD key as:

```text
salt = "mycelix-pulse/v2/hybrid-kem/salt\0"
       || x25519_ephemeral_public
       || ml_kem_ciphertext

ikm  = u32(len(ss_c)) || ss_c || u32(len(ss_pq)) || ss_pq

key  = HKDF-SHA256(
         salt = salt,
         ikm  = ikm,
         info = "mycelix-pulse/v2/hybrid-kem/aes-256-gcm\0",
         length = 32
       )
```

The sender generates a fresh uniformly random 12-byte nonce and performs
AES-256-GCM with the canonical AAD below. Secret buffers are zeroized on a
best-effort basis; JavaScript/WASM runtimes do not guarantee complete erasure.

## Canonical AAD and signature transcript

The AAD is the following concatenation:

```text
"mycelix-pulse/envelope/v2-hybrid-pqc\0"
|| u16(version = 2)
|| bytes(suite_utf8)
|| message_id[32]
|| bytes(sender_agent)
|| bytes(recipient_agent)
|| sender_ml_dsa_key_id[32]
|| recipient_hybrid_key_id[32]
|| x25519_ephemeral_public[32]
|| bytes(ml_kem_ciphertext)
|| nonce[12]
|| optional_32(in_reply_to_message_id)
|| optional_32(thread_id)
|| i64(created_at_micros)
```

The signature transcript is `AAD || bytes(aes_gcm_ciphertext_and_tag)`.
Signatures themselves are excluded. The active Holochain agent key and the
sender's advertised ML-DSA-65 key independently sign those exact bytes. A
recipient accepts a V2 message only if both signatures verify and the sender's
key bundle is agent-bound and valid for the message time.

The conductor signature asserts that the agent authorized publication of the
ciphertext transcript. Because encryption occurs in the client, it does not
independently prove which plaintext the UI displayed.

**Two enforcement layers, now both real (updated 2026-07-18).** DHT
validators (every node, `mail_messages_integrity`) check structure, the
Holochain agent's Ed25519 signature over the transcript, **and now the
ML-DSA-65 signature's cryptographic validity against the sender's real
published key bundle** (fetched via `must_get_valid_record` using a
`sender_mldsa_bundle_hash` pointer field, independently re-verified against
the already-signed `sender_mldsa_key_id` before being trusted — see
`verify_ml_dsa_v2` in `mail_messages_integrity`). Contrary to this doc's
earlier claim, HDI/WASM *can* link real ML-DSA verification — that was only
blocked by `ml-dsa`'s default (signing-capable) feature set, not by
verification itself, which needs no randomness. A message with a
garbage-but-correctly-sized ML-DSA signature is now rejected by
`send_email_v2` itself, before the entry ever reaches the DHT. **Updated
2026-07-19**: the recipient's key-state is now also DHT-validator-enforced,
not just client-checked — a new `recipient_bundle_hash` pointer field (same
independently-re-verified-before-trust contract as the sender's) lets
`verify_recipient_key_state` reject a message addressed to a recipient
whose bundle isn't `Active` at send time. The **recipient client**
independently re-verifies ML-DSA and both parties' key states before
trusting decrypted content, deliberately redundant with the DHT-level
check, not a stand-in for it. See ADR-002 for
the full enforcement-boundary statement and the Sweettest that exercises it
(`phase0_v2_negative_paths`, case C now proves rejection, not acceptance).

## Fixed sizes and rejection rules

- X25519 public keys: 32 bytes.
- ML-KEM-768 encapsulation key: 1184 bytes.
- ML-KEM-768 ciphertext: 1088 bytes.
- ML-DSA-65 verifying key: 1952 bytes.
- ML-DSA-65 signature: 3309 bytes.
- AES-GCM nonce: 12 bytes; authentication tag: 16 bytes.

Unknown versions/suites, malformed component lengths, and invalid key-bundle
bindings are hard failures **at the DHT validator**. Non-active recipient
keys for new sends, revoked-compromised sender keys, and ML-DSA signature
failure are hard failures **at the recipient client** (see the enforcement-
boundary note above) — the network layer does not reject these. AES-GCM
failure is a hard failure wherever decryption is attempted. None of these
trigger downgrade or partial plaintext display.

## Key custody and claim boundary

The alpha recovery model is same-device only. A non-exportable Web Crypto AES
wrapping key is stored by structured clone in IndexedDB. Serialized ML-KEM and
ML-DSA seeds are persisted only as AES-GCM-wrapped ciphertext and reconstructed
transiently for WASM operations. Pulse provides no raw-key download or portable
restore in this milestone.

This design provides hybrid classical/post-quantum message confidentiality and
application-level ML-DSA authentication when all gates pass. It does not claim
a post-quantum Holochain substrate, a PQ ratchet, forward secrecy against later
static endpoint compromise, or post-compromise recovery.

## Required evidence before V2 becomes the live default

- NIST known-answer coverage for the underlying ML-KEM and ML-DSA versions.
- Identical protocol vectors in native and `wasm32-unknown-unknown` builds.
- Tests showing that changing either shared-secret contribution changes the
  derived AEAD key.
- Tests requiring both the agent and ML-DSA signatures.
- Negative tests for transcript mutation, malformed encapsulation, substituted
  bundles, stale/revoked keys, unknown versions/suites, and downgrade attempts.
- V1 readability and an end-to-end V2 lifecycle across separate conductors,
  including restart recovery.
