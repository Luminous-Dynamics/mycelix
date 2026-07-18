# ADR-002: Working-alpha security and lifecycle contracts

Status: Accepted for implementation, 2026-07-13.

## Canonical product path

The supported alpha is the Leptos browser application plus the Holochain 0.6
DNA and deterministic Sweettests. Alternate application and backend surfaces
are frozen and excluded from the support claim.

## Runtime truth

Runtime mode is one of `Live`, `Demo`, or `Unavailable`. Demo mode is entered
only on the designated demo domain or through an explicit action. Failure to
connect, authenticate, decode, or call a zome produces `Unavailable`; it must
not substitute sample data.

## Key custody and recovery

Holochain/lair owns agent signing. The browser owns the X25519, ML-KEM, and
ML-DSA private material. A non-exportable Web Crypto AES key is structured-
cloned into IndexedDB and wraps the serialized hybrid seeds. Raw seeds exist
transiently in WASM memory during use and are cleared on a best-effort basis.
Recovery is same-browser-profile only. Clearing site data or losing the device
makes historical mail unrecoverable.

Non-exportability does not prevent same-origin malicious code from invoking a
key. CSP, dependency integrity, and safe content rendering are therefore part
of the cryptographic boundary.

Encryption keys have stable IDs and states: `active`, `retired`,
`revoked_compromised`, and `lost`. Only active keys accept new messages.
Retired and compromised keys may decrypt retained history, with an explicit
warning; lost keys cannot.

## Envelope authorization

`EncryptedEnvelopeV1` remains readable for compatibility and explicit
diagnostics. The implementation defaults new live messages to
`EncryptedEnvelopeV2HybridPqc`; release claims remain blocked until its full
implementation-evidence gate passes. V2 combines X25519 and ML-KEM-768 through
domain-separated HKDF, encrypts with AES-256-GCM, and requires both the
Holochain agent signature and an application-level ML-DSA-65 signature over
the same serializer-independent transcript. The transcript binds version,
suite, message ID, agents, key IDs, both KEM components, nonce, ciphertext,
immutable metadata, and timestamp. AES-GCM AAD binds the same identity,
routing, KEM, nonce, and metadata fields. Unknown versions and suites fail
before body interpretation.

There is no silent downgrade. Failure to construct V2 for a recipient that
advertises it fails the send. V1 sending is available only through an explicit
diagnostic/compatibility selection.

The signature asserts that the agent authorized publication of those exact
ciphertext and routing bytes. It does not prove that ciphertext matches
plaintext previously displayed by the browser.

## Enforcement boundary: DHT validation vs. client verification

This boundary is a real, disclosed constraint, not an oversight — state it
precisely rather than let "ML-DSA verification is mandatory" be read as
network-enforced.

**Enforced by every DHT validator** (`mail_messages_integrity`, host-independent
and unit-tested in `validate_email_v2_structure`, plus the host-dependent
checks in `validate_encrypted_email_v2`): `sender == action.author`; ciphertext
length bounds; ML-DSA-65 and Holochain-agent signature *lengths*; canonical
transcript encoding (unknown version/suite fails closed); the Holochain agent's
Ed25519 signature over that exact transcript; and `created_at` skew against the
action timestamp.

**Not enforced by any DHT validator**: the ML-DSA-65 signature's cryptographic
validity, and the sender/recipient hybrid key bundle's lifecycle state
(`active`/`retired`/`revoked_compromised`/`lost`) at send time. HDI/WASM
zomes cannot currently link the RustCrypto ML-DSA crate (the same dependency-
graph conflict documented in `PULSE_V2_CRYPTO_SPEC.md`), so a structurally
valid but cryptographically bogus ML-DSA signature — or a message referencing
a revoked/lost key ID — is accepted onto the DHT and gossiped like any other
valid entry.

**The recipient client is the actual enforcement point** for both: it calls
`verify_ml_dsa` against the sender's historical key bundle before decrypting
(`load_v2_inbox` in `apps/leptos/src/mail_context.rs`), and it refuses to
trust a sender bundle whose state is `revoked_compromised` or `lost`. The
Holochain agent signature still makes ciphertext authorship and integrity
fully network-verifiable — an attacker cannot forge *who* published given
bytes — but the *application-layer* post-quantum authentication claim is
honest only as "verified by the recipient before use," not "verified by
consensus." `phase0_v2_negative_paths` in the Sweettest suite exercises this
boundary directly: a garbage-but-correctly-sized ML-DSA signature is accepted
by `send_email_v2` and gossips normally, then fails `verify_ml_dsa` when the
recipient attempts to open it.

## Delivery semantics

Delivery evidence is message-content-independent and recipient-scoped by
`(MessageId, RecipientAgentPubKey)`. Sender commit derives `committed`;
recipient-signed receipts derive `observed`, `delivered`, and `read`.
Transitions are monotonic and receipt creation is idempotent. A failed network
attempt is local diagnostic information, not a terminal peer-visible state.

## Explicit exclusions

The alpha excludes attachments, portable recovery, offline sending,
distributed body search, SMTP, a PQ ratchet, post-compromise security, Sybil resistance,
calendar, chat, meetings, AI, mobile, and multi-account support. Plaintext
drafts may be stored on the same device; sending requires a live conductor.

The honest V2 claim is hybrid post-quantum message confidentiality and
application-level authentication. Holochain identities/actions remain
classical, and static recipient keys do not provide forward secrecy or
post-compromise recovery.
