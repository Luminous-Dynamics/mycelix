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

**Update (2026-07-18): the ML-DSA gap described below is closed.** The
original claim — "HDI/WASM zomes cannot currently link the RustCrypto
ML-DSA crate" — was true only for `ml-dsa`'s *default* feature set, which
pulls in `getrandom` 0.4's browser-only Web Crypto backend for
signing/keygen. Signature **verification** needs no randomness at all
(`ml-dsa`'s `verifying.rs` has no `rand_core`/`getrandom` gating anywhere),
so `mail_messages_integrity` now depends on `ml-dsa` with
`default-features = false, features = ["alloc"]` — proven to link cleanly
in a real `cargo build --release --target wasm32-unknown-unknown`, not
just `cargo check` (which had falsely "passed" earlier by skipping the
link step entirely). See `verify_ml_dsa_v2` in that crate for the
implementation and its own doc comment for the full mechanism (a new
`sender_mldsa_bundle_hash` pointer field lets the validator fetch the
sender's real bundle via `must_get_valid_record`, then independently
re-derives and checks its content hash, authorship, and state before
verifying).

**Enforced by every DHT validator** (`mail_messages_integrity`, host-independent
and unit-tested in `validate_email_v2_structure`, plus the host-dependent
checks in `validate_encrypted_email_v2`): `sender == action.author`; ciphertext
length bounds; ML-DSA-65 and Holochain-agent signature *lengths*; canonical
transcript encoding (unknown version/suite fails closed); the Holochain agent's
Ed25519 signature over that exact transcript; `created_at` skew against the
action timestamp; **and now the ML-DSA-65 signature's cryptographic validity
against the sender's real historical key bundle, and that bundle's lifecycle
state (rejecting anything other than `Active`)**.

**Update (2026-07-19): the recipient-side gap described below is also
closed.** `EncryptedEmailV2` gained a `recipient_bundle_hash: ActionHash`
lookup pointer, mirroring `sender_mldsa_bundle_hash`'s contract exactly (not
itself trusted — the validator independently re-derives the fetched
bundle's content hash and checks it against the already-signed
`recipient_hybrid_key_id`, and its real Holochain authorship, before
trusting it). A new `verify_recipient_key_state` in `mail_messages_integrity`
rejects the message if the recipient's bundle isn't in the `Active` state at
send time — closing the asymmetry described below. Proven via 3 new native
mocked-HDI-host unit tests (accept-Active / reject-Revoked /
reject-wrong-author), same evidence tier as the sender-side fix, plus a real
`cargo build --release --target wasm32-unknown-unknown`. ~~Still not
enforced by any DHT validator: the *recipient* hybrid key bundle's lifecycle
state at send time (only the *sender's* state was checked, since only the
sender's key was what `verify_ml_dsa_v2` needed) — a message addressed to a
recipient whose key has since been revoked/lost was still accepted onto the
DHT; the recipient client remained the enforcement point for that specific
case.~~

**The recipient client remains an additional, independent enforcement
point**, not the sole one: it *also* calls `verify_ml_dsa` against the
sender's historical key bundle before decrypting (`load_v2_inbox` in
`apps/leptos/src/mail_context.rs`, via a separate by-agent-and-key-id
lookup), and refuses to trust a sender bundle whose state is
`revoked_compromised` or `lost`. This is deliberately redundant with the
DHT-level check, not a stand-in for it. `phase0_v2_negative_paths` in the
Sweettest suite exercises the DHT-level boundary directly: a
garbage-but-correctly-sized ML-DSA signature referencing the sender's real
published bundle is now **rejected by `send_email_v2` itself**, synchronously,
before the entry ever reaches the DHT — the inverse of what this test used
to prove.

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
