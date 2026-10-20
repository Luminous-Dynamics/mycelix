# Health Vault Capability Handoff v0

**Status:** research/design contract; implementation not started by this document; not qualified.  
**Tracks:** [HEALTH-CAP-HANDOFF-001 / #5328](https://github.com/Luminous-Dynamics/mycelix/issues/5328), [caller-bound read / #5289](https://github.com/Luminous-Dynamics/mycelix/pull/5289), [consent enforcement / #5281](https://github.com/Luminous-Dynamics/mycelix/issues/5281).

## 1. Problem and paradigm shift

The caller-bound read slice correctly separates a grant from an Assigned Holochain capability, but its current `ConsentCapabilityOffer` returns the raw `CapSecret` to the local caller/UI. The remaining privacy condition is therefore a convention: the frontend must get the secret to the right person without logging, persisting, copying or misrouting it.

**Paradigm shift: the app layer should never receive plaintext capability material.** Create the capability and encrypt a versioned offer within the owner-side coordinator boundary. Only the encrypted envelope or an opaque offer reference crosses into UI/transport code. The recipient verifies and decrypts the offer locally, then creates its local `CapClaim`. A successful decrypt is not itself permission to read; the owner's live source-side grant, scope, expiry and revocation checks remain authoritative.

This follows Holochain's capability and cryptography model, which provides Assigned grants and key-store-backed encryption building blocks. Those primitives are not, by themselves, a complete delivery protocol:
- https://developer.holochain.org/build/capabilities/
- https://developer.holochain.org/build/cryptography-functions/

**Pinned-version research:** the Personal workspace declares `hdk = "0.6.0"`. The versioned [HDK 0.6.0 cryptography module](https://docs.rs/hdk/0.6.0/hdk/x_salsa20_poly1305/index.html) lists `create_x25519_keypair` and X25519 box encryption/decryption; it notes that encryption private keys remain in Lair and that box messages are repudiable, so box encryption alone is not third-party-verifiable proof of owner authorship. Add a separate authenticated owner-identity binding/signature to the canonical envelope. Important limit: the current `CapSecret` is still application data handled by the coordinator in order to create the grant; encrypting it inside the zome only prevents the plaintext secret from crossing the UI/transport boundary. It does **not** establish secret-free WASM memory or protect a compromised conductor. Keep secret lifetime bounded and do not claim hardware isolation.

## 2. Invariants

The implementation MUST preserve all of these:

1. **No plaintext secret across app boundary.** Raw `CapSecret` must not appear in the return value of browser-facing offer creation, app entries, public DHT values, link tags, signals, logs, analytics, crash reports, errors, receipts or CI output.
2. **Dedicated encryption keys.** Use dedicated X25519 keys managed by Holochain's keystore. Do not convert an Ed25519 identity/signing key to X25519 as a shortcut.
3. **Authenticated identity binding.** The recipient encryption public key must be bound to the intended `AgentPubKey`, with version, key ID, validity/rotation semantics and proof that the recipient controls the corresponding private key. A public key supplied in a request body is not automatically an authenticated binding.
4. **Authenticated sender.** A decryptable box alone is not enough to establish that the offer came from the named owner. Bind the owner's encryption key to the owner identity and authenticate the canonical offer bytes with a verifiable owner signature or an equivalently qualified binding protocol.
5. **Exact offer binding.** Bind the envelope to protocol version, unique opaque offer ID, owner identity, grantee identity, original grant creation action hash, recipient key ID, issue time, expiry and encrypted capability material. Keep the local capability-grant hash needed for owner-side revocation out of recipient-facing payload unless the recipient demonstrably requires it.
6. **Purpose separation.** This protocol transports one already-authorized, single-function Assigned capability. It does not broaden the granted function set, record scope, grant lifetime or identity of the assignee.
7. **Fail closed.** Unknown version, bad signature, recipient mismatch, wrong key, malformed ciphertext, missing grant, stale/unavailable authority, expiry or revocation must produce an explicit non-success with zero protected record bytes returned.
8. **Idempotence.** Re-delivering the same valid offer cannot create multiple effective claims or change the bound owner/grantee/grant.
9. **No retroactive promises.** Revocation blocks subsequent successful source-side calls after the revoke operation commits, subject to the in-flight race contract. It cannot recall a response already returned or plaintext copied by a legitimate grantee.
10. **Versioned migration.** A legacy manually transmitted offer is not silently treated as a v1 encrypted offer; old claims and legacy public health records keep their explicitly documented status.

## 3. Proposed envelope

The exact Rust type and serialization are an implementation decision after the pinned HDK API is verified. Its semantic fields must be equivalent to:

```text
HealthCapabilityOfferV1 {
    protocol_version,
    offer_id,                    // random opaque identifier; not a health/category label
    owner_agent,
    grantee_agent,
    grant_create_action_hash,     // original grant, never an update action
    recipient_encryption_key_id,
    sender_encryption_key_id,
    issued_at,
    expires_at,
    encrypted_payload,
    owner_authentication,         // signature over canonical domain-separated bytes
}
```

The encrypted payload contains only the minimum recipient-required fields, such as the capability secret and grant reference/expiry needed to create a matching local claim. The owner's local-only revocation handle remains owner-local.

Before implementation, freeze one canonical byte encoding for the signed subject. Do not sign ad hoc `JSON.stringify` output or rely on incidental map ordering. Add cross-implementation golden vectors if non-Rust clients will ever verify the envelope.

### Key enrollment and rotation

- Prefer an existing qualified, general-purpose encryption-key registry if one exists and meets the author/key-binding contract. Do not create a Health Vault-specific public identity index without measuring its metadata leakage.
- Otherwise, design a versioned key binding that is authored by the corresponding agent, references a dedicated X25519 public key, and includes a proof-of-possession challenge path. Specify how a grantee rotates or loses the key and how the owner learns the new binding.
- Reject duplicate active key IDs, expired bindings, key substitution, mismatched signing identity and unknown key versions.
- Keep private encryption key material in the Holochain keystore; never export it to app storage.
- Re-check the API signatures against the repository's actual Cargo/HDK lock before selecting functions. Do not infer that current online docs match the pinned version.

## 4. Delivery options to compare

| Option | Benefits | Costs / failure modes | Role |
|---|---|---|---|
| A. Encrypted direct handoff | Small scope; no durable mailbox; low metadata retention | Sender and recipient must be available; manual transfer can still be misrouted; no automatic retry | First implementation candidate, using ciphertext only |
| B. Durable encrypted envelope/mailbox | Allows later pickup and retry; supports acknowledgement/reconciliation | DHT/mailbox exposes routing/timing metadata; deletion is not reliable erasure; replay and spam controls required | Follow-up after threat/metadata review |
| C. Remote signal carrying the offer | Low interaction latency | Holochain remote signals are send-and-forget and are not queued for offline recipients; delivery result is not authorization evidence | **Not** a sole delivery or storage mechanism; at most notify “offer available” |

A DHT-backed envelope may store ciphertext only; this does not mean its existence, author, time, routing link or size are private. Never put the raw capability secret in the DHT, even if encryption is expected. Do not claim that expiry deletes immutable history.

## 5. State machine and race semantics

The observable app state should be monotonic and distinguish at least:

`Prepared -> Encrypted -> Delivered/Available -> Claimed -> {Expired, Revoked, Superseded}`

Failed delivery is a separate retryable state; it is not `Delivered`. An acknowledgement proves at most that a particular protocol step completed, not that a health record was read.

- Offer creation does not prove delivery.
- Ciphertext availability does not prove recipient decryption.
- Decryption does not prove a valid claim.
- A valid `CapClaim` does not prove current read authorization.
- A successful previous read does not authorize a later read.
- A durable receipt must not include the secret, ciphertext plaintext, or health payload.

**Revocation race:** after revoke commits, subsequent source-side calls must reject. A call that entered before the revoke commit may already be in flight; test and document whether it can return after the revoke response. Do not claim linearizable cancellation of in-flight disclosure unless the runtime actually enforces it.

## 6. Frozen adversarial corpus

A machine-readable, synthetic first version is committed alongside this note:
- [Corpus: `HEALTH_VAULT_CAPABILITY_HANDOFF_V1.json`](fixtures/HEALTH_VAULT_CAPABILITY_HANDOFF_V1.json)
- [JSON Schema: `HEALTH_VAULT_CAPABILITY_HANDOFF_V1.schema.json`](fixtures/HEALTH_VAULT_CAPABILITY_HANDOFF_V1.schema.json)

The corpus contains 25 stable IDs (`HCH-001` through `HCH-025`) across positive control, identity-binding, tampering, expiry, revocation, delivery retry, key rotation and secret-sink checks. Every fixture expects zero protected health-payload bytes from the tested handoff/read boundary. Denials have typed failure codes; offline authorization is an explicit `unavailable` result rather than success. The schema requires each of the 25 known fixture IDs exactly once. Fixture data is synthetic and contains no capability secrets or patient payloads.

**Current evidence ceiling:** the JSON and schema were parsed, and authoring-time structural assertions checked the count, ID uniqueness and zero-payload invariant. A standards-compliant JSON Schema validator and runtime harness have not yet been executed against these files. Corpus presence is not test execution or security qualification.

Give every fixture a stable ID and classify its status as authored, executed, failed or blocked; do not count an unexecuted test as coverage evidence.

| ID | Attack / boundary | Expected result |
|---|---|---|
| HCH-001 | Correct owner, recipient key binding and current offer | Recipient can ingest the claim |
| HCH-002 | Ciphertext copied to another agent key | Reject; zero health bytes |
| HCH-003 | Wrong/substituted recipient encryption public key | Binding check rejects |
| HCH-004 | Sender key is not bound to owner identity | Reject |
| HCH-005 | Offer metadata changed after signing | Signature/binding rejects |
| HCH-006 | Ciphertext altered or truncated | Authenticated decrypt fails |
| HCH-007 | Unknown protocol version | Reject; never downgrade |
| HCH-008 | Wrong grant action hash or update hash substituted | Reject |
| HCH-009 | Duplicate delivery of the same offer | Idempotent existing result |
| HCH-010 | Expired offer is newly redeemed | Reject |
| HCH-011 | Grant expires after offer generation | Owner read rejects |
| HCH-012 | Grant revoked before redemption/read | Reject |
| HCH-013 | Grant revoked concurrently with read | Documented race result; no post-commit newly admitted call |
| HCH-014 | Recipient encryption key rotated | Old offer follows explicit supersession policy |
| HCH-015 | Recipient key lost | No fallback to plaintext or old key; explicit reissue flow |
| HCH-016 | Owner is offline/partitioned | No cached authorization success |
| HCH-017 | Delivery interrupted, retried or reordered | No duplicate effective claim; outcome truthful |
| HCH-018 | Secret scanner probes app/DHT/log/telemetry surfaces | No plaintext secret matches |
| HCH-019 | Malformed envelope / invalid UTF-8 / oversized input | Bounded fail-closed error |
| HCH-020 | Same grantee uses the claim after owner revokes capability | Capability gate and current state reject |
| HCH-021 | Legacy unencrypted offer presented to v1 | Explicit legacy/unsupported outcome |
| HCH-022 | Sender identity binding rotates | Verify only under configured current/history policy |
| HCH-023 | Duplicate active recipient key IDs | Reject ambiguous selection |
| HCH-024 | Delivery succeeds but acknowledgement is lost | Retry/reconciliation does not claim duplicate issuance |
| HCH-025 | Secret placed in error string or debug formatting | Static/runtime leak probe fails the build |

Each negative fixture must assert both its rejection class and that no protected payload bytes were returned. The suite must run in a real multi-agent/conductor scenario in addition to unit tests of pure envelope validation.

## 7. Benchmark contract

No performance or industry-leading result has yet been measured. First freeze the corpus and establish a reproducible baseline.

### Hard security gates

- 0 false accepts across the frozen negative-control suite.
- 100% rejection of invalid identity binding, wrong recipient, tampered/unsupported envelope, expiry/revocation and unavailable authority.
- 0 plaintext capability secrets crossing UI/transport boundaries.
- 100% idempotent replay results.
- 100% root/workspace mirror parity for any mirrored source changes.
- 0 false claims of delivery when only a send attempt was made.

### Performance and operations

Run the same fixtures and machine/network/conductor manifest for: manual handoff baseline; encrypted direct handoff; and, if selected, durable encrypted envelope. Measure:

- offer creation and encryption/decryption latency;
- delivery completion, retry count and time-to-recovery;
- claim ingestion latency and duplicate-claim incidence;
- owner-side authorization p50/p95/p99;
- revoke-commit-to-subsequent-denial latency and concurrent-race outcomes;
- envelope bytes and metadata fields/bytes visible to each actor;
- behavior under owner offline, recipient offline, packet loss, restart and key rotation.

Candidate target after baseline measurement: p95 encrypted direct handoff below 5 seconds on a defined local-network testbed, excluding explicit human-confirmation time. This is an experimental target only, not an accepted result or a cross-network guarantee. Report failures and unavailable operations in the denominator; publish hardware, Holochain/HDK versions, test seed, scenario manifest and exact source SHA with every result.

### Industry comparison

Use Holochain Assigned-capability semantics as the access-control baseline; use the Holochain keystore-backed encryption APIs as the cryptographic building blocks. Compare to the current manual-secret handoff on the same harness, not to unrelated vendor marketing numbers.

HL7 FHIR R5 Consent is a useful interoperability reference because its policy representation includes actors, permitted/denied decisions, purpose-of-use and time-bounded provisions. SMART App Launch v2.2 provides a separate OAuth-based resource-scope and launch-context model. These standards solve adjacent, not identical, problems: a Holochain capability controls access to one zome function, while a consent policy describes who may do what, for what purpose, and during which period. The current Mycelix grant schema lacks purpose and field-level restrictions, so the adapter must not silently widen a FHIR policy into a broader Holochain grant. Unsupported rules should return an explicit non-representable/deny result. Before claiming interoperability, freeze a mapping corpus and test round-trip preservation for all supported fields plus loss detection for purpose, deny exceptions, security labels and fine-grained resource filters. Sources: https://hl7.org/fhir/R5/consent.html and https://build.fhir.org/ig/HL7/smart-app-launch/app-launch.html.

A superiority claim requires (1) the same adversarial fixtures, (2) no weaker security policy, (3) reproducible latency/size results, (4) explicit failure and metadata accounting, and (5) exact-head receipts. Until then, state targets and gaps, not “better than SOTA.”

## 8. Qualification gates

Before implementation approval:
- confirm available encryption/signing APIs and types against pinned HDK/Holochain versions;
- choose key-binding and delivery mode explicitly;
- resolve whether an existing generic identity/key registry is adequate;
- review public metadata of any proposed mailbox.

Before release:
- native tests, formatting, WASM build, strict Clippy, and real multi-agent conductor suite pass on one exact source SHA;
- root/workspace mirrors are byte-identical where required;
- independent secret-sink scan and metadata review complete;
- no required check is counted as PASS when queued, skipped, or blocked before reaching this code.

## 9. Nonclaims

This protocol does not make a compromised endpoint safe, prevent a legitimate grantee from copying disclosed plaintext, recall already disclosed records, prove clinical/POPIA compliance, hide all Holochain action metadata, or guarantee that a concurrent in-flight response is cancelled by revocation. Cryptography APIs are primitives; application security depends on key identity binding, protocol state and exact-head qualification.
