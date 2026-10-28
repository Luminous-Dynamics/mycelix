# NCF-1 PostgreSQL ↔ Holochain integration contract

**Status:** architecture and executable SQL-side conformance target; no new Holochain zome implementation or Sweettest result is claimed yet.

## Existing project fit

The repository already has `mycelix-finance`, a Holochain hApp using the workspace-pinned HDK 0.6.1 / HDI 0.7.1 line. Its finance README describes MYCEL, SAP, and TEND and cross-community TEND clearing. The existing test guide calls for Rust Sweettest integration scenarios against a running conductor. The nested-currency crate in this PR explicitly says it is a standalone reference model rather than a zome or production ledger. The PostgreSQL profile should therefore integrate with the existing hApp, not replace Holochain with SQL and not pretend that the standalone model was already production infrastructure.

For current Holochain 0.6-era integration tests, prefer the Rust-based Sweettest harness. The official upgrade guide says Tryorama was replaced in the 0.6.1 development environment, while the old Tryorama documentation marks it deprecated. See [Holochain 0.5→0.6 upgrade guide](https://developer.holochain.org/resources/upgrade/upgrade-holochain-0.6/) and [current validation guidance](https://developer.holochain.org/build/validation/).

## Responsibility split

### PostgreSQL shared-coordinator profile

PostgreSQL is the transactional coordinator for operations using the same database authority domain. One transaction owns:

1. immutable request-ID / verifier-evidence binding;
2. canonical-work deduplication;
3. agreement, community/instrument cap, and reservation-slice lock/check;
4. cap and slice counter mutation;
5. accepted canonical obligation and durable receipt;
6. a Holochain transactional-outbox row.

The app receives an accepted receipt only after the SQL statement and surrounding transaction commit. The outbox worker is separate from request admission.

### Holochain replicated-evidence profile

Holochain is the signed source-chain and DHT evidence path. It stores authored claims, their links, policy/manifest references, and validation evidence. Integrity-zome validation is deterministic and checks what an author was permitted to assert at that point in their source chain. The DHT distributes operations to authorities for validation and storage; validation receipts help a conductor assess DHT health. They are not a cross-agent serializable database commit, a BFT quorum certificate, or legal settlement finality. See [DHT operations](https://developer.holochain.org/build/dht-operations/) and [validation receipts](https://developer.holochain.org/build/validation-receipts/).

The Holochain WASM zome should not connect directly to PostgreSQL. An external Rust service / conductor client is the bridge: it calls the PostgreSQL transaction API, leases committed outbox rows, submits signed content through the Holochain client, and records the resulting action hash and receipt progress. The service must not promote a missing acknowledgement to a failure or successful commit without idempotent reconciliation.

## Authority attestation and trust model

A PostgreSQL outbox row, JSON receipt, or SHA-256 digest is not a public-key signature and is not independent proof that PostgreSQL committed. The first production bridge must make that trust boundary explicit instead of treating a payload hash as authentication.

The proposed policy is a **registered monetary-authority agent** in Holochain. A narrowly privileged Rust dispatcher uses that agent's conductor-keystore identity to author the claim only after it has read a committed outbox row. The Holochain action signature binds the entry to the author. The integrity zome checks the action author against an immutable, governance-authorized AuthorityManifest that is addressable from the claim. This is an authority attestation of the coordinator receipt, not a zero-knowledge proof of a PostgreSQL transaction. The private signing identity must not be held by untrusted caller processes or embedded in SQL.

AuthorityManifest minimum fields:
- schema version, authority ID, authorized AgentPubKey, policy epoch, manifest digest;
- allowed community(s), instrument/profile family/revision range, permitted operation types, and validity interval;
- governance authorization action hash and previous-manifest/fencing reference; and
- explicit key activation and revocation semantics.

The claim must carry the manifest action hash/digest and the exact authority epoch it relies on. Validation must check (a) the Holochain action author equals the manifest's authorized AgentPubKey, (b) the operation and community/instrument/direction are inside the manifest scope, (c) the epoch/validity rules are satisfied, and (d) the referenced manifest is itself governance-authorized. If a dependency is temporarily absent from the validator's data, return the correct unresolved-dependency outcome rather than permanently rejecting a valid claim on the basis of an incomplete DHT view.

For source-chain determinism, do not resolve "the latest key" through an unconstrained live lookup during validation. Pin each claim to an immutable authority-manifest action. Revocation is a new manifest/epoch transition; the contract must specify whether it blocks newly authored claims only or also makes previously committed claims disputable. Settlement checks should require the currently applicable policy epoch when policy calls for it. The manifest chain and its governance authorization must have a deterministic genesis/trust root.

The canonical PostgreSQL receipt digest remains a content-binding checksum, not signer identity. Its immutable fields should include stable event ID, obligation/request IDs, canonical-work digest, principal/community, instrument/profile revision, integer amount string, gross direction/agreement, reservation/slice/shard/epoch, verifier version/evidence digest, and SQL receipt digest. The Holochain claim action's author signature attests to the envelope under the registered authority key. A malicious or compromised authorized authority can still lie about SQL state; mitigating that requires tighter key custody, independent audits/witnesses, or a separately designed threshold authority policy.

## Canonical cross-language claim digest

The claim's immutable content binding uses the versioned `NCF-AUTHORITY-CLAIM-V1` profile and the golden fixture `ncf-authority-claim-v1-vectors.json`. Producers and Holochain validators must use the same bytes, not independently serialize JSON and hope that map ordering, numeric representation, Unicode normalization, or JSONB formatting happen to agree.

Encoding is normative:

1. Prefix the preimage with the exact domain separator bytes `NCF-AUTHORITY-CLAIM-V1\0` (the characters `\0` here denote one zero byte).
2. Encode exactly these 19 fields in this exact order: `schema_version`, `outbox_event_id`, `obligation_id`, `request_id`, `canonical_work_digest`, `principal_id`, `community_id`, `instrument_id`, `profile_revision`, `amount_units`, `agreement_id`, `direction`, `reservation_id`, `shard_id`, `epoch`, `verifier_version`, `verifier_receipt_digest`, `sql_receipt_digest`, `authority_manifest_digest`.
3. Every field value is a NUL-free ASCII string. Before each field value, append its length as a 4-byte unsigned big-endian integer; then append the exact field bytes. Do not include field names in the framed payload because the order is fixed by the profile version.
4. UUID fields use canonical lowercase hyphenated text. Decimal integer fields use base 10 without leading zeros; amount and epoch are strings, not JSON numbers, and arbitrary-precision integer values must remain exact. Digest fields are 64 lowercase hexadecimal ASCII characters. Direction is exactly `a_to_b` or `b_to_a`.
5. Compute SHA-256 over the complete domain-prefixed framed byte sequence. Any field order, type, spelling, or value difference changes the digest.

Two golden vectors cover a basic claim and an amount `9007199254740993`, which exceeds JavaScript's exact integer range. Their expected digests and framing lengths are frozen in the JSON fixture. The separate Python reference test currently passes 11 local tests, but the checked-in test source has not yet been proven byte-identical to the locally tested copy; therefore this is reference-model evidence, not exact-source qualification. The same vectors must be consumed by a Rust implementation and a Holochain integrity-zome test before cross-language conformance can be claimed.

**Digest versus signature:** this digest is content binding only. It does not prove SQL commit, signer identity, or real-world service delivery. The `sql_receipt_digest` field is an opaque receipt identifier inside the envelope until a separate SQL-receipt canonicalization profile is specified; do not recompute it using PostgreSQL JSONB text in another language. The actual Holochain action signature authenticates the author, and deterministic integrity validation must additionally verify that author against the pinned governance-authorized AuthorityManifest and its scope/epoch. A compromised authorized authority can still issue false claims; this profile does not eliminate key-custody risk.

## Proposed entry types

These are the first reviewable data contracts; they are not yet implemented entry definitions.

### `ReservationManifest`

Fields: schema version, issuer community, exact instrument ID and profile revision, agreement ID, direction, shard ID, epoch, reserved gross-exposure units as a canonical decimal string, designated home/writer agent key, predecessor/fencing reference, creation authority, and manifest digest.

Integrity validation checks the authorized issuer/governance record, canonical instrument profile, direction-to-community mapping, nonnegative epoch, positive finite reserved units, signature/capability, and predecessor/fencing constraints. A stale epoch or unproven transfer of a slice cannot grant new spend rights.

### `ObligationClaim`

Fields: schema version, stable outbox/event ID, canonical obligation ID, request ID, canonical-work digest, principal, community, instrument ID, profile revision, exact amount string, direction, agreement, reservation ID, shard ID, epoch, verifier version/evidence digest, PostgreSQL canonical-receipt digest, AuthorityManifest action hash/digest, and authority policy epoch.

The database transaction and the independently verified claim must agree on every immutable field. Amounts are encoded as integer decimal strings, never floating point. The canonical profile and manifest references are addressable hashes, not mutable ticker names.

### `SettlementClaim`

Fields: stable settlement ID, obligation ID and canonical receipt digest, exact settlement amount and instrument, direction, settlement evidence digest, and the predecessor claim/action hash. Validation rejects a settlement whose obligation is absent, whose instrument/amount/direction differs, or whose settlement ID is rebound. Missing addressable dependencies should be returned as unresolved/retryable, not permanently rejected simply because data has not arrived yet.

## Idempotency and publication protocol

1. PostgreSQL atomically creates the canonical obligation, cap/slice accounting, request binding, receipt, and outbox record.
2. A dispatcher leases outbox rows using `FOR UPDATE SKIP LOCKED` and records an attempt.
3. The dispatcher authors the claim using the exact stable event ID and canonical content. The Holochain coordinator checks its authored source-chain index before retrying the same event.
4. After the call returns, the dispatcher records the action hash. If it crashes after the Holochain call but before this update, it retries/reconciles using the same event ID and canonical digest.
5. The DHT validation-receipt count and any unresolved dependency state are recorded separately. A policy may require receipt thresholds before exposing a claim as replicated, but this is not global settlement finality.
6. Settlement is a distinct transition with its own idempotency key and source-chain evidence. It must not be inferred merely from a PostgreSQL row update or from the presence of an authored action.

The outbox is an **at-least-once delivery mechanism**, not a cross-system two-phase commit. If a call may have succeeded but its acknowledgement was lost, query/retry with the same stable ID. Never generate a new monetary obligation ID merely because a network acknowledgement timed out.

## Crucial limitation: offline global uniqueness

A PostgreSQL `UNIQUE(canonical_work_digest)` constraint guarantees uniqueness only among transactions using the same PostgreSQL database. It cannot coordinate separate disconnected PostgreSQL instances. Holochain's DHT can distribute and validate evidence, but a DHT lookup is not a synchronous global unique-index lock; identical entries can be authored by different agents with distinct source-chain actions.

The offline profile therefore needs an explicit policy. This proposal selects **deterministic home-shard assignment** for a canonical work identity, with the home shard and designated author bound into the signed reservation manifest. A disconnected non-home shard must not accept the same work as a fresh obligation. If the home shard is unreachable, safety wins over offline liveness for that work. Once connectivity returns, agents can reconcile deterministic event identities and receipt history. Any alternative (threshold quorum, synchronous online authority, or later dispute/reversal) must be separately specified and benchmarked; do not imply global anti-duplicate guarantees from eventual DHT gossip alone.

## Sweettest scenarios required before qualification

Use Rust Sweettest against an actual built DNA and 3–5 conductors/agents. Run one deterministic scenario at a time initially, as the current repository testing guide notes conductor tests may need `--test-threads=1`.

| ID | Scenario | Required result |
|---|---|---|
| HC-NCF-001 | Authorized reservation manifest, valid obligation claim | Correct action authored; DHT operations validate |
| HC-NCF-002 | Wrong issuer / ungranted capability | Claim rejected by integrity validation |
| HC-NCF-003 | Changed instrument, denomination revision, amount, direction, or manifest digest | Rejected, no authorized economic mutation |
| HC-NCF-004 | Old epoch or stale reservation owner attempts a new claim | Rejected; no implicit capacity reclamation |
| HC-NCF-005 | Same outbox event retried after simulated lost acknowledgement | Same canonical event/receipt is reconciled; no second SQL debit |
| HC-NCF-006 | Dispatcher crashes after action authored but before PostgreSQL marks outbox published | Retry reuses stable event identity and converges |
| HC-NCF-007 | Three agents concurrently present same canonical-work digest | Home-shard routing permits only the designated owner to author the authoritative claim; non-home submissions reject |
| HC-NCF-008 | Network delay / dependency not yet replicated | Validation returns unresolved/retryable where required, not a false permanent invalidity |
| HC-NCF-009 | Invalid settlement, wrong obligation, duplicate settlement ID, amount/direction mismatch | Reject without decrementing the SQL obligation twice |
| HC-NCF-010 | PostgreSQL has committed but Holochain conductor is unavailable | Outbox remains pending/retry; SQL receipt remains durably queryable; system does not claim DHT replication |
| HC-NCF-011 | PostgreSQL restart with pending/published outbox rows | Deterministic recovery; reconcile action hashes before generating new events |
| HC-NCF-012 | DHT validation receipt count below policy threshold | Record authored/unreplicated state separately; do not label globally settled |
| HC-NCF-013 | Unauthorized agent copies a valid SQL receipt payload | Integrity validation rejects because the action author is not the manifest-authorized monetary authority |
| HC-NCF-014 | Authorized key submits a claim outside manifest community/instrument/direction scope | Integrity validation rejects without treating the SQL receipt hash as authority |
| HC-NCF-015 | Claim references an absent authority manifest during DHT propagation | Validation returns unresolved/retryable until the pinned governance-authorized manifest is available |
| HC-NCF-016 | Key rotates or authority is revoked at a later policy epoch | New claims must pin the currently applicable manifest; prior claims follow the explicitly specified grandfather/dispute policy |
| HC-NCF-017 | Same event ID is retried with altered amount, instrument, evidence digest, or manifest reference | Stable event binding rejects the conflict; no new SQL debit or accepted Holochain authority claim is created |

Required test receipts bind the Mycelix PR head, DNA hash, coordinator/integrity zome WASM hashes, Sweettest harness source, agent count, conductor version, operation history, and result hashes. All 17 scenarios are plans, not current pass evidence.

## Required state machine

Use separate names rather than one overloaded `accepted` flag:

- `sql_committed`
- `holochain_authored`
- `dht_validation_pending`
- `dht_receipts_threshold_met`
- `settlement_evidence_pending`
- `settled`
- `disputed` / `reversed`

Each transition requires a stable event ID and evidence binding. A terminal settlement state is based on the settlement policy, not merely on an outbox delivery status.

## Qualification ceiling

The SQL profile in this branch has not yet run on PostgreSQL in this editing environment; the new workflow is intended to exercise it on PostgreSQL 18.6. No new Holochain zome or Sweettest execution is claimed. The offline home-shard rule is a design proposal that needs adversarial validation, and neither PostgreSQL nor Holochain alone guarantees real-world service delivery, external payment finality, legal tender status, secure key custody, or multi-community global uniqueness under an unrestricted partition.
