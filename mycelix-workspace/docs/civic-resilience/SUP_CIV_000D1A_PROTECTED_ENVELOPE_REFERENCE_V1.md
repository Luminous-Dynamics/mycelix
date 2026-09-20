# SUP-CIV-000D1A — Protected-Envelope Semantic Reference Model v1

Status: independent semantic reference model only. Child of #2307 and stacked on the exact SUP-CIV-000D0 subject from draft PR #2314.

This tranche executes **no cryptographic primitive** and selects **no storage topology**. It freezes key-lifecycle decision classes before product Rust can define them by implementation convenience.

## Parent boundary

Exact parent subject:

`f4f48ef45bbaff7ae9e7668fc1820fdfadd1375f`

That parent is itself subject to its dedicated qualifier. A green child cannot repair or substitute for an unqualified parent.

## Why another reference layer

The reuse census establishes that Xenia session security, wire AEAD, hardened local files, Holochain capabilities/private entries, and generic crypto availability are useful primitives but are not themselves a durable multi-recipient protected-case theorem.

The next boundary therefore needs an implementation-independent state model before selecting an AEAD/KEM/wrapping implementation.

The reference model deliberately uses opaque 32-byte fixture commitments. Its SHA-256 digest is only a deterministic **reference-model lineage witness**:

```text
ReferenceModelDigest != RuntimeEnvelopeId
```

A later codec/crypto tranche must freeze normative bytes and cryptographic identity separately.

## Three independent counters

The model separates:

```text
payload_version
!= envelope_revision
!= key_epoch
```

### `payload_version`

Changes only when the protected plaintext payload changes semantically.

### `envelope_revision`

Changes for every admitted envelope transition, including recipient/key metadata changes that leave the protected payload unchanged.

### `key_epoch`

Changes only when a fresh DEK is required by the strong lifecycle profile.

This prevents `version++` from hiding whether payload content, recipient metadata, or the actual encryption key changed.

## Recipient identity

A recipient row contains separate opaque references for:

```text
recipient subject
recipient key identity
key profile
wrap profile
wrapped-DEK commitment
wrap-context commitment
```

V1 requires unique recipient subjects and unique active recipient key identities. A group recipient, if later needed, should be represented as an explicit group subject/profile rather than by silently sharing one individual key across several subject rows.

```text
RecipientListed != RecipientCurrentlyAuthorized
CryptographicRecipient != LegitimatePurpose
```

Current authorization/accountability stays outside this envelope theorem.

## DEK generation boundary

The strong V1 semantic profile accepts only:

```text
random-per-payload-dek-v1
```

as the DEK generation class and binds an opaque generation-receipt reference.

The model can prove the declared generation class/lineage changed where required; it cannot prove entropy quality:

```text
GenerationReceipt != EntropyProof
SessionKey != DurableCaseDEK
```

## Closed transition algebra

### CreateEnvelope

Initial counters are exactly:

```text
payload_version = 1
envelope_revision = 1
key_epoch = 1
```

At least one canonical recipient is required and no previous-envelope reference exists.

### AddRecipientCurrentEpoch

Adds exactly one recipient while keeping:

- payload version;
- key epoch;
- payload ciphertext commitment;
- payload AAD commitment;
- DEK-generation receipt;
- every existing recipient binding

unchanged.

This operation intentionally carries a strong warning:

```text
AddRecipientCurrentEpoch != FutureOnlyAccess
```

A recipient who receives the current epoch's DEK may be able to decrypt any available ciphertext protected by that same DEK/epoch. The operation must never be described as future-only admission.

### AddRecipientFutureOnly

Adds exactly one recipient **and** requires:

- `key_epoch + 1`;
- fresh DEK-generation receipt;
- changed ciphertext commitment;
- changed AAD commitment;
- rewrapped DEK commitment for every existing recipient.

This creates a new forward epoch for the current payload without asserting old ciphertext became unknowable.

### RemoveRecipientForwardExclusion

Removes exactly one recipient and requires the same full DEK/ciphertext rotation as a future-only addition.

```text
ForwardExclusion != RetroactiveUnread
```

A removed recipient may retain any plaintext, ciphertext keys, exports, screenshots, backups, or knowledge previously obtained.

### RotateAfterCompromise

Keeps the recipient subject set fixed but requires a fresh DEK/key epoch, new ciphertext/AAD commitment, and new wrapped-DEK commitment for every recipient.

This establishes only a new forward cryptographic epoch.

### RewrapRecipientKey

Changes exactly one recipient's key identity/wrap/context while keeping the current DEK, key epoch, ciphertext and payload AAD unchanged.

Therefore:

```text
RewrapRecipientKey != CompromiseRecovery
RewrapSameDEK != RevokeOldKnowledge
```

If the old recipient key or DEK may have been compromised, use a fresh-key-epoch operation instead.

### ReplacePayload

Requires:

- `payload_version + 1`;
- `envelope_revision + 1`;
- `key_epoch + 1`;
- fresh DEK-generation receipt;
- changed ciphertext/AAD;
- fresh wrapped-DEK commitment for every recipient;
- unchanged recipient subject set.

This is the conservative V1 profile: protected payload revisions do not silently reuse the previous payload DEK.

### RetireEnvelope

Retirement is a separate immutable disposition receipt referencing one exact envelope. It does not mutate history and does not claim erasure.

```text
RetirementReceipt != RetroactiveErasure
Retired != PreviouslyDisclosedPlaintextUnknown
```

A retired envelope is refused as a base for later transitions by this reference profile.

## Lineage rules

Every non-create transition must bind:

```text
envelope_revision = previous + 1
previous_envelope_ref = exact reference-model digest of previous state
```

No `latest timestamp wins`, skipped revisions, arbitrary `SetState`, or parent substitution exists in the reference algebra.

The fixture digest is non-normative and exists only to make the independent oracle deterministic.

## Thirty-case hostile corpus

The executable oracle covers 30 exact cases, including:

- valid create;
- session-derived DEK refusal;
- duplicate recipient subject refusal;
- duplicate active recipient key refusal;
- non-canonical ordering refusal;
- invalid initial counters;
- current-epoch recipient addition;
- current-epoch mutation/DEK-change refusals;
- future-only recipient addition;
- future-only stale-epoch/stale-wrap refusals;
- forward-exclusion removal;
- removal without epoch/cipher/wrap rotation;
- removal of the final active recipient;
- compromise rotation;
- recipient-set mutation during compromise rotation;
- key rewrap;
- no-op/same-key rewrap;
- DEK change hidden inside rewrap;
- payload replacement;
- payload-version skipping;
- previous-ref substitution;
- envelope-revision skipping;
- retirement;
- wrong-envelope retirement;
- post-retirement transition;
- unknown generic state-setting operation.

The checked-in manifest closes the expected disposition and refusal reason for all 30 cases.

## Load-bearing non-equivalences

```text
PayloadVersion != EnvelopeRevision
EnvelopeRevision != KeyEpoch
AddRecipientCurrentEpoch != FutureOnlyAccess
RewrapRecipientKey != CompromiseRecovery
KeyRotation != KnowledgeRevocation
RecipientListed != RecipientCurrentlyAuthorized
SemanticAdmission != CryptographicValidity
ReferenceModelDigest != RuntimeEnvelopeId
RetirementReceipt != RetroactiveErasure
```

## Evidence order after this tranche

```text
000D1A  semantic lifecycle oracle                         <- this tranche
000D1B  normative codec/transcript + independent vectors
000D1C  exact primitive profiles + Rust crypto implementation
000D2   storage topology/profile
000D3   envelope + storage + accountable-access composition
```

000D1B/1C must not import this Python model into production Rust. Product code should independently implement the qualified decision/transcript contract and then be compared against frozen vectors.

## Standards alignment without authority laundering

The design intentionally keeps KEM/session establishment distinct from stored-data key lifecycle. FIPS 203 defines ML-KEM as a mechanism for establishing a shared secret; it does not by itself define durable multi-recipient case-key policy. NIST SP 800-57 separately treats symmetric data-encryption keys and their backup/archive/wrapping lifecycle. RFC 9180 HPKE is useful reference material for KEM/KDF/AEAD composition to a recipient, but this tranche does not claim HPKE supplies the multi-recipient lifecycle, authorization, retention, or revocation theorem required here.

## Claim ceiling

A PASS establishes only the exact semantic state/transition decisions exercised by this independent standard-library oracle and its closed manifest.

It does **not** establish:

- cryptographic correctness;
- entropy quality;
- normative wire/storage encoding;
- recipient authorization or legitimate purpose;
- storage availability/replication;
- metadata privacy;
- legal compliance;
- anonymity;
- retroactive revocation of knowledge;
- municipal authority;
- Johannesburg readiness;
- deployment readiness.
