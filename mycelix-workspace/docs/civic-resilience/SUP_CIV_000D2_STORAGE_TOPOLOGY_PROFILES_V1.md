# SUP-CIV-000D2 — Protected Storage Topology Profiles v1

Status: semantic storage-topology selection contract only. Exact parent: SUP-CIV-000D0 `f4f48ef45bbaff7ae9e7668fc1820fdfadd1375f`.

This tranche deliberately stays independent from the cryptographic-envelope line. A storage topology may be selected only for properties it actually owns; encryption, authorization, release, legal compliance, and municipal legitimacy remain separate theorems.

## Core law

```text
CryptographicEnvelopeValid != StorageTopologyQualified
StorageAvailable != RecipientAuthorized
StoragePrivateByTopology != MetadataPrivate
```

No profile is a universal winner.

## Closed profiles

### 1. `author-private-source-chain-v1`

Use a Holochain private entry when the datum is genuinely author-controlled and author-local.

Strengths:
- plaintext content is not replicated to the shared DHT;
- minimal shared plaintext footprint;
- can compose capability-mediated remote reads.

Limitations:
- durable multi-party availability is not established;
- author/device availability remains load-bearing;
- action metadata remains public under Holochain semantics;
- copying/handoff creates a new theorem.

```text
PrivateEntry != MultiPartyProtectedDatabase
```

### 2. `encrypted-public-dht-v1`

Store qualified protected-envelope ciphertext as public DHT content.

Strengths:
- distributed ciphertext replication;
- offline/multi-reader availability can be strong;
- exact ciphertext artifacts can be content-addressed and evidence-bound.

Limitations:
- ciphertext/action timing/size/link graph remain observable according to representation;
- recipient/key lifecycle must already be qualified;
- no global-erasure claim;
- key revocation cannot erase knowledge already disclosed.

```text
EncryptedDhtCiphertext != MetadataPrivacy
```

### 3. `restricted-dna-encrypted-v1`

Use a restricted/clone DNA as a participant-bounded case space, with encryption for sensitive Civic payloads.

Strengths:
- isolates case traffic from the broader public DNA;
- preserves distributed replication within the admitted group;
- can map naturally to institution/team/case-space boundaries.

Limitations:
- network membership is not per-record need-to-know;
- removal does not retroactively unread data;
- participant-visible metadata still needs its own profile.

```text
RestrictedDnaMembership != PerRecordLeastPrivilege
```

### 4. `external-protected-store-v1`

Keep protected case payloads in an institutional database/object store/KMS-backed service and place only public-safe references/evidence commitments in Mycelix.

Strengths:
- can compose mature database, HSM/KMS, backup, retention and institutional access controls;
- easier integration with incumbent municipal/health/social-service systems;
- can support strong operational availability without putting sensitive plaintext on a shared DHT.

Limitations:
- creates an external trust/availability boundary;
- external database state is not Mycelix truth;
- adapter authorization, evidence cuts and audit integrity need qualification;
- offline/decentralized operation is deployment-specific.

```text
ExternalProtectedStore != MycelixVerifiedTruth
```

### 5. `civic-hybrid-institutional-pilot-v1`

Initial sensitive Civic pilot candidate:

```text
public/coarsened operational envelope on Mycelix
+ protected payload in external protected store
+ opaque evidence/provenance refs
+ reciprocal access-accountability
+ bounded encrypted local offline cache where explicitly permitted
```

The public plane must never contain the protected narrative merely because the protected backend is unavailable.

This is a **candidate profile**, not a Johannesburg deployment approval or legal-compliance claim.

```text
JohannesburgCandidateProfile != JohannesburgDeploymentReady
```

## Fail-closed selector

The selector must refuse rather than weaken protection:

```text
protected store unavailable
    != permission to publish plaintext

profile unknown
    -> refuse

required metadata confidentiality exceeds topology
    -> refuse or choose another qualified profile

required strong erasure/retention semantics not owned
    -> refuse or choose another qualified profile
```

## Selection rules

### Sensitive free text / precise geography

Resident narratives, exact addresses, live responder/vulnerable-person location, contact details, special-category information, allegations, credentials/tokens and detailed diagnostics never enter public plaintext DHT storage by default.

### Strong retention / erasure requirements

Prefer a topology that explicitly owns the required store-level controls. Public DHT ciphertext may still persist indefinitely; crypto-key destruction is not represented as guaranteed global erasure.

### Decentralized offline multi-reader casework

Use `encrypted-public-dht-v1` or `restricted-dna-encrypted-v1` only after the required protected-envelope/key-lifecycle theorem qualifies.

### Single-author private state

`author-private-source-chain-v1` may be appropriate when multi-party durable access is not required.

### Institutional interoperability

`external-protected-store-v1` or the Civic hybrid profile may be appropriate where existing institutional KMS/database/retention controls are part of the deployment boundary.

### Metadata unlinkability

No DHT topology in this contract independently establishes unlinkability. If metadata confidentiality is load-bearing and cannot be adequately coarsened, select a different topology or refuse.

## Offline cache boundary

Offline caches are derived availability mechanisms, not systems of record.

A future cache profile must bind:
- exact protected payload/envelope version;
- device/key profile;
- expiry;
- maximum offline duration;
- revocation/currentness behavior after reconnect;
- wipe/retirement semantics and their limitations;
- access-accountability behavior while offline.

```text
OfflineCache != SystemOfRecord
```

## Hybrid does not mean duplicate everything

Every datum has one declared storage owner. A hybrid profile is not permission to place sensitive plaintext in public DHT, restricted DNA, external store and local cache simultaneously.

```text
HybridProfile != DuplicateSensitivePlaintextEverywhere
```

## Required non-equivalences

```text
PrivateEntry != MultiPartyProtectedDatabase
EncryptedDhtCiphertext != MetadataPrivacy
RestrictedDnaMembership != PerRecordLeastPrivilege
ExternalProtectedStore != MycelixVerifiedTruth
HybridProfile != DuplicateSensitivePlaintextEverywhere
OfflineCache != SystemOfRecord
DeleteAction != GuaranteedGlobalErasure
StorageAvailability != RecipientAuthorization
StorageProfileSelected != LegalCompliance
JohannesburgCandidateProfile != JohannesburgDeploymentReady
```

## Continuation

```text
SUP-CIV-000D0 reuse ownership
        ├── SUP-CIV-000D1 envelope/crypto line
        └── SUP-CIV-000D2 storage topology           <- this tranche
                         \
                          -> SUP-CIV-000D3 executable composition
```

## Claim ceiling

A PASS may establish only the closed topology vocabulary, selection rules, fail-closed fallback rule and profile-relative nonclaims for this exact subject. It does not establish encrypted storage correctness, recipient authorization, legal compliance, anonymity, global erasure, municipal authority, Johannesburg deployment readiness or universal suitability.
