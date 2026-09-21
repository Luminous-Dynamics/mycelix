# FIN-SYNC-003B0 structural rail-capability evidence v1

Status: source specification for the first pure FIN-SYNC-003B0 candidate.

## Claim boundary

A `RailCapabilityEvidenceBundleV1` binds bounded evidence to one exact FIN-SYNC-003A static rail-capability profile.

It is deliberately **not** a currentness theorem:

```text
structural capability evidence assembled
!= evidence current now
!= provider capability current
!= adapter instance current
!= credential/session valid
!= execution authorized
!= capacity reserved
!= plan admitted
!= operation dispatched
```

FIN-SYNC-003B1 owns the later trusted-time currentness join. FIN-SYNC-003D owns execution credential/authority/capacity admission.

## Why this tranche exists

Provider and adapter evidence often carries chronology fields, version numbers, publication dates, signed policy periods, or observation timestamps. Those are useful evidence but they do not provide a trusted `now`.

```text
signed timestamp
!= trusted current time

larger source revision
!= authority to erase independent contradictory evidence
```

003B0 therefore records source chronology as opaque profile-bound commitments and never compares it to `CLOCK_REALTIME` or another local clock.

## Exact parent binding

Every bundle binds one exact FIN-SYNC-003A `RailCapabilityProfileV1` by `profile_commitment`.

```text
003B0 evidence for static profile A
!= evidence for static profile B
```

## Closed evidence dimensions

V1 uses explicit dimensions for adapter artifact/configuration, provider API profile, capacity lock, prepare, commit, cancel/abort, query, evidence production, reversal, atomicity, idempotency, finality profile, synchronization profile, disclosure, timing, and resources.

These dimensions identify what a source is talking about. They do not form a strength ordering.

## Evidence claims

An item carries one closed structural claim class:

```text
SupportsDeclaredStaticSemantics
ContradictsDeclaredStaticSemantics
ObservesRelatedFact
Indeterminate
```

003B0 preserves both identity conflicts and independent semantic contradictions.

### Evidence-identity conflict

```text
same evidence_id + same canonical item
-> exact replay; idempotent

same evidence_id + changed canonical item
-> IdentityConflicted
```

### Independent semantic contradiction

Support and contradiction from distinct evidence identities about the same exact `(dimension, claim_subject_commitment)` are retained as a separate fact:

```text
source A supports subject S
+ source B contradicts subject S
-> SemanticallyContradicted
```

No source wins by arrival order, larger revision number, or later-looking timestamp.

The disposition vocabulary is therefore:

```text
NoDetectedConflict
IdentityConflicted
SemanticallyContradicted
IdentityConflictedAndSemanticallyContradicted
```

`NoDetectedConflict` is intentionally narrow:

```text
no detected identity/semantic conflict
!= sufficient evidence exists
!= static capability is true
!= capability is current
```

An empty or wholly indeterminate bundle may still have `NoDetectedConflict`; downstream theorems must separately prove evidence sufficiency/currentness.

## Source chronology is evidence only

Chronology is represented as:

```text
SourceChronologyEvidenceV1 {
    chronology_profile,
    chronology_commitment,
}
```

There is intentionally no `now`, `is_current`, `fresh`, `valid_now`, or ambient clock field.

A chronology commitment may bind a provider timestamp, signed issuance time, policy validity statement, build timestamp, or another chronology object under its own profile. FIN-SYNC-003B1 later evaluates currentness only after joining a qualified trusted-current-time theorem.

## No credential authority

003B0 contains no bearer API token, OAuth/session secret, private key, account password, prepared-operation bearer handle, or credential-validity boolean.

```text
provider exposes primitive P
!= this principal may invoke P
```

Credential scope/currentness belongs to FIN-SYNC-003D.

## Canonical item identity

Each evidence item commitment is:

```text
SHA256(
  "MYCELIX_FIN_SYNC_CAPABILITY_EVIDENCE_ITEM_V1\0"
  || u16(commitment_profile_revision = 1)
  || static_profile_commitment
  || u8(dimension_tag)
  || text(evidence_id)
  || evidence_commitment
  || source_profile
  || u8(claim_tag)
  || claim_subject_commitment
  || optional(source_revision)
  || optional(chronology_profile || chronology_commitment)
)
```

Text is `u32(byte_length) || UTF-8`; integers are unsigned big-endian; profile refs use exact `(id, revision, digest)` canonical bytes.

## Canonical bundle identity

The constructor:

1. rejects the wrong static profile commitment;
2. enforces physical total and per-dimension bounds;
3. computes every item commitment;
4. drops only exact duplicate identity+item replays;
5. preserves every unique changed identity reuse;
6. detects support/contradiction pairs for exact claim subjects;
7. sorts positive records by raw item-commitment bytes;
8. sorts identity conflicts and contradicted subjects canonically;
9. derives the exact structural disposition;
10. hashes the complete positive receipt.

The bundle commitment binds the derived records, identity conflicts, contradicted claim subjects, and disposition—not merely a bag of source hashes.

## Bounds

V1 freezes semantic post-decode limits:

```text
MAX_EVIDENCE_ITEMS = 512
MAX_EVIDENCE_ITEMS_PER_DIMENSION = 64
MAX_CONFLICT_IDENTITIES = 128
MAX_CONTRADICTED_SUBJECTS = 128
```

These are not raw parser-allocation theorems.

## Frozen independent vector

Parent static capability profile:

```text
9b7e2d1f6976f85a568c78cd2be92013e3ab8776119c0477f46aa91348280c0d
```

Adapter artifact evidence item:

```text
fad252e445eefdc1a63a86697f0a271ef57b8806bcc4aa22b29de8089853a8b4
```

Provider API evidence item:

```text
a27525fbcd3ac7e94560bc5336e57ed7a0c59bc69edc5d3820ce98e639f94da4
```

No-detected-conflict two-item bundle:

```text
3493d337413c6efa6d372f67ce45bf2c9072d2269f1edbb1e78c0063207816d4
```

The commitment is unchanged by the `Uncontested` -> `NoDetectedConflict` source-label hardening because the canonical V1 disposition tag remains `1`.

These values were independently reconstructed from the frozen language-neutral byte format before source publication. A later executable qualifier must independently reproduce them again.

## Adversarial source corpus

The authored tests cover:

- exact parent static-profile vector;
- independent item and bundle vectors;
- empty evidence => `NoDetectedConflict`, never positive capability;
- input permutation invariance;
- exact duplicate evidence idempotence;
- same evidence ID + changed semantics => identity conflict;
- independent support + contradiction => semantic contradiction;
- simultaneous identity conflict + semantic contradiction;
- large source revision cannot suppress another source;
- chronology changes identity but creates no currentness claim;
- wrong static profile binding fails closed;
- physical per-dimension bounds;
- portable positive output contains no credential/current/bearer/secret surface.

Authored tests are source coverage only until executed under an exact qualification environment.

## Promotion path

```text
003A source
   ↓
003B0 source
   ↓
independent exact-head diagnostic
   ↓
Cargo-derived dependency capsule
   ↓
exact-head qualification
   ↓
QualifiedStructuralRailCapabilityEvidenceV1
```

003B1 remains blocked from minting any `CurrentRailCapabilityWitnessV1` until a trusted-current-time theorem is independently qualified.

## Nonclaims

A PASS for this tranche may establish deterministic bounded structural evidence assembly only.

It does not establish live provider truth, evidence freshness, current capability, credential validity, financial authority, reservation/capacity, plan admission, dispatch, settlement finality, legal discharge, regulatory compliance, or commercial satisfaction.
