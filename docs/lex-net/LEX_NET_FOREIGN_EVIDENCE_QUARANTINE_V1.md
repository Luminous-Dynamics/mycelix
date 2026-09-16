# LEX-NET-018 — Foreign-Evidence Quarantine and Explicit Promotion Boundary v1

Status: draft executable contract for issue #1113.

## Governing theorem

`received foreign evidence != local canonical fact != locally recognized evidence != local authority`.

A foreign object may be authentic, schema-valid, semantically translated, and recognized for a bounded purpose while still remaining foreign-origin evidence. LEX-NET-018 therefore separates storage/admission from local authoritative state and requires explicit promotion of a minimal recognized projection.

## Scope

LEX-NET-018 is a deterministic, effects-disabled staging contract. It does not fetch remote evidence, interpret law, perform identity resolution, decide recognition policy, mint capabilities, authorize actions, or execute external effects.

The v1 contract composes already-qualified inputs from:

- LEX-NET-001 local recognition; and
- LEX-NET-017 semantic translation fidelity when translation occurred.

Future interpretation, transparency/equivocation, currentness, and dependency receipts may be added by later qualified profiles. Their planned existence is not treated as qualified evidence in v1.

## Trust domains

The contract distinguishes at least these domains:

1. **ForeignSource** — immutable source bytes or source commitment plus source provenance.
2. **ForeignDerived** — translated/derived foreign projections whose lineage still points to ForeignSource.
3. **Quarantine** — locally stored foreign evidence that is inspectable only through bounded evidence interfaces and is not authoritative domain state.
4. **RecognizedProjection** — a new minimal local projection created from an explicit positive, current, purpose-bound LEX-NET-001 recognition result.
5. **AuthoritativeDomainState** — outside LEX-NET-018. Entry requires a separately qualified domain transition and authority/effect contract.

Moving data between storage locations does not change its trust domain. Copying, caching, indexing, serializing, materializing, or re-enveloping foreign evidence cannot make it local authoritative state.

## Required quarantine record

A `QuarantineRecordV1`-equivalent representation binds:

- exact foreign source commitment;
- claimed source/profile identity where supplied;
- origin-verification evidence reference where supplied;
- exact LEX-NET-017 TranslationReceipt commitment when translation occurred;
- translated projection commitment when present;
- ingest identity / local quarantine namespace;
- deterministic state/disposition;
- retention class or policy reference without requiring indefinite plaintext retention;
- explicit `foreign_origin = true` invariant;
- explicit non-authority flags;
- predecessor/supersession references rather than in-place history rewrite.

The contract does not require protected source payloads to be copied into every receipt. Commitments and bounded projections are preferred where possible.

## Promotion contract

Promotion is not mutation of the foreign object. A positive local recognition result may produce a separate `RecognizedProjectionV1` only when all required bindings match.

The projection binds:

- exact foreign source commitment;
- exact quarantine-record commitment;
- exact recognition receipt / recognition result commitment;
- exact local recognition policy/profile identity;
- exact purpose;
- exact resource/action domain where the recognition profile supplies one;
- exact recognized claim names and values;
- exact projection commitment;
- validity/currentness material supplied by the recognition result;
- explicit foreign-source lineage;
- explicit `grants_local_authority = false`;
- explicit `grants_external_effect_authority = false`.

Only claims explicitly recognized by the input recognition result may appear in the promoted projection. Unrecognized fields in the foreign object remain quarantined.

## Storage and indexing boundary

Before explicit promotion, quarantined evidence MUST NOT:

- enter authoritative identity or principal indexes;
- satisfy authority/delegation lookups;
- trigger payments, provisioning, release, adjudication, eligibility, sanctions, reputation, scoring, or other consequential automation;
- participate in local-authority namespace resolution;
- become canonical merely because a cache/materialized view copied it;
- be joined into authoritative domain records by textual identifier equality;
- be exposed through unrelated search/index/log surfaces;
- cause one safe field to mark the whole source object trusted.

A recognized projection may be indexed only within the exact purpose/domain permitted by the local profile. That index remains evidence-oriented and does not itself mint authority.

## Namespace isolation

Foreign identifiers are namespaced by their source/profile context. A foreign identifier equal in text to a local principal, capability, resource, or authority identifier does not collide with or inherit the local object's semantics.

`foreign:example:admin` and `local:admin` are distinct even if their human-readable labels match.

## Provenance / taint invariants

1. Foreign-origin provenance survives translation, caching, projection, promotion, export, and replay.
2. A local recognized projection references its source; it never rewrites the source as locally authored.
3. Recognition for purpose A cannot be replayed for purpose B.
4. Recognition of claim subset S cannot authorize use of source claims outside S.
5. Source update/replacement creates a new evidence lineage.
6. Recognition-policy/currentness change creates new reliance evidence; historical receipts remain immutable.
7. Rejection or later revocation blocks future positive use under the affected profile but does not delete the historical fact that a prior evaluation occurred.
8. Conflicting foreign histories remain distinct evidence; quarantine never performs last-write-wins reconciliation.

## Retention and privacy

Auditability does not require permanent plaintext retention. A deployment may retain commitments, receipts, tombstones, encrypted blobs, or other policy-compliant evidence while deleting or cryptographically erasing source payloads where appropriate.

Quarantine data is not globally discoverable merely because it is retained. Search, disclosure, export, and operator access remain separately authorized and purpose-limited.

## Resource and parser safety

This contract receives already-decoded frozen test inputs. Production adapters remain responsible for bounded parsing, size/depth limits, media-type handling, and canonical interpretation. LEX-NET-018 must not infer trust from successful parsing.

## Reference dispositions

The v1 evaluator uses exactly these bounded dispositions:

- `Quarantined`
- `PromotedRecognizedProjection`
- `PromotionRejected`
- `PromotionIndeterminate`
- `PromotionUnsupported`

No disposition means factual truth, legal validity, local authority, capability issuance, or external effect.

## Promotion preconditions

A positive `PromotedRecognizedProjection` requires all of the following:

- foreign source commitment present;
- quarantine state valid;
- recognition disposition exactly `RecognizedEvidence`;
- recognition result bound to the same source/projection commitment;
- exact purpose match;
- recognized claim list is non-empty;
- every promoted claim is in the recognized claim list;
- no duplicate claim names;
- no authority/effect flag in source data can be promoted merely because it exists;
- if translation occurred, the exact TranslationReceipt commitment and target projection commitment must match;
- the input recognition state is not expired/revoked/indeterminate under the frozen fixture.

Missing or conflicting evidence yields `PromotionIndeterminate` where the condition could be resolved by additional evidence; explicit negative recognition yields `PromotionRejected`; unsupported receipt/profile forms yield `PromotionUnsupported`.

## Adversarial corpus

The frozen corpus includes at least:

1. raw authenticated foreign object cannot write directly to local canonical state;
2. one recognized safe field does not promote the whole source document;
3. foreign `admin_capability=true` remains excluded when not in recognized claims;
4. customs-purpose recognition replayed for payment purpose is rejected;
5. recognition bound to source A cannot promote source B;
6. translation receipt/output commitment mismatch fails closed;
7. foreign identifier equal to local principal identifier does not collide;
8. cache/materialized-view copy preserves foreign provenance;
9. stale/revoked recognition cannot create a new positive projection;
10. source update creates a new lineage rather than overwriting historical source commitment;
11. unrelated search/index exposure is forbidden by the modeled storage policy;
12. positive recognized subset creates exactly the bounded projection and no authority.

## Nonclaims

A LEX-NET-018 PASS does not establish factual truth, legal validity, identity truth, universal trust, regulatory compliance, data-protection compliance, local authority, capability issuance, external-effect authority, production security, production storage safety, or fitness for any jurisdiction.

It proves only the frozen quarantine/promotion semantics for the exact synthetic corpus and exact qualified parent lineage.
