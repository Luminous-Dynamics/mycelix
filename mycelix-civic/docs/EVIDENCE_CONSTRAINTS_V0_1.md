# Civic evidence constraints v0.1

**Status:** additive integrity/coordinator contract. The zome workflow must pass before this is considered validated.

## Purpose

Stores a typed pairwise constraint over two existing Civic evidence IDs. Relations are `MustLink` and `CannotLink`; reasons are contradiction, supersession, scope conflict, privacy conflict, dependency requirement, policy requirement or duplicate claim.

## Author and authority model

- The create API does not accept an issuer DID. The coordinator obtains the caller DID from the active Holochain agent and inserts it into the entry.
- The integrity callback checks that the entry issuer equals `did:mycelix:{action.author}`. The Holochain action itself is the authenticated authorship record.
- Creation is gated by the existing Civic proposal-eligibility requirement.
- `authority_receipt_sha256` is a commitment/reference to external governance material. It is not a signature and does not prove an issuer's authority by itself. Consumers that require delegated authority must resolve and verify the referenced credential/receipt through the issuing Mycelix governance boundary.
- The generic offline credential helper currently uses a BLAKE3 keyed MAC and requires a shared key. It must not be treated as a portable public-key signature.

## Temporal and mutation semantics

- Validity uses a half-open interval `[valid_from, valid_until)` in Holochain timestamp microseconds; null `valid_until` means no planned expiry.
- The coordinator supplies `created_at`; callers cannot choose the issuer or knowledge timestamp.
- The v0.1 entry is append-only: updates and deletes are rejected by integrity validation.
- This means early revocation/shortening is not yet exposed in v0.1. Until a separate author-bound revocation/supersession record and resolver exist, issuers must choose an appropriate validity window at creation. The `Supersession` reason is descriptive and does not automatically deactivate an older record.

## Indexes and API

- A link is created from each evidence endpoint to the constraint entry.
- A stable ID anchor supports lookup by constraint ID; an issuer anchor supports listing constraints authored by a DID.
- Read APIs: `get_evidence_constraints(evidence_id)`, `get_evidence_constraint(id)`, and `get_evidence_constraints_by_issuer(issuer)`.
- If more than one entry is indexed under one constraint ID, lookup fails closed and requires manual integrity review.

## MACE integration

Symthaea should resolve these records through the Civic coordinator, then independently check: the Holochain record/action author, integrity-validation status, Civic eligibility context, record ID and endpoint binding, relation/reason, policy digest, authority receipt, and the record's `created_at` plus validity interval against the requested bitemporal snapshot.

Do not accept a caller-provided DID plus SHA-256 digest as proof of authority. Do not infer that a governance entry verifies the underlying evidence content; it authorizes a relationship between evidence identifiers under a named policy.

## Known limits

- No early revocation record or update flow in this increment.
- The constraint policy digest and authority receipt digest are opaque references; external authority/capability verification remains a consumer responsibility.
- Index-link removal is not yet given a dedicated append-only revocation protocol.
- Pairwise constraints are not a complete global authorization engine; the consumer must still enforce rights, purpose, scope and source lifecycle on each evidence receipt.

The schema file defines both the create-input and stored-record forms. All timestamps are integer microseconds.