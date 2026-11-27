# Craft Work Evidence V1 — reference contract

- **Identifier:** CRAFT-WORK-EVIDENCE-001
- **Status:** Draft reference implementation; not a deployed Holochain protocol
- **Version:** 1
- **Owner:** Mycelix Craft
- **License:** AGPL-3.0-or-later

## Purpose

Represent a bounded claim about a person's contribution to a work event, bind its evidence artifacts by digest, and verify that independent assessors are authorized for the declared professional domain, scope, and jurisdiction. The contract is a narrow evidence boundary; it is not a worker reputation score, credential wallet, job marketplace, payment ledger, or universal licensing registry.

## Required separation

`work completed != work verified != capability demonstrated != compensation settled`

This crate only admits cryptographically valid, policy-scoped attestations about a versioned work claim. It does not independently establish that the attestor observed reality accurately, that source artifacts are truthful, that a credential is legally recognized, that an entire capability has been demonstrated, or that compensation is due.

## Ownership boundaries

- **Craft:** work claims, participant roles/contribution scope, employer/worker agreements, assessor requests, attestations and disputes.
- **Praxis:** learning records, learner-selected evidence, assessment history and advisory capability projections. Craft must not write Praxis history or infer mastery from a caller-supplied scalar.
- **Identity:** identifiers, key ownership, issuer trust, key/credential status and revocation authority. The current reference accepts a caller-supplied trust snapshot; it does not become the identity authority.
- **Manufacturing:** work orders, planned and consumed materials, process events and physical verification. Craft references those records; it does not recreate them.
- **Finance:** payment authorization, journal ownership, settlement and finality. An accepted work-evidence bundle never creates a payable or ledger mutation.
- **Symthaea:** optional advisory matching or evidence-gap analysis. It cannot sign, verify, or promote its own recommendation into an authoritative assessment.

## Contract contents

A `WorkEvidenceClaimV1` binds:
- claim identity and revision;
- subject, organization, professional domain and declared participant role;
- contribution scope and outcome summary;
- optional agreement and Manufacturing work-order references;
- observation interval and creation time;
- evidence artifact identifiers, media types, URIs, SHA-256 digests and capture provenance.

An `AssessorAuthorityV1` binds:
- authority ID and revision;
- issuer DID and issuer key ID;
- one assessor DID;
- professional domain, scope set and jurisdiction;
- validity interval;
- issuer Ed25519 signature over a domain-separated canonical payload.

A `WorkAttestationV1` binds:
- claim digest, attestation ID and one-time nonce;
- assessor identity/key and authority ID;
- domain, scope and jurisdiction;
- corroborated / unable-to-corroborate / disputed outcome;
- observation time;
- explicit conflict disclosure and limitations;
- assessor Ed25519 signature over a domain-separated canonical payload.

The policy and trust store supply:
- trusted and revoked authority IDs;
- issuer and assessor public keys, controller binding, validity and revocation state;
- the required professional domain, exact scope and jurisdiction;
- minimum distinct assessor count, maximum evidence age and prohibited conflicts.

## Verification semantics

The independent verifier checks in fail-closed order:
1. schema versions and bounded structural validity;
2. claim digest binding;
3. subject is not the assessor;
4. authority is trusted, not revoked, and currently valid;
5. domain, exact scope and jurisdiction match policy;
6. observation timestamp is not in the future and satisfies the age limit;
7. conflict disclosure is declared complete and contains no prohibited conflict;
8. outcome is corroborated;
9. issuer key is trusted, controller-bound, in validity and signature-valid;
10. assessor key is trusted, controller-bound, in validity and signature-valid;
11. claim observation, artifact-capture, claim-creation and attestation timestamps form a non-future causal sequence;
12. both attestation replay keys have not already been consumed;
13. assessor quorum is made up of distinct identities.

All required assessors must pass. A duplicate assessor cannot inflate quorum. A changed claim revision changes the claim digest and invalidates old attestations.

## Replay and distributed-state requirement

The verifier returns two namespaced replay keys per attestation: `nonce:assessor_did:authority_id:nonce` and `attestation:authority_id:attestation_id`. The host must atomically reserve/persist both before any consequential downstream action. This in-memory reference library cannot guarantee atomicity across concurrent Holochain calls or nodes. A host that checks and stores replay state in separate non-atomic operations is not protected from races.

## Canonicalization

Signatures use explicit, fixed-field-order, length-prefixed binary encodings with versioned domain separators. SHA-256 digests are calculated over those bytes. Evidence references, authority scopes, conflicts and limitations are sorted before encoding where order is non-semantic. No JSON serializer output is signed. This design must receive independent cryptographic review and cross-language test vectors before production use.

## What is deliberately not claimed

- Hashes bind bytes; they do not prove who created those bytes or whether their contents are truthful.
- A valid signature proves control of the corresponding key at verification time under the supplied trust snapshot; it does not prove professional competence or the truth of the signed statement.
- A conflict declaration is a signed assertion, not proof that undisclosed conflicts do not exist.
- An authority record is meaningful only when the relevant Identity/guild authority and current revocation state are independently trusted.
- “Corroborated” is an assessor opinion under a named scope; it is not regulatory licensing, universal certification, or objective truth.
- A claim-level status/dispute/correction/revocation event log, privacy-preserving selective disclosure, live Holochain integrity validation, Praxis adapter, Manufacturing adapter, and Finance settlement adapter are future work.
- No deployment, hosted CI result, physical repair, assessor registry, legal recognition or production security qualification is implied by this draft.

## Adversarial fixture set

`fixtures/adversarial-v1.json` declares synthetic positive and negative cases. The unit suite maps every case ID to executable verifier input mutation. The fixture set is a regression corpus, not an independent certification or a complete threat model.
