# Community Knowledge Governance Contract — HMP-001 v0.1

**Status:** Proposed governance/protocol requirements. No runtime enforcement is claimed.  
**Date:** 2026-11-14  
**Parent:** [Symthaea HMP-001](https://github.com/Luminous-Dynamics/symthaea/blob/research/human-meaning-pluralism-v0-1/docs/research/HUMAN_MEANING_PLURALISM_V0_1.md)

## 1. Purpose

Mycelix should provide the authority, permissions, custody, correction and accountability layer for knowledge shared by people and communities. This includes cultural heritage, religious and spiritual traditions, oral histories, local ecological knowledge, community practices, psychological research data and philosophical work.

The design must not assume that all knowledge is open merely because it can be found online. Some material should be represented only by a metadata placeholder; some should not be collected at all.

This contract is people- and purpose-oriented. It uses the CARE Principles (Collective Benefit, Authority to Control, Responsibility and Ethics) alongside—not instead of—appropriate FAIR data practices. [Global Indigenous Data Alliance](https://www.gida-global.org/careprinciples)

## 2. Governing invariants

1. **Authority is explicit.** A model, external researcher or single contributor cannot appoint itself the authority for a community's knowledge. Record the relevant holder, authorized representative or community-defined governance process, including scope and expiry.
2. **Permission is operation-specific.** Read, display, index, retrieve, summarize, translate, adapt, train, redistribute and export are distinct operations. Permission for one does not imply permission for another.
3. **Default deny.** Missing, ambiguous, pending, denied, expired or withdrawn permissions block the affected operation. Do not infer permission from public availability, citation, prior ingestion or a reputation score.
4. **No consensus-to-truth conversion.** DHT agreement can attest that a specified record/revision was accepted under a specified protocol. It cannot establish that a doctrinal, historical, metaphysical or cultural claim is true.
5. **No belief scoring.** Mycelix trust/reputation mechanisms must not rate the worth, credibility or rights of a person based on religion, nonreligion, ritual participation, worldview or protected identity.
6. **Benefit and reciprocity are recorded.** Where community contributors request it, record attribution, access to outputs, compensation or other agreed benefits. Do not assume publication itself is a community benefit.
7. **Correction and withdrawal propagate.** Corrections, scope changes, authority changes and revocations must invalidate derived access decisions and cached representations within a defined service-level objective.

## 3. Permission object

The HMP-001 knowledge record schema is the interchange starting point: [schema in Symthaea](https://github.com/Luminous-Dynamics/symthaea/blob/research/human-meaning-pluralism-v0-1/schemas/pluralistic-knowledge-record-v0.1.0.schema.json).

A runtime authorization decision should additionally bind:

- record identifier and immutable content digest;
- actor/service identity and verified role;
- operation requested;
- authority record and permission reference;
- purpose and project context;
- time of decision, expiry and policy version;
- result: **ALLOW**, **DENY** or **INDETERMINATE**;
- reason code and audit receipt.

**INDETERMINATE is not an allow.** Unknown authority, stale policy, unavailable revocation state, conflicting grants or an unverifiable actor must fail closed for protected operations.

Recommended access classes:

| Class | Meaning | Default |
|---|---|---|
| **public_source** | Publicly accessible source with a separately established reuse basis | No model-training permission inferred |
| **community_controlled** | Content whose use is governed by a community-defined authority | Deny until scoped authorization |
| **restricted** | Sacred, initiatory, confidential, safety-sensitive or otherwise restricted content | No content ingestion by default; metadata placeholder only where authorized |
| **reference_only** | An item may be cited as existing, but the content is not licensed/authorized for copying or model use | No copying, retrieval or training |

Permission basis should be explicit: documented license/public-domain status, direct permission, community authorization or another reviewed legal basis. Retain a reference to the actual permission evidence rather than a bare boolean.

## 4. Storage and lifecycle

- Do not put sensitive or restricted content into a broadly replicated DHT as a shortcut to availability.
- Separate public metadata from protected payloads. A digest proves byte identity, not permission, truth or safe disclosure.
- Store only the minimum metadata needed to enforce rights. Sensitive coordinates, personal narratives, names and ceremonial details require purpose limitation and minimization.
- Define retention and expiry before ingestion.
- Make revocation operational: update policy state, stop future access, invalidate caches/indexes and issue a correction/revocation receipt. Do not promise that already-disclosed information can be recalled.
- Preserve a non-sensitive audit trail of the decision and revision without retaining prohibited content.
- Where authority is disputed, quarantine the record from new use until a legitimate process resolves the dispute. Do not let an automated majority decide who owns a tradition.

## 5. Negative-control test matrix

These are required future tests, not existing passing tests.

| Case | Expected decision |
|---|---|
| Public web page, no license or reuse basis | Deny model training; allow only the specifically justified operation |
| Explicitly licensed public source, in-scope citation | Allow that operation only |
| Community-controlled source, no authority reference | Deny |
| Community-controlled source, grant for display but not training | Allow display; deny training |
| Restricted ceremonial record | Deny content read/retrieval/export; metadata-only if separately authorized |
| Permission pending, expired, denied or withdrawn | Deny affected operation |
| Revocation occurs while a result is cached | Invalidate cache; deny subsequent read |
| Two conflicting authority records | INDETERMINATE then quarantine; do not choose by popularity |
| Actor's identity or role cannot be verified | Deny protected operation |
| Digest matches but permission is absent | Deny; integrity is not authorization |

The acceptance condition for this suite is **zero unauthorized reads, exports, translations, retrievals or training uses**. A single hard-gate failure blocks release regardless of other benchmark scores.

## 6. Separation of concerns

- Source provenance answers: where did this claim come from?
- Epistemic assessment answers: what kind of claim is this, and what evidence supports it?
- Permission answers: which actor may perform which operation for which purpose?
- Governance answers: who is authorized to decide or correct the record?
- Replication/consensus answers: what state was recorded under the protocol?

Never collapse these into one trust score.

## 7. Implementation sequence

1. Review existing Mycelix identity, consent, attribution, Knowledge and governance primitives; do not duplicate them.
2. Map the HMP-001 permission vocabulary onto current public APIs and record missing primitives as explicit blockers.
3. Implement a pure, deterministic policy evaluator with ALLOW / DENY / INDETERMINATE outputs and reason codes.
4. Add the negative-control matrix above as fixtures; test stale policy, revoked permission, cache invalidation and conflicting authority.
5. Add multi-agent integration tests proving that each node enforces the same signed policy state and that partitions/stale state do not silently authorize use.
6. Conduct a threat model and community review before storing any real community-controlled or sensitive knowledge.


## 9. Existing Mycelix substrate mapping — do not duplicate it

A repository read of the current public main branch found relevant work that HMP should integrate with rather than replace. This is a source-level mapping, not a runtime integration test.

| Existing surface | What is present in source/docs | HMP implication |
|---|---|---|
| mycelix-knowledge/zomes/claims/integrity/src/lib.rs | Legacy Claim and ClassifiedClaim include content, author, source references, claim type, domain and classification. The E/N/M/H model distinguishes empirical, normative, materiality and harmonic dimensions. | Extend/map this record carefully; HMP needs attributed voice, scope, time/place/language, interpretation status and operation-specific rights. |
| Claims epistemics | NormativeLevel includes contested, emerging and endorsed levels; ClassificationConsensus stores aggregated classification; EpistemicClassification::overall_strength() combines empirical, normative and materiality dimensions. | Do not treat normative consensus or one composite epistemic score as truth. Keep empirical support, normative endorsement, metaphysical commitment, community scope and source provenance separate. |
| mycelix-knowledge/README.md | Describes a five-factor “Truth Engine” whose factors include attestation count, reputation, source quality, time decay and consistency; its wording says “social consensus defeats misinformation.” | This is a design/README claim, not independent proof of misinformation detection. HMP must explicitly guard against popularity/reputation becoming truth, especially for religious, philosophical and identity claims. |
| mycelix-core/docs/civilizational-os/happs/MYCELIX_CHRONICLE_DESIGN.md | A design document describes archive provenance and an AccessPolicy abstraction. The document itself is marked design phase. | Reuse its vocabulary only after checking current implementation and maturity. Do not assume Chronicle's proposed access-control flow is deployed or enforceable. |
| mycelix-personal/zomes/health-vault | A health-specific ConsentGrant binds grantee, record types, expiry and active status; integrity validation requires record types. | Useful precedent, but not a general cultural-knowledge permission system. Do not repurpose health consent semantics without a domain-neutral threat model and explicit rights scopes. |

### 9.1 Required epistemic separation

HMP's data model must not encode a single scalar “truth” or “trust” value for a knowledge record. At minimum retain separately:

- **Empirical support:** observations, experiments, historical documents and replication status.
- **Interpretive provenance:** whose reading or translation is represented, and its scope.
- **Normative stance:** what an individual or group endorses, rejects or contests.
- **Metaphysical status:** whether a proposition is theological, philosophical, experiential or empirically testable.
- **Community authority:** who may authorize uses of community-controlled knowledge.
- **Permission state:** which operations are allowed for which actor and purpose.
- **Review status:** draft, source checked, expert reviewed, community reviewed, contested or retracted.

A vote can establish that a group recorded an endorsement under a specified rule. It cannot establish that a deity exists, that a ritual has one universal meaning, that a minority interpretation is false, or that a historical event occurred.

### 9.2 Integration decision

The first implementation target should be a domain-neutral, pure authorization evaluator that can be called by the claims/knowledge coordinator and any future Chronicle adapter. It must not live only in the health vault. The evaluator should consume signed, versioned grants and explicit revocation state; any stale, missing, contradictory or unverifiable state returns INDETERMINATE and denies protected operations.

Before implementation, inspect the actual claims coordinator read/write paths and Chronicle maturity. The current HMP RFC does not yet prove that knowledge retrieval is permission-gated end-to-end. In particular, entry integrity validation alone cannot prevent an unauthorized coordinator from returning an otherwise valid record.

## 10. First executable policy prototype — HMP-001 research branch

A pure Rust decision kernel now exists at `mycelix-knowledge/zomes/claims/coordinator/src/hmp_rights.rs`. It returns ALLOW, DENY or INDETERMINATE for one requested operation using an already-authenticated actor and an already-fetched permission snapshot.

The kernel checks:
- authenticated actor binding and grantee binding;
- record identity and declared purpose;
- explicit permission status;
- verified community authority and authority reference for community-controlled/restricted records;
- known, active revocation state;
- permission reference and basis;
- grant validity window;
- per-operation allowed and denied lists, including conflicting rules.

It contains 15 focused unit tests for scoped citation/display, denied training/retrieval, revocation, unknown revocation, authentication mismatch, pending status, disputed authority, missing authority reference, record/grantee/purpose mismatch, expiry, conflicting rules and missing grants.

**Important boundary:** this kernel is a deterministic policy function, not a complete authorization system. It does not verify signatures, fetch current DHT authority/revocation state, resolve authority disputes, persist grants, invalidate retrieval/index caches or enforce a read path. The existing `get_claim` and `latest_claim_records_from_links` paths are not yet wired to HMP rights. The public claims model is currently designed around broadly retrievable claims; community-controlled knowledge must not be placed into that path until authorization is enforced end-to-end.

The next required integration step is a separate HMP knowledge entry or access-controlled envelope with immutable content digest, signed grant/revocation references and coordinator-level authorization before content retrieval. Do not retrofit access control onto public claims by silently changing their semantics.
## 8. Claim boundary

This document is a governance contract only. It does not establish that the current Mycelix implementation enforces these rules, that any community has authorized the project, that existing DHT/storage behavior satisfies the requirements, or that legal compliance has been assessed.
