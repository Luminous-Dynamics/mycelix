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

## 8. Claim boundary

This document is a governance contract only. It does not establish that the current Mycelix implementation enforces these rules, that any community has authorized the project, that existing DHT/storage behavior satisfies the requirements, or that legal compliance has been assessed.
