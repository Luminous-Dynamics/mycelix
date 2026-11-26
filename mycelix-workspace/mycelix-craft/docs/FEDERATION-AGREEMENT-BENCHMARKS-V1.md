# Craft Federation Agreement V1 — Qualification Benchmarks

- **Work item:** CRAFT-FEDERATION-AGREEMENT-001 (#6737)
- **Status:** Proposed qualification protocol; no measured results
- **Version:** 1.0-draft
- **Date:** 2026-11-26

## Safety and sovereignty invariants

| Measure | Target |
| --- | --- |
| Transitive permissions accepted | 0 |
| Implicit reverse edges accepted | 0 |
| Same-federation membership treated as trust | 0 |
| Out-of-scope or wrong-purpose requests accepted | 0 |
| Changed-term or missing-party consent treated as active | 0 |
| Stale/revoked local authority snapshots accepted | 0 |
| Resource reservation/custody inferred from coordination | 0 |
| Settlement or ledger writes from the agreement core | 0 |
| Frozen positive/negative corpus classification | 100% |
| Independent replay of golden vectors | 100% |

Targets are qualification requirements, not measured results.

## Adversarial graph corpus

Generate directed graphs with 2–100 institutions and test:
- A→B, B→A absent;
- A→B and B→C, A→C absent;
- a complete set of federation members with no cooperation edges;
- multiple scoped edges between one pair;
- overlapping but unequal scope digests;
- duplicate party, consent, edge and scope identities;
- partial consent, wrong terms digest and stale local decisions;
- effective withdrawal at t−1, t, and t+1;
- agreement start/expiry boundaries;
- suspended, terminated and proposed states;
- party insertion/removal and treaty revision;
- replay of an old agreement after partition healing;
- disclosure purpose and profile mismatch.

## Operational measures

Measure separately:
- time and cost to negotiate the first bilateral agreement;
- time and cost to add a new scope without rewriting unrelated scopes;
- number of local approval steps and re-approvals after revision;
- dispute frequency and resolution duration;
- exit and revocation latency;
- cross-community coordination latency under intermittent connectivity;
- information fields disclosed per interaction;
- unauthorized inference attempts against disclosure profiles;
- number of downstream owner approvals needed for resource/evidence/economic effects.

## Matched baselines

Compare:
1. ad-hoc email/manual MOUs and human checklists;
2. centralized federation administrators and shared role directories;
3. shared-membership trust assumptions;
4. direct, signed, versioned and purpose-bound agreement references.

The comparison must use identical scenarios and threat models. Report administrative burden, local autonomy, privacy exposure, failure handling and coordination latency separately. Do not collapse these into one federation score.

## Field pilot

A prospective water-pump restoration can test two independent institutions:
- institution A accepts a pinned electrical-inspection framework from B;
- B does not gain reciprocal recognition without its own explicit edge;
- both agree to purpose-limited evidence disclosure;
- mutual-aid coordination may be used to request help;
- a resource owner separately authorizes any equipment access;
- each institution retains local project, member, professional and financial authority.

Independent review should record negotiation time, consent comprehension, disclosure volume, dispute handling and actual service outcome. A successful agreement test is not proof the repair worked.
