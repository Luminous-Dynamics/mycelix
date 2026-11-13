# IPB-S4 — Federation adapter and sovereignty conformance

**Status:** Draft v0.1  
**Parent:** LISE / Mycelix #6119  
**Manifest:** `IPB_S4_FEDERATED_SOVEREIGNTY.yaml`  
**Claim ceiling:** Synthetic benchmark specification; no production, legal-compliance or field-effectiveness claim.

## 1. Why S4 exists

S2 tests local expenditure authorization. S3 tests coordinated control inside one institution. S4 tests the boundary between institutions.

The key failure is **authority laundering across federation**: one institution issues a credential, vote, approval, service attestation or execution receipt and another system mistakenly treats it as local authority.

The benchmark separates:
- identity/authentication: who presented an assertion and whether its cryptographic envelope verifies;
- claim validity: whether the asserted fact is supported, current and within issuer scope;
- bilateral mandate: what the parties agreed to exchange or perform;
- local institutional authority: who may take an action under each institution's own current rules;
- execution: what was attempted;
- observed effect: what independently happened;
- challenge/remedy: how each party contests and corrects the result.

A valid signature is evidence about the signed statement. It is not, by itself, proof that the statement is true, that its issuer has power over the recipient, or that the recipient is authorized to execute the requested action.

## 2. Reference federation pattern

Two institutions share bounded service evidence:

- Institution A owns a resource and retains authority over its treasury.
- Institution B supplies a service and can attest to a service-completion fact.
- The bilateral mandate permits B to submit service evidence for one named purpose and resource.
- A can use that evidence in its own decision, but must apply its own local constitution, policy and authority before any local expenditure.
- Both institutions can refuse a request under their own rules.
- The mandate defines validity, purpose, data fields, revocation, dispute, liability, in-flight reconciliation and exit.

**The federation does not merge the constitutions.** Each side evaluates its own rule set and retains its own decision/authority references. A shared mandate is a contract between the sides, not a master constitution or a universal authority token.

The exact initial profile is frozen in the YAML manifest.

## 3. Authority non-transitivity and delegation conservation

A partner's authenticated identity is not automatically a partner's authority, and a partner's authority is not automatically authority in the receiving institution.

Every action must be independently traceable to:
1. the actor's current authority in the institution originating the action;
2. the bilateral mandate and its version, scope, purpose and validity;
3. the receiving institution's independently recorded acceptance;
4. the receiving institution's own local authority for any action taken under its jurisdiction.

Delegation is bounded by all relevant source constraints. A subdelegate cannot receive more authority, data rights, resource scope, time or purpose than the delegator validly holds and the mandate permits. If the native authority contracts cannot express a needed restriction, the adapter reports unsupported/indeterminate; it does not invent a parallel authority engine.

No scalar “federation trust score” is used. A successful evidence exchange, a local authorization, and an externally observed effect are distinct outcomes.

## 4. Data-use and privacy contract

Each exchange binds:
- named purpose;
- recipient institution and service;
- resource and operation;
- allowed data fields/categories;
- retention/deletion requirements where applicable;
- mandate version and expiry;
- revocation state/epoch;
- downstream sub-processing/delegation constraints.

The adapter records **requested**, **permitted**, **disclosed** and **retained** data separately. It must not equate a technically possible disclosure with an authorized disclosure.

Where selective disclosure is supported, reveal only the claims necessary for the agreed purpose. The W3C credential security Recommendation defines JOSE, SD-JWT and COSE mechanisms for securing credentials/presentations; those mechanisms support integrity, authenticity, encryption and selective disclosure but do not define local institutional authorization or prove substantive claim truth.

If field-level disclosure cannot be observed, record it as unavailable or not measured. Do not score an opaque payload as data minimization.

## 5. Time, revocation and stale assertions

Federation needs explicit time semantics:
- mandate validity is bounded, with exact inclusive/exclusive semantics;
- assertion issuance time, expiry and revocation epoch are distinct;
- receiving-side observation time is recorded separately from source event time;
- cached policy and credential state cannot silently override current local rules;
- later corrections do not rewrite what an institution knew at the historical decision time.

A valid signature over a stale or revoked assertion is not current authority. The receiver must apply its current local acceptance and revocation rules. The benchmark tests replay after expiry/revocation and unilateral local policy change.

## 6. Unknown execution and reconciliation

A timeout is not proof that an action failed. It is not proof that it succeeded.

When a cross-institution execution may have mutated a resource but the result is unknown:
- preserve the exact attempt/idempotency key and source receipt;
- block new mutation and unsafe retry;
- query the authoritative executor and/or independent observer;
- reconcile the result to one of confirmed-applied, confirmed-not-applied, partial/contested or still-indeterminate;
- resume only under the owner-native reconciliation rules.

This composes with Mycelix #2953 and #3117. LISE must not implement a competing payment/execution lifecycle.

## 7. Constitutional incompatibility and dispute

Two institutions may have legitimate but incompatible constraints. Do not silently compute a weakest-common-denominator policy or assume that the more permissive side wins.

Each institution keeps:
- its own authoritative text and interpretation;
- its own current policy and authority state;
- the bilateral mandate and accepted version;
- the other side's submitted claim and its own evidence status;
- any challenge, dispute and remedy references.

If a mandate cannot be lawfully or constitutionally performed by one party, that party can refuse or return a typed contested/indeterminate disposition. The dispute path must be known before the conflict occurs. The other institution cannot erase or unilaterally reclassify its evidence.

## 8. Safe exit and continuity

An exit event must distinguish:
- no new work after effective exit;
- in-flight work that is already committed or may have executed;
- outstanding claims, disputes and remedies;
- data return/deletion/retention obligations;
- revocation and cache invalidation;
- continued access to records required for independent audit or remedy;
- replacement provider and successor authority.

A successful exit is not “delete everything and disconnect.” It is safe termination with an auditable disposition of obligations and records.

## 9. Initial 14-case panel

| Case | Attack/control | Expected typed disposition |
|---|---|---|
| S4-01 | Valid bounded evidence exchange | Evidence exchange accepted |
| S4-02 | Partner credential used as local treasury authority | Missing local authority |
| S4-03 | Sub-delegation amplifies authority | Scope mismatch |
| S4-04 | Reuse for unrelated purpose | Purpose mismatch |
| S4-05 | Minimum approved fields only | Evidence exchange accepted |
| S4-06 | Excess personal/financial fields requested | Disclosure denied or minimized |
| S4-07 | Mandate expired | Expired |
| S4-08 | Stale assertion after revocation | Revoked |
| S4-09 | Different constitutional versions over time | Both versions preserved in bitemporal view |
| S4-10 | Non-waivable local conflict | Contested or indeterminate |
| S4-11 | Possible mutation followed by timeout | Reconciliation required |
| S4-12 | Partner exits with work in flight | Partner exited; new work prohibited; reconcile |
| S4-13 | Shared identity/evidence dependency with incomplete inventory | Indeterminate |
| S4-14 | Valid signature but issuer/claim scope is insufficient for local authority | Evidence authenticity may be supported; local authority missing |
| S4-15 | Duplicate/replayed operation ID | Rejected or reconciled; no duplicate mutation |

The manifest's `expected_case_count` should be treated as the locked case count. The table above is descriptive; the manifest is authoritative. *(Note: the first manifest currently contains 14 entries; adding the explicit replay row makes 15. This mismatch must be resolved before freezing the version.)*

## 10. Fair comparisons and benchmarks

Compare:
- manual bilateral agreement and documented local authorization;
- conventional identity federation plus API/workflow access controls;
- a data-space connector/contract implementation where suitable;
- a participatory workflow system when the task includes member participation;
- composed Mycelix owner-native authority/lifecycle/evidence contracts with S4 adapters.

Match parties, constitutions, agreement language, evidence, tasks, timing, resources, threat budget, data fields and evaluator access. A platform is not failed for functions outside its designed scope. Use typed `not_supported` / `not_measured` values.

Report separately:
- unauthorized cross-boundary actions;
- authority laundering and delegation amplification;
- purpose/scope/data disclosure deviations;
- stale/revoked assertion acceptance;
- unknown-outcome retries and duplicate effects;
- local rule and bitemporal reconstruction fidelity;
- dispute/appeal completion and remedy;
- safe-exit completion, in-flight reconciliation and record access;
- privacy exposure, staff burden, integration and maintenance cost.

Do not collapse these into one federation score or vendor ranking.

### Proposed engineering gates

- Zero accepted peer-only authorizations in the locked negative panel.
- Zero delegation amplification.
- Zero excess-field disclosure in the observable field-level panel.
- Zero stale/revoked assertion authorizations.
- Zero blind retry after unknown mutation outcome.
- 100% preservation of both institutions' local policy/constitution versions in bounded bitemporal tests.
- All constitutional conflicts remain explicitly contested/indeterminate until the declared local dispute process resolves them.
- Exit blocks new work, reconciles in-flight work and preserves audit/remedy access.
- Exact owner-native references, policy/mandate versions and evidence dispositions are independently replayable.

These are fixture gates, not empirical estimates of federation risk.

## 11. Industry and standards context

- **OECD, 20 October 2026, Data Spaces between Co-ordination Challenges, Financial Viability and Public Policy.** Data spaces are described as trusted sharing environments based on agreed governance frameworks, policies, semantic models, protocols, processes and facilitating services. The OECD also discusses bootstrapping, coordination and financial sustainability. LISE should benchmark governance and operating arrangements, not simply connector throughput.
- **EU Data Act, Regulation (EU) 2023/2854.** Applies from 12 September 2025 and provides fair-access/use-of-data and interoperability context. IPB-S4 is not a compliance test.
- **W3C Securing Verifiable Credentials using JOSE and COSE, Recommendation 13 October 2026.** Defines JOSE, SD-JWT and COSE security approaches for credentials and presentations. It supports cryptographic integrity/authenticity/confidentiality but does not grant authority across institutions.
- **NIST SP 800-63C-4 (2025).** Covers federation and assertions among separately administered relying parties. S4 adds a distinct institutional authorization and constitutional-scope test above identity federation.
- **NIST IR 8587 (September 2026).** Addresses token/assertion forgery, theft and misuse, including key management and lifecycle controls. S4 should import relevant replay/revocation attack patterns while preserving local authority semantics.

## 12. Owner-native integration map

- Mycelix #3129: interoperability workflow profiles.
- Mycelix #2953/#3117: local authority, capability/lease, execution and outcome lifecycle.
- Mycelix #6069: bitemporal history, challenge and correction.
- Mycelix #3126: simultaneous contested/contradictory evidence and indeterminate outcomes.
- Mycelix #4835: independent evaluator and oracle separation.
- Mycelix #1243/#1371/#5695/#5762: source lineage, correlation and common-cause closure.
- LISE #6119: scenario, threat model, conformance, matched comparison and claim discipline.

The adapter returns native references and typed projections only. No duplicate identity provider, credential model, event store, authority engine, dispute engine or federation bus is introduced.
