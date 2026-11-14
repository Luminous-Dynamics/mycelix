# IPB-S5 — Institutional succession, continuity and recovery conformance

**Status:** Draft v0.1  
**Tracking:** Mycelix #6122  
**Manifest:** `IPB_S5_CONTINUITY.yaml`  
**Evidence ceiling:** Synthetic software benchmark specification; no certification, legal compliance or field-effectiveness claim.

## 1. The paradigm shift: recover authority, not just infrastructure

Traditional business continuity and disaster recovery focus on restoring services, records and technology to an acceptable capacity. LISE must additionally test whether the recovered institution can still act under valid authority and preserve its rights and remedies.

The core invariant is:

> Recovery restores an authorized institutional capability; it does not create new authority merely because the previous authority is unavailable.

A server restart is not proof that:
- the service is running under the current constitution and policy;
- the key used for a new action remains trusted;
- the successor was appointed through a valid process;
- recovered records are complete and authentic;
- an appeal remains independent;
- a provider failover preserves data purpose and authority boundaries.

S5 therefore measures continuity by **capability and authority state**, not by a single organization-wide uptime bit.

## 2. Industry baseline and comparison

LISE should compose with and benchmark against existing continuity systems, not claim to replace them.

| Baseline/framework | Established purpose | S5's additional question |
|---|---|---|
| ISO 22301:2019 + Amd 1:2024 | Requirements for a business continuity management system, covering preparation, response and recovery | Does restored service also restore valid local authority, succession legitimacy, independent review and remedy continuity? |
| ISO/CD 22301 (2026 committee draft) | Proposed revision under development; it is not yet a published replacement for ISO 22301:2019 | How should LISE track continuity requirements as the standard evolves without treating a draft as current certification criteria? |
| ISO 22313:2020 | Guidance for applying ISO 22301 | Does the institution's tested recovery capability match declared continuity objectives in real exercises? |
| NIST SP 800-34 Rev. 1 (2010) | Federal information-system contingency planning guidance | Are system recovery plans connected to constitutional authority, roles, appeals, key custody and records provenance? |
| NIST SP 800-57 Part 1 Rev. 5 (2020) and Rev. 6 initial public draft (2025) | Cryptographic key lifecycle and protection guidance | Does key recovery/rotation preserve historical verification while preventing old or suspect keys from authorizing new actions? |
| Conventional cloud DR / multi-region failover | Service availability, backup and restoration | Does a failover provider have independently validated authority, records and purpose restrictions, or has only the endpoint changed? |
| Organization-specific succession plans | Appointment, delegation and leadership transitions | Are succession events tied to exact constitutional versions, current grants, evidence and independent challenge? |
| Mycelix owner-native lifecycle + S5 adapter | Candidate composition of authority, capability, evidence, history, uncertainty and reconciliation contracts | Does the combined system preserve lawful action, service, rights, remedy and safe exit under matched failures? |

### Current standards status (checked 13 November 2026)

- ISO's published edition is ISO 22301:2019 with Amendment 1:2024. ISO/CD 22301 is in committee-draft development in 2026 and will replace the 2019 edition only if/when a new final edition is published.
- ISO 22313:2020 remains a published guidance edition; ISO's lifecycle indicates a revision project is under way.
- NIST SP 800-34 Rev. 1 is a published contingency-planning guide dating from 2010. It is useful as a baseline framework, not a current implementation benchmark for institution-specific authority recovery.
- NIST SP 800-57 Part 1 Rev. 6 is an initial public draft published 5 December 2025; the final published Part 1 baseline remains Rev. 5 (2020) at the time of this check. The draft includes updated discussion of key establishment/storage and post-quantum algorithms. Treat draft language as research context, not a mandatory current requirement.

These references inform scenario design but do not make S5 a compliance test.

## 3. The five continuity planes

S5 separates five planes because they fail differently:

1. **Authority plane:** current constitutional rule, succession procedure, delegation, local decision rights and current authority snapshot.
2. **Cryptographic plane:** signing keys, trust anchors, key holders, rotation, revocation, recovery and historical signature verification.
3. **Evidence plane:** decision records, manifests, provenance, event-time and as-known-at reconstruction, independent observations and unresolved claims.
4. **Service plane:** infrastructure, networks, runtime, storage, identity service, operators, failover profile and declared degraded capabilities.
5. **Accountability plane:** independent reviewers, appeals, remedy, obligations, non-retaliation, audit retention and dissolution.

Continuity is qualified per essential capability. For example, a service can safely restore read-only history and appeal intake while payment authorization remains suspended pending key and authority revalidation.

## 4. Recovery state machine

The benchmark should make the transition explicit:

`Healthy → FaultDetected → Contained → EvidencePreserved → AuthorityRevalidated → RecoveryAuthorized → DegradedOrFullService → IndependentlyReviewed`

Possible branches:
- `Contested` when successor or recovery evidence conflicts.
- `Indeterminate` when dependency, provenance or execution result is unknown.
- `SafeExit` when recovery is not viable under the authorized procedure.

Never jump from `FaultDetected` to `FullService` merely because a process or endpoint becomes available.

### Recovery requirements

- preserve the pre-failure policy and authority snapshots;
- record the fault event and detection/observation time separately;
- revoke compromised or stale credentials for new actions;
- retain evidence needed to verify historical actions;
- independently verify the replacement role/key/provider according to the applicable owner contract;
- permit only capabilities whose authority, evidence and technical prerequisites are satisfied;
- re-establish appeals, review and remedy before declaring the institution fully recovered;
- record residual uncertainty, lost data, obligations and unresolved disputes.

## 5. Key rotation is not authority succession

Cryptographic and institutional recovery are distinct:

- **Key rotation** replaces compromised, unavailable or expiring signing material.
- **Credential renewal** re-establishes the holder's current assertion under the issuer's policy.
- **Authority succession** determines who may perform an institutional function under a valid constitutional procedure.
- **Provider failover** moves execution or data processing to an alternate service.
- **Records recovery** reconstructs authoritative history from evidence sources.

A replacement key cannot appoint its own holder. A new administrator cannot create their own authority by repairing the server. A restored backup cannot override current revocation state. A historically valid signature remains historically verifiable according to archive policy but must not authorize a new action after revocation.

## 6. Unknown and partial recovery

If an action might have executed before the failure, the system must not assume success or failure. It must preserve the attempt and invoke the owner-native reconciliation path before any retry or new mutation.

If a records backup exists but its provenance or completeness is unknown, retain it as a candidate source; do not silently treat it as an authoritative reconstruction.

If infrastructure is available but the signing key or authority state is not, read-only service may continue only where the scenario explicitly allows it.

If a successor appointment is contested, preserve both claims and route to the constitutional adjudication/appeal path. Do not resolve a governance dispute by selecting whichever credential happens to be operational.

## 7. Initial 15-case panel

| Case | Fault/control | Expected disposition |
|---|---|---|
| S5-01 | Normal operation | Continue under existing authority |
| S5-02 | Founder lost, no valid successor grant | Suspend pending lawful succession |
| S5-03 | Steward lost, preauthorized successor valid | Continue under substitute authority |
| S5-04 | Signing key lost, replacement unverified | Key recovery pending verification |
| S5-05 | Signing key compromised | Revoke for new actions; preserve historical verification |
| S5-06 | Provider outage, approved read-only alternate | Degraded service under verified failover |
| S5-07 | Shared provider/identity dependency unknown | Indeterminate dependency independence |
| S5-08 | Records custodian lost, verified recovery manifest | Continue after independent reconstruction |
| S5-09 | Backup provenance/completeness unknown | No new mutation until recovery verified |
| S5-10 | Stale successor credential | Reject for new authority |
| S5-11 | Bounded emergency read-only role | Continue under temporary limited authority and review |
| S5-12 | Conflicting succession claims | Contested; no irreversible authority change |
| S5-13 | Infrastructure restored, trust/authority not ready | Partial degraded service only |
| S5-14 | Recovery non-viable, authorized dissolution | Safe exit/dissolution initiated |
| S5-15 | Multiple preauthorized substitutes and independent reviewer | Continue without single-person dependency |

## 8. Measurements and benchmark targets

Do not collapse these into an uptime score.

**Continuity by capability**
- availability of each essential function during the fault;
- time to detect, contain, recover and independently verify;
- hours in each degraded/suspended state;
- residual harm and affected participants.

**Authority integrity**
- unauthorized successor actions;
- stale/expired/revoked credential acceptance;
- time to validate current successor authority;
- scope and expiry drift in emergency grants;
- actions attempted while prerequisites are unresolved.

**Cryptographic recovery**
- time to revoke and rotate;
- percentage of historical signatures still verifiable under archive policy;
- replay acceptance after revocation;
- key substitution or holder-binding errors;
- recovery process independence and custodian concentration.

**Evidence continuity**
- RPO against the declared fixture value;
- reconstruction completeness and independent replay rate;
- event-time/as-known-at fidelity;
- provenance gaps, conflicting backups and unknown events;
- appeal/remedy availability during and after recovery.

**Dependency resilience**
- exact shared identities and common-cause cut sets when qualified owner data exists;
- incomplete/unknown dependency counts;
- single-person, single-provider and single-record-custodian concentration;
- correlated failures and restoration order dependencies.

**Governance and human impact**
- access to independent appeal/review;
- participant burden and service exclusion during degraded mode;
- continuity of obligations, non-member rights and remedies;
- legitimacy and contestability of successor appointment.

### Proposed engineering gates

- Zero actions authorized solely due to predecessor unavailability.
- Zero new actions under expired, revoked or suspect keys/credentials.
- Zero recovery actor self-appointments outside the declared constitutional procedure.
- Zero blind retries or duplicate effects after unknown outcomes.
- 100% exact policy/authority snapshot preservation in the frozen panel.
- 100% declared essential functions reach their expected state (continue, degrade, suspend or exit).
- Historical signature verification remains possible after key revocation under the declared archive policy.
- Every contested succession claim stays contested until the authorized dispute process resolves it.
- RTO/RPO are compared to scenario inputs with observed restoration correctness, not merely service-up checks.

These are fixture gates, not guarantees of resilience in every real institution. Real-world targets must be chosen by the institution according to impact and obligations.

## 9. Fair SOTA comparison protocol

Match the same institutional service, fault timing, role graph, constitution, data and record corpus, key lifecycle, recovery resources, RTO/RPO, participant population, accessibility conditions and evaluator.

Baseline arms:
1. Documented manual business-continuity and succession process.
2. Conventional cloud/IT DR with alternate provider and backup restoration.
3. ISO 22301-aligned BCMS and documented exercise evidence.
4. Existing key-management/recovery and identity-federation workflow.
5. Mycelix owner-native authority/evidence/reconciliation composition with S5 projection.
6. Hybrid manual governance plus external independent review.

Do not assume software wins. A platform that is not designed to appoint successors is marked `out_of_scope` for that function, not failed. Report independently:
- correct authority recovery;
- time to service restoration;
- time to evidence reconstruction;
- false recovery/false failover;
- RPO/RTO attainment;
- recovery cost and staff/participant burden;
- appeals and remedies preserved;
- residual risks and common-cause uncertainty.

Use held-out failure combinations and independent evaluators. Publish exact software/configuration/policy/evidence digests, negative controls, missing outcomes and deviations.

## 10. Owner-native integration map

- Mycelix #2953/#3117: authority, capabilities/leases, execution and outcome reconciliation.
- Mycelix #6069: bitemporal history and appeal/correction.
- Mycelix #3126: contested/open-world uncertainty.
- Mycelix #4835: independent evaluator separation.
- Mycelix #5762/#5695/#1243/#1371: common-cause and transitive dependency analysis; pending mainline integration and qualification.
- Symtropy #3684: role concentration and formal separation-of-duties analysis.
- Existing identity, key, storage, provider and succession modules remain owner implementations.

The S5 adapter emits benchmark projections and native references only. It must not become a new key manager, backup fabric, identity provider, authority engine or succession system.

## 11. Standards and source registry

- ISO 22301:2019: https://www.iso.org/standard/75106.html
- ISO/CD 22301 (2026 draft): https://www.iso.org/standard/93606.html
- ISO 22313:2020: https://www.iso.org/standard/75107.html
- NIST SP 800-34 Rev. 1: https://csrc.nist.gov/pubs/sp/800/34/r1/upd1/final
- NIST SP 800-57 Part 1 Rev. 5: https://csrc.nist.gov/pubs/sp/800/57/pt1/r5/final
- NIST SP 800-57 Part 1 Rev. 6 initial public draft: https://csrc.nist.gov/pubs/sp/800/57/pt1/r6/ipd

The NIST Rev. 6 document is a draft, not a final replacement. Standards inform the test design; this benchmark does not certify compliance.
