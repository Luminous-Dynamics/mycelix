# IPB-S5 — Authority-preserving continuity and succession

**Status:** Draft v0.1  
**Tracking issue:** Mycelix #6125  
**Manifest:** `IPB_S5_AUTHORITY_PRESERVING_CONTINUITY.yaml`  
**Claim ceiling:** Synthetic benchmark specification; not production DR qualification, certification or field effectiveness.

## 1. Research question

Can an institution recover critical functions after losing or compromising a founder, administrator, signing key, provider, custodian or evidence source **without automatically transferring the former operator's power to the recovery operator**?

This extends ordinary service continuity into institutional continuity. A restored database or healthy service is not sufficient if the restored system has stale authority, no valid successor, no independent oversight, or no path to challenge the recovery.

## 2. Paradigm shift: continuity of function without continuity of power

Separate four state dimensions:

1. **Service state:** available, degraded, paused or unavailable.
2. **Authority state:** current, expired, revoked, contested or missing.
3. **Evidence state:** complete, partial, stale, contradictory or unavailable.
4. **Remedy state:** accessible, degraded, blocked or unresolved.

The system should support *selective continuity*: read-only and low-risk functions may continue while high-impact mutation capabilities remain paused pending current authority or evidence. This is preferable to an institution-wide fail-open/fail-closed switch, which either creates unsafe authority or unnecessarily disables all service.

The recovery path itself is a privileged operation and must be bounded by existing local rules. A recovery operator should never receive a universal superuser role merely because the incumbent is unavailable.

## 3. Reference institution and profile

The machine-readable manifest defines:
- synthetic member institution A;
- current constitution and policy versions;
- governing council, key custodians, continuity operator, independent reviewer, primary and backup providers;
- explicit 2-of-3 recovery threshold;
- emergency authority with a one-hour synthetic maximum and ordinary-governance renewal;
- service-specific RTO/RPO targets.

These numeric values are **scenario parameters**, not universal resilience requirements:
- member-service-read: 30-minute RTO, 5-minute RPO, 25% degraded capacity;
- treasury-outbound: 4-hour RTO, zero RPO for the decision/operation record, no degraded mutation;
- governance-record-read: 30-minute RTO, zero RPO;
- policy amendment: 24-hour RTO, zero RPO.

A result is measured against these predeclared values and reports misses. It must not silently relax the target after a failure.

## 4. Fifteen-case benchmark panel

| ID | Fault/attack | Expected disposition |
|---|---|---|
| S5-01 | Authorized normal succession | Recovered under current authority |
| S5-02 | Founder/administrator unavailable | Recovered through approved multi-party succession |
| S5-03 | One signing key lost | Key recovery succeeds without scope expansion |
| S5-04 | Compromised key replayed after revocation | Stale/revoked authority rejected |
| S5-05 | Primary service provider unavailable | Degraded but safely operating through approved provider |
| S5-06 | Stale backup contains old policy/authority | Recovery incomplete; high-impact capabilities paused |
| S5-07 | Recovery operator appoints itself | Unauthorized recovery rejected |
| S5-08 | Recovery quorum unavailable | Affected capabilities paused; read-only service may continue |
| S5-09 | Apparent recovery custodians share an unresolved control domain | Independence indeterminate |
| S5-10 | Emergency authority reaches sunset | Temporary scope expires; renewal requires ordinary process |
| S5-11 | Provider replacement | New provider accepted; evidence and remedy references preserved |
| S5-12 | Recovery snapshot/dependency inventory incomplete | Recovery incomplete and indeterminate |
| S5-13 | In-flight payment may have executed before timeout | Reconciliation required; no blind retry/new mutation |
| S5-14 | Approved institutional dissolution | New work stops; obligations and remedy records preserved |
| S5-15 | Sole knowledge custodian departs | Independent successor reconstructs current obligations and recovery |

The YAML manifest is the authoritative case identifier source. The fixture contains no personal data, real keys or live payments.

## 5. State machine and non-compensatory gates

Do not return a single `recovered=true`.

A useful evaluation object must preserve the separate state dimensions and per-function outcomes. Example:

| Function | Service state | Authority state | Evidence state | Permitted next action |
|---|---|---|---|---|
| Member read service | Degraded/available | Current | Complete enough | Continue read-only |
| Treasury outbound | Paused | Unknown | Partial | Reconcile, no mutation |
| Governance record read | Available | Not applicable to read | Complete | Continue read-only |
| Policy amendment | Paused | Successor quorum missing | Incomplete | No amendment |

The exact representation must use Mycelix owner-native types. This table is explanatory, not a new schema.

Hard gates:
- zero consequential action without current local authority;
- no revoked/stale credential can authorize new work;
- no self-appointed successor or recovery superuser;
- no unsafe mutation when the prior operation result is unknown;
- no independent-review claim on incomplete dependency closure;
- no incomplete backup promoted to a complete reconstruction;
- emergency privileges expire or receive an ordinary constitutional renewal;
- service restoration must be independently observed, not just self-attested by the recovery operator.

These rights/authority gates are non-compensatory. Excellent RTO cannot compensate for unauthorized action.

## 6. Metrics

### Service continuity
- RTO and RPO achievement for each declared function;
- minimum degraded capacity and duration;
- data loss, reconstruction latency and service debt;
- provider migration time and cost;
- full time-to-safe-operation, distinct from time-to-online.

### Authority and security
- unauthorized actions during recovery;
- stale/revoked assertion acceptance;
- authority/role expansion;
- concentration of recovery and signing power;
- emergency authority exposure duration;
- time from compromise detection to effective revocation.

### Evidence and accountability
- percentage of recovery actions reconstructable from current authority, policy, attempt, observation and outcome references;
- policy/version mismatch detection;
- known versus unknown backup/dependency counts;
- continued independent audit, appeal and remedy access;
- participant and beneficiary burden during recovery.

Report denominators, per-function timing, misses and uncertainty. Do not average an authority violation into a better RTO or a higher service-availability percentage.

## 7. Matched baselines and current industry context

S5 must compare competent implementations of:
- manual continuity and succession with independent review;
- conventional enterprise IAM, key management, backup/restore and DR workflows;
- managed cloud/provider failover and key recovery;
- ISO 22301-aligned business continuity management;
- the composed Mycelix owner-native lifecycle and authority/evidence contracts.

A system is not failed for lacking a feature outside its design. Report `not_supported` / `not_measured` explicitly.

**Standards context as of 13 November 2026:**
- ISO 22301:2019 with Amendment 1:2024 is the published baseline. ISO/CD 22301 Edition 3 is under development and will replace the 2019 edition; the committee draft is not a final standard.
- ISO 22313:2020 guidance was confirmed in 2025; revision work is underway.
- NIST SP 800-34 Rev. 1 remains published contingency-planning guidance.
- NIST IR 8587 was finalized 15 September 2026 and provides current guidance on token/assertion forgery, theft, misuse, key management and lifecycle for federation/API/cloud scenarios.

The benchmark is not an ISO or NIST compliance audit. It adds institutional authority, succession, remedy and rights continuity to those continuity/security foundations.

## 8. Owner-native integration boundary

Use, do not duplicate:
- #2953 for execution attempts, current authority/capabilities and reconciliation;
- #3117 for governance decision, authorization, implementation and review;
- #6069 for bitemporal reconstruction, correction, challenge and appeal;
- #3126 for unknown and contested evidence;
- #4835 for independent evaluator/oracle separation;
- #5762/#5695/#1243/#1371 for correlation and common-cause dependencies;
- #6113 for owner-native integration.

Until the common-cause owner contract is mainline and independently qualified, S5-09 must remain indeterminate. Until the execution reconciliation owner contract is integrated, S5-13 must remain an expected specification outcome, not an implemented capability.

No parallel IAM, identity provider, key custody, authority, event store, evidence graph or succession registry should be created.

## 9. What could make this state of the art

S5 can become more than a disaster-recovery checklist by making **recoverability a continuously testable institutional property**:
- continuously re-evaluate succession coverage when people, providers, policies or dependencies change;
- test authority after recovery, not just service uptime;
- evaluate safe degradation per capability rather than institution-wide;
- measure whether key/authority recovery is independent of the system it is intended to recover;
- test if records and remedies survive the exit of the party that operated or hosted them;
- use fault injection to remove one role/provider/key/dependency at a time, then exercise selected correlated failures.

A stronger S5 benchmark should include hidden, held-out recovery paths and compare a recovery plan's *predicted* RTO/RPO and authority outcomes to actual injected-exercise results. Miscalibration becomes a metric. Simulated recovery does not count as an observed service restore.

## 10. Qualification protocol

- Freeze scenario, policy, fault injections, target RTO/RPO and primary outcomes before implementation tuning.
- Match service, records, staff, recovery budget and attacker capability across baselines.
- Keep development and held-out fault sequences separate.
- Have an evaluator independent of the recovery operator verify authority, service outcome and record continuity.
- Publish negative controls, missed targets, unresolved dependencies and recovery failures.
- Keep software conformance, tabletop exercises, fault-injection exercises and field observations as separate evidence tiers.
- A field pilot requires consent, privacy minimization and an explicit non-disruption safety plan.

No S5 performance result exists until an independently evaluated run has been executed.
