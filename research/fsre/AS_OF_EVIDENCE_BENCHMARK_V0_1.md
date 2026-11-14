# FSRE as-of evidence admissibility benchmark v0.1

**Date:** 2026-11-14  
**Status:** Research protocol with a synthetic production/reference implementation. No source authentication, operational deployment, field result or superiority claim is made.

## 1. Research question

Can a recovery claim be reproduced using only evidence that the responsible decision process could actually have known at the decision time, while distinguishing what was true in the world, what was recorded, what was available, and what was learned later?

A snapshot of today's database is not a historical decision record. A source may describe a valid fact that had not yet reached the operator. A record may have been visible but already outside its valid interval. A revocation may be effective in the world but learned by the operator only after the decision. Treating these as the same thing creates hindsight leakage and can make a recovery option appear qualified when it was not.

FSRE should add a deterministic **as-known-at-the-cutoff** evidence admissibility profile above owner-native evidence systems. It should not create a competing evidence ledger or temporal database.

## 2. Prior-art boundary: reuse, do not reinvent

Time and provenance semantics are established:

- **Bitemporal data systems** distinguish *valid/application time* (when a fact is true in the represented world) from *system/transaction time* (when the database records a version). These are orthogonal timelines; a retrospective correction must not erase the previous database assertion.
- **OGC API - Records, Part 1: Core** recognises temporal instants and intervals and discusses multiple temporal properties, including valid time and transaction time. It also identifies observation-related temporal characteristics such as phenomenon time, result time and valid time in relevant standards.
- **W3C PROV-DM / PROV-N** model provenance through entities, activities, agents, use, generation, attribution and derivation. Provenance describes how a result came about; it does not, by itself, decide whether a particular evidence item was available to a particular operator at a historical decision cutoff.
- **SQL:2011-style temporal support and vendor temporal tables** provide useful mechanisms for querying historical database states. They do not automatically define the service-specific freshness policy, source availability to an external decision-maker, scenario scope, revocation semantics or claim threshold needed by a federated resilience evaluation.

The contribution hypothesis is therefore narrower: combine owner-native provenance and temporal records with an explicit decision-role availability time, a claim-specific freshness policy, half-open validity intervals, revocation effective/knowledge times, and independently reproducible claim dispositions.

References:
- OGC API - Records - Part 1: Core, §7.2.7 Temporal information: https://docs.ogc.org/is/20-004r1/20-004r1.html
- W3C PROV-DM: https://www.w3.org/TR/prov-dm/
- W3C PROV-N: https://www.w3.org/TR/prov-n/
- Microsoft SQL Server temporal tables overview (implementation example, not the SQL standard): https://learn.microsoft.com/en-us/sql/relational-databases/tables/temporal/overview
- PostgreSQL wiki summary of SQL:2011 valid/application time and system/transaction time: https://wiki.postgresql.org/wiki/SQL2011Temporal

## 3. Temporal contract

Every replay freezes a scenario, a decision cutoff and a declared clock basis. Each evidence record retains the following distinct times:

| Field | Meaning | Common failure if collapsed |
|---|---|---|
| `observed_at` | When the phenomenon or state was measured | Future observations leaked into past decisions |
| `available_at` | When this evidence became accessible to the declared decision process | Delayed publication treated as if known immediately |
| `valid_from` | Inclusive start of the world-time interval asserted by the record | Future-effective permit or capacity treated as active too early |
| `valid_until` | Exclusive end of the asserted validity interval | Expired evidence treated as current at the boundary |
| `revoked_effective_at` | When the source says the evidence ceased to apply | World-time invalidation missed |
| `revocation_known_at` | When the revocation became available to the decision process | Later correction silently rewrites what was knowable earlier |

Use half-open valid intervals `[valid_from, valid_until)`. If no upper bound is declared, the interval is open-ended only where the owner-native schema permits it. All compared timestamps must share an explicit clock/time basis; incomparable clock domains are rejected rather than converted by assumption.

Freshness is claim-specific. A telemetry observation, chemical stock count, laboratory result, crew roster and authority grant have different useful lifetimes. A run-level default may exist, but the claim owner must be able to override it and the effective threshold must be present in the replay report.

A claim declares a minimum number of distinct admissible source references. Duplicate references cannot be counted twice. The minimum is a contract field, not a confidence score.

## 4. As-known versus hindsight semantics

For a frozen cutoff (T):

1. An observation with `observed_at > T` is `FUTURE_OBSERVATION`.
2. An observation available only after (T) is `NOT_YET_AVAILABLE`, even when it describes an earlier event.
3. Evidence whose `valid_from > T` is `NOT_YET_VALID`.
4. Evidence with `valid_until <= T` is `EXPIRED`.
5. Evidence older than its claim-specific freshness limit is `STALE`.
6. A revocation both known and effective by (T) is `REVOKED_AS_KNOWN`.
7. A revocation known by (T) but effective after (T) is `REVOCATION_PENDING`; the original evidence remains admissible only until its effective time.
8. A revocation effective by (T), but learned after (T), is reported as `ADMISSIBLE_WITH_HINDSIGHT_REVOCATION` for the historical replay, with a separate hindsight marker. This means the decision process could have relied on the evidence as known then; it does not establish that the evidence was objectively correct.
9. Missing or explicitly unknown required evidence contributes no admissible record. If the claim's minimum is not met, the claim is `NOT_ESTIMABLE`.

The profile never changes source records, backdates availability, or uses future observations to repair a past decision. It preserves both the historical decision-time evidence state and subsequent corrections.

## 5. Benchmark challenge families

The open synthetic corpus should cover:

1. **Future observation:** observation occurs after the cutoff.
2. **Delayed availability:** observation predates the cutoff, but publication/availability follows it.
3. **Future validity:** a permit or capacity certificate is available now but becomes valid later.
4. **Validity-end boundary:** `valid_until == decision_at`; the evidence is expired because the end is exclusive.
5. **Freshness boundary:** age exactly at the allowed maximum is accepted; one second older is stale.
6. **Claim-specific freshness:** the same observation is admissible for a long-lived inventory claim but stale for a short-lived operational state, under separately declared policies.
7. **Known revocation:** revocation effective and known by the cutoff blocks evidence.
8. **Retrospective revocation:** effective before the cutoff but learned later; preserve the as-known result and separately disclose hindsight.
9. **Future-effective revocation:** notice is known before the cutoff but not yet effective.
10. **Clock disagreement:** source evidence uses an incompatible time basis; fail closed.
11. **Scenario crossover:** evidence from another scenario cannot qualify this claim.
12. **Duplicate evidence reference:** the same source item cannot satisfy multiple required evidence slots by being copied under different record IDs.
13. **No evidence / unknown evidence:** claim requirement remains visible and not estimable.
14. **Row-order permutation:** the canonical report is independent of input order.
15. **Correction sequence:** a later source correction creates a new evidence version; the historical cutoff remains reproducible while the later cutoff sees the correction.

A sealed challenge set should be prepared by a second reviewer and should include boundary combinations not present in the public fixture.

## 6. Matched baselines and ablations

Compare under an identical frozen evidence timeline, scenario and decision cutoff:

- B0: current operator workflow / control-room record at the time;
- B1: naive current-snapshot evaluation (hindsight-prone baseline);
- B2: source-native bitemporal query where the underlying source supports it;
- B3: provenance graph/query using owner-native PROV-compatible lineage;
- B4: temporal catalogue query using OGC-style temporal record fields;
- B5: FSRE as-of evidence profile with claim-specific policies and independent oracle;
- ablations: remove availability time, valid time, revocation knowledge time, claim-specific freshness, scenario binding, or source-reference deduplication.

A comparator should be credited for native temporal capabilities it already provides. FSRE must not claim that incumbents lack bitemporal history or audit trails without inspecting the deployed configuration and running the matched task.

## 7. Proposed qualification gates

For a frozen deterministic challenge suite:

- 0 future observations or future-available records accepted at an earlier cutoff;
- 0 evidence with validity ending at or before the cutoff accepted as current;
- 0 revocations known and effective by the cutoff accepted;
- 100% exact boundary agreement with an independently authored reference;
- 100% source owner/reference preservation in the output;
- 0 duplicated source references counted twice for one claim;
- 100% explicit effective freshness policy per claim;
- 100% unknown or insufficient-evidence claims remain `NOT_ESTIMABLE`;
- 100% row-order invariance and deterministic replay for the same frozen inputs.

These are conformance targets, not a real-world false-qualification rate. Operational evaluation must add independent source authentication, clock-quality assessment, actual decision logs, operator review, sealed holdouts and post-event outcome verification.

## 7.1 Claim-envelope interoperability

The v0.2 claim envelope carries decision-process availability, valid-time start/end, revocation effective/knowledge times, an immutable `evidence_snapshot_id`, and the as-of report reference. The [as-of evidence crosswalk](schemas/as-of-evidence-envelope-crosswalk-v0.2.json) maps these fields without promoting admissibility into source authenticity or claim truth.

The new [as-of portfolio composer](../../tools/fsre-asof-portfolio-composer/README.md) is the first executable composition path for the richer temporal profile. It requires exact equality of scenario, evidence snapshot, decision cutoff and clock basis, then checks every portfolio owner/reference against both the evidence-row disposition and the enclosing claim's minimum evidence count. The composer is a software-level gate over synthetic receipts; it does not authenticate their sources, establish physical truth or validate a full multi-interval plan.

One boundary remains explicit: the v0.1 evaluator compares exact UTC seconds and does not propagate non-zero clock uncertainty. The envelope records whether clock uncertainty was considered; records with non-zero or unresolved clock bounds cannot be presented as exact-time qualified by this evaluator.

## 8. Integration boundary and next experiments

The current as-of reference, independent oracle and portfolio composer reproduce a synthetic, point-in-time evidence-to-portfolio link. The next qualification step is an independently authored, disjoint challenge set covering delayed publication, retrospective corrections, claim-level evidence thresholds, ambiguous owner/reference bindings, evidence snapshots and clock uncertainty.

Only after lawful historical source snapshots can be replayed should the profile move from synthetic conformance (M1a) to retrospective real-data evaluation (M1b), connected shadow trials (M2) and operator decision support (M3). M4 bounded closed-loop operation requires a separate authority and safety case, fail-safe behavior, override, drift/security monitoring, auditability and prospective outcomes. The as-of composer provides none of those controls and does not itself qualify an M4 deployment.

The research contribution to test is: **can heterogeneous recovery claims be reproduced without hindsight leakage, with claim-specific temporal policies and independently verifiable evidence lineage, while leaving source ownership and operational authority with their incumbents?**
