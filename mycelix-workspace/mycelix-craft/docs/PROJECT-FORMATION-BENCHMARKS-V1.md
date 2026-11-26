# Craft Project Formation V1 — Benchmark Plan

- **Work item:** CRAFT-PROJECT-CORE-001 (#6729)
- **Status:** Proposed benchmark protocol; no measured results
- **Version:** 1.0-draft
- **Date:** 2026-11-26

## 1. Research question

Can a need-first, evidence-aware team formation system help communities assemble safe, capable and consenting teams more effectively than vacancy matching, taxonomy matching or manual coordination, without silently excluding people because optional evidence is missing?

The benchmark tests the composition of project formation, contextual evidence and consent. It does not claim that the reference core by itself improves employment, worker welfare or community outcomes.

## 2. Matched baselines

Every baseline receives the same project need, work packages, candidate pool, available evidence and safety constraints:

1. keyword / lexical matching;
2. taxonomy-only capability matching;
3. CV and credential matching;
4. work-sample assessment;
5. manual expert team composition;
6. constrained optimization over declared capabilities and availability;
7. Craft requirement-level coverage with abstention and developmental pathways.

Use the same frozen fixtures and disclose any baseline that cannot represent consent, evidence uncertainty, safety requirements or plural institutions.

## 3. Primary measures

| Measure | Definition | Qualification target |
| --- | --- | --- |
| Safety false acceptance | Unsafe or unqualified assignments marked Covered | 0 in frozen adversarial corpus |
| Scope mismatch acceptance | Evidence accepted despite wrong subject, context, jurisdiction or policy | 0 |
| Consent violation | Withdrawn or non-consenting participant retained | 0 |
| Development substitution | Supervised learning treated as qualification | 0 |
| Distinct-contributor integrity | One person counted multiple times toward minimum coverage | 0 |
| Unknown-state integrity | Unknown evidence silently converted into qualified or failed capability | 0 |
| Explanation completeness | Output identifies the exact requirement, evidence state, scope and gap | 100% of frozen fixtures |
| Independent replay | Second implementation reproduces every frozen output | 100% |

Targets are qualification requirements, not measured results.

## 4. Matching quality and fairness measures

Measure precision@k, recall@k and nDCG@k only where a ranking baseline exists. For Craft's non-ranking coverage model, report requirement-level precision/recall, evidence coverage, abstention quality, false exclusion, review rate, uncovered requirements, safe development opportunities and team redundancy.

Run counterfactual tests that vary optional education labels, institution prestige, reputation, membership rank, honorifics and non-required credentials while holding actual task evidence constant. None may silently become a hard eligibility veto.

Measure subgroup differences only with participant consent, lawful purpose, minimum disclosure and a documented privacy protocol. Do not infer sensitive attributes to produce a fairness score.

## 5. Adversarial corpus

Freeze positive, negative and ambiguous cases for:

- exact-scope current qualified evidence;
- wrong capability vocabulary or profile revision;
- wrong work context or jurisdiction;
- wrong subject and duplicated evidence;
- stale, expired, revoked, unknown and disputed evidence;
- different assessment-policy revision or digest;
- missing and changed project terms digest;
- future consent and post-withdrawal participation;
- duplicate contributor assignments;
- one contributor counted twice against a multi-person threshold;
- mentor self-supervision or mentor not on the consenting team;
- supervision offered for a safety-critical requirement;
- partial coverage, conflicting evidence and unavailable reviewers.

Add generated/property tests for at least 100,000 cases before production qualification; fuzzing volume alone is not a correctness argument.

## 6. Team formation comparison

For optimization experiments, use constrained set-cover / integer-programming formulations as a transparent baseline. Hard constraints include consent, scope-valid evidence, safety, jurisdiction, availability and owner-defined resource constraints. Soft objectives may include travel, time, cost, redundancy and participant preferences.

Report the feasible set and Pareto trade-offs rather than a universal weighted score. Publish:
- requirement coverage by project and work package;
- unique qualified contributors per requirement;
- critical capability single points of failure;
- unknown and contested evidence;
- learning pathways available;
- allocation and coordination latency;
- number of human reviews and their resolution time.

Do not trade away hard safety or consent constraints to improve optimization objectives.

## 7. Field evaluation

The first proposed pilot remains a community water-pump / renewable-energy repair. Pre-register the outcome measures and compare against the community's normal coordination process.

Measure independently:
- time from accepted need to authorized, resourced project;
- time to qualified team formation;
- repair acceptance, rework and recurrence;
- water-service availability before and after repair;
- paid hours, compensation and unpaid hours;
- pre/post capability gain and delayed transfer to unseen tasks;
- mentor compensation and learner withdrawal/appeal;
- equipment utilization and maintenance;
- participant comprehension, consent and perceived control.

Use independent inspection and physical measurements. Do not count a signed work record, project closure or payment as proof that a service improved.

## 8. Reporting

Every report must include exact commit SHA, frozen dataset version, baseline implementation/version, random seed, toolchain, hardware, commands, raw outputs, exclusion rules and confidence intervals. Publish negative results. Separate synthetic conformance, independent reproduction, host integration and field evidence.

Never combine safety, productivity, fairness, autonomy, economic plurality and community benefit into a universal Craft score.
