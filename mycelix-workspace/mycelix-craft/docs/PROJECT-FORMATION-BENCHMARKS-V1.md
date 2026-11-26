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

## 3. State of the art and fair algorithm comparison

Craft must not claim that skill coverage, team formation or cost-aware optimization is itself novel. These are mature research topics:

- The 2025 ACM Computing Surveys review, *A Survey of Subgraph Optimization for Expert Team Formation*, covers skill coverage, proficiency, communication/geographic cost, availability, capacity and budget constraints. It also notes that network-density/proficiency objectives can disadvantage newcomers and less-connected experts.
- The ECML-PKDD 2025 *A QUBO Framework for Team Formation* gives a unified formulation of coverage and expert-cost trade-offs with QUBO solution methods.
- *Team formation in large organizations: A deep reinforcement learning approach* (Decision Support Systems, 2024) optimizes internal team formation while minimizing organizational impact.
- A 2025 social-network study compares greedy and genetic methods under communication-cost and workload-balance constraints.
- 2026 SIAM coverage of representative team design emphasizes the tension between complementary skills, cohesion, preferences and representation.

Craft's potential contribution is the **problem contract**, not an unsupported claim that its optimizer beats these methods: consent-bound offers; non-compensable safety and legal constraints; evidence identity, context, jurisdiction and validity; explicit abstention; development paths; institutional sovereignty; and no reliance on social-network centrality as a proxy for competence.

### Required algorithm baselines

Run the same frozen tasks and constraints against:

1. Greedy coverage and greedy multi-constraint variants.
2. Genetic algorithms, with fixed seed and declared population/generation budgets.
3. QUBO formulations and the selected solver/backend.
4. Deep reinforcement learning, where training data and policy are reproducible.
5. Social-subgraph optimization with communication-cost objectives.
6. Exact MILP/CP-SAT or exhaustive search on tractable instances to provide an optimality bound.
7. A human expert-team baseline and the normal community coordination process.

A baseline that cannot represent consent, mandatory safety, uncertainty, external institutions or developmental paths must be marked **not semantically equivalent**; do not silently simplify Craft's constraints to fit its objective function.

### Algorithmic metrics

Report feasible-solution rate, hard-constraint violations, coverage precision/recall, objective-specific Pareto fronts, optimality gap on small exact instances, runtime p50/p95/p99, peak memory, team size, skill redundancy, workload balance, coordination cost, re-formation latency after a no-show, and sensitivity to stale or missing evidence.

For social-graph methods, add cold-start and disconnected-network cohorts. A method that maximizes past collaboration density can systematically disadvantage newly arrived, remote or previously unconnected people; measure that failure explicitly rather than assuming network centrality is merit.

Do not combine these metrics into a universal optimizer score. Publish each objective and constraint separately.

### Industry and standards comparison

- **Upwork** already provides hourly/fixed-price contracts, funded milestones, review/approval and dispute assistance. Its published payment-protection terms describe work diaries for eligible hourly work, pre-funded fixed-price milestones, a review period and dispute assistance; coverage is conditional on the contract and submission rules. Craft should not build or claim escrow/payment protection as novel; Finance or a licensed provider remains the effect owner.
- **Eightfold** already documents internal project recommendations based on skills, individual growth goals and business needs. Its published product documentation describes skill-alignment weighting, analysis of up to 50 skills, profile-based skill augmentation, employee skill interests and configurable 1-to-5-star match thresholds. Its Talent Intelligence Platform also calibrates positions, stack-ranks candidates and can apply hard filters. Craft should not claim project recommendations, skill matching or learning-oriented matching as unique. The fair comparison is whether Craft's evidence scope/provenance, explicit Unknown/HumanReview states, participant-controlled disclosure and cross-institution project formation produce better safety, inclusion, explainability and actual outcomes under matched cases. Do not compare vendor claims as measured benchmark results.
- **W3C VC Data Model 2.0** and **Open Badges 3.0** already provide credential interoperability. Craft should reference them and preserve source claim limits, rather than invent a competing badge format.
- **ILO Recommendation 208** makes apprenticeship quality broader than skill progression: adequate remuneration, hours limits, leave, social-security access, occupational safety and health, injury compensation, complaints/dispute resolution, data protection and labour-market mobility are all relevant protections. The pilot must measure these, not treat them as optional HR outcomes.
- **ESCO/OFO**, **Valueflows** and **GS1 EPCIS 2.0** are useful optional mappings for occupations, economic processes and physical event visibility. Preserve source vocabulary/revision/digest and record lossy mappings.
- Cooperative ownership, Ostrom-inspired commons governance and Sociocracy 3.0 patterns are prior art. Craft's target is not to invent democracy or consent; it is to make autonomous rules and negotiated interoperation explicit, inspectable and interoperable.

### References

- ACM Computing Surveys (2025), *A Survey of Subgraph Optimization for Expert Team Formation*: https://doi.org/10.1145/3737455
- ECML-PKDD (2025), *A QUBO Framework for Team Formation*: https://doi.org/10.1007/978-3-032-05981-9_29
- Decision Support Systems (2024), *Team formation in large organizations: A deep reinforcement learning approach*: https://doi.org/10.1016/j.dss.2024.114343
- (2025), *A novel collaborative effectiveness-driven team formation strategy in social networks*: https://doi.org/10.1080/21642583.2025.2600786
- SIAM News (2026), *Can Artificial Intelligence Help Build Better, More Representative Teams?*: https://www.siam.org/publications/siam-news/articles/can-artificial-intelligence-help-build-better-more-representative-teams/
- Upwork Payment Protection: https://support.upwork.com/hc/en-us/articles/211062568-How-Upwork-protects-your-payments
- Eightfold Matching and Calibration (updated 2026-11-13): https://docs.eightfold.ai/en_US/talent-intelligence-platform/tipmatching-and-calibration
- Eightfold Project Recommendations (updated 2026-11-13): https://docs.eightfold.ai/en_US/learning-and-professional-growth/project-recommendations
- W3C VC Data Model 2.0: https://www.w3.org/TR/vc-data-model/
- 1EdTech Open Badges 3.0: https://www.imsglobal.org/spec/ob/v3p0/
- ESCO: https://esco.ec.europa.eu/en/use-esco/use-esco-services-api
- Valueflows 1.0: https://www.valueflo.ws/introduction/status/ (stable core for operational planning and observation of economic activity; do not assume removed or experimental agent/bioregional models are part of 1.0)
- GS1 EPCIS 2.0: https://www.gs1.org/standards/epcis
- ILO Recommendation 208: https://normlex.ilo.org/dyn/nrmlx_en/f?p=NORMLEXPUB:12100:0::NO::P12100_ILO_CODE:R208

## 4. Primary measures

| Measure | Definition | Qualification target |
| --- | --- | --- |
| Safety false acceptance | Unsafe or unqualified assignments marked Covered | 0 in frozen adversarial corpus |
| Scope mismatch acceptance | Evidence accepted despite wrong subject, context, jurisdiction or policy | 0 |
| Consent violation | Withdrawn or non-consenting participant retained | 0 |
| Development substitution | Supervised learning treated as qualification | 0 |
| Distinct-contributor integrity | One person counted multiple times toward minimum coverage | 0 |
| Unknown-state integrity | Unknown evidence silently converted into qualified or failed capability | 0 |
| Revocation-status freshness | Nominally valid evidence accepted after its status snapshot exceeds the requirement freshness bound | 0 |
| Future status timestamp | Future-dated issuer/revocation observation accepted | 0 |
| Explanation completeness | Output identifies the exact requirement, evidence state, scope and gap | 100% of frozen fixtures |
| Independent replay | Second implementation reproduces every frozen output | 100% |

Targets are qualification requirements, not measured results.

## 5. Matching quality and fairness measures

Measure precision@k, recall@k and nDCG@k only where a ranking baseline exists. For Craft's non-ranking coverage model, report requirement-level precision/recall, evidence coverage, abstention quality, false exclusion, review rate, uncovered requirements, safe development opportunities and team redundancy.

Run counterfactual tests that vary optional education labels, institution prestige, reputation, membership rank, honorifics and non-required credentials while holding actual task evidence constant. None may silently become a hard eligibility veto.

Measure subgroup differences only with participant consent, lawful purpose, minimum disclosure and a documented privacy protocol. Do not infer sensitive attributes to produce a fairness score.

## 6. Adversarial corpus

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

## 7. Team formation comparison

For optimization experiments, use constrained set-cover / integer-programming formulations as a transparent baseline. Hard constraints include consent, scope-valid evidence, safety, jurisdiction, availability and owner-defined resource constraints. Soft objectives may include travel, time, cost, redundancy and participant preferences.

Report the feasible set and Pareto trade-offs rather than a universal weighted score. Publish:
- requirement coverage by project and work package;
- unique qualified contributors per requirement;
- critical capability single points of failure;
- unknown and contested evidence;
- learning pathways available;
- allocation and coordination latency;
- number of human reviews and their resolution time;
- requirement-gate semantics: SafetyCritical and Required gaps block the package; Developmental and Preferred gaps remain visible but do not silently veto project formation.

Do not trade away hard safety or consent constraints to improve optimization objectives.

## 8. Field evaluation

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

For any apprenticeship pathway, translate ILO Recommendation 208 into observable pilot safeguards: a written agreement specifying the parties' roles and obligations; adequate remuneration that can progress with acquired competence; lawful work-hour limits; applicable leave and social-security access; occupational safety and health training/protection; work-injury compensation; complaints and dispute resolution; personal-data protection; and no post-apprenticeship restriction on labour-market mobility. Track failures and withdrawals as outcomes, not as missing data to be discarded. Reference: https://normlex.ilo.org/dyn/nrmlx_en/f?p=NORMLEXPUB:12100:0::NO::P12100_ILO_CODE:R208.


## 9. Reporting

Every report must include exact commit SHA, frozen dataset version, baseline implementation/version, random seed, toolchain, hardware, commands, raw outputs, exclusion rules and confidence intervals. Publish negative results. Separate synthetic conformance, independent reproduction, host integration and field evidence.

Never combine safety, productivity, fairness, autonomy, economic plurality and community benefit into a universal Craft score.
