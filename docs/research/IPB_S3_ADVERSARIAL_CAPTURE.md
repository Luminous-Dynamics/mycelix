# IPB-S3 — Adversarial Institutional Capture

**Version:** 0.1 research fixture  
**Status:** Synthetic Rust implementation authored; hosted validation required  
**Parent:** LISE / IPB, Mycelix #6114  
**Evidence boundary:** The suite tests declared control graphs and attack fixtures. It does not estimate real-world capture prevalence, infer coalition membership, establish institutional legitimacy, or certify a live organization.

## 1. The research gap

IPB-S2 tests a bounded transaction-level conflict-of-interest control: can a conflicted actor authorize an expenditure? That is necessary but insufficient. An institution can pass every local transaction rule while one coalition controls the system's agenda, appointments, authorization, treasury custody, audit and appeal.

S3 changes the unit of attack from an individual decision to a **cross-domain control path**. It asks whether the same declared coalition can exercise every critical execution function while also disabling the independent oversight capacity required by the scenario's own constitution.

This is deliberately not a universal definition of capture. Each institution or benchmark scenario supplies its own constitutional domains, quorum sizes, required independent oversight and capture predicate. Those are policy inputs, not values the benchmark is entitled to choose.

## 2. Paradigm shift: distinguish three graphs

The research model separates three structures that conventional role/permission reviews often collapse:

1. **Formal authority graph:** who is authorized to perform which function, under what scope and threshold.
2. **Effective power graph:** who can actually assemble the decisive coalition, block decisions, control appointments or exercise pivotal influence under the observed availability conditions.
3. **Accountability graph:** who can independently audit, challenge, appeal, reverse, remedy or expose the action.

The S3 reference fixture represents the bounded authority/availability surface and the declared coalition. Symtropy #3684 remains the owner for voting-power indices, pivotality, minimal winning coalitions, role concentration and separation-of-duties analysis. Symthaea #3054/#3095 and Hostile Agent Bench #1872 remain the owners for adversarial agents, mechanism experiments and coalition search. S3 should consume those analyses as versioned evidence when the adapters are ready; it must not reimplement them.

A system can have distributed formal role assignments but concentrated effective power through correlated delegations, common appointment authority, quorum manipulation, or control of the information/appeal path. Conversely, concentrated execution responsibility does not automatically imply capture when audit and appeal remain independently staffed and operational.

## 3. Implemented S3 fixture semantics

The current fixture defines seven domains:

- agenda;
- appointments;
- authorization;
- treasury custody;
- audit;
- appeal;
- constitutional amendment.

The synthetic policy marks four domains as its critical execution path (agenda, appointments, authorization, treasury custody), and two as oversight functions (audit, appeal). It is a test policy, not a constitutional recommendation.

Each domain declares:
- authorized controllers at the scenario cutoff;
- which authorized controllers are available in the attack window;
- the number of distinct signers required to exercise the domain.

The scenario declares coalition membership as synthetic ground truth. For each domain, the evaluator records explicit scenario-local identifiers for the authorized set, available set, coalition-available set and independent-available set, as well as counts and quorum results. The receipt also carries the policy ID/version and sorted coalition-member identifiers so an independent reviewer can inspect the declared control topology. In any real evaluation, identifiers must be approved pseudonyms; personal identity data must not be published. This is a benchmark output projection, not a new identity store.

The current capture predicate is intentionally explicit:
- every critical execution domain can be exercised by the coalition; **and**
- at least the policy-configured number of oversight domains lack an independently available quorum.

Coalition access to an oversight function is reported separately from independence. If the coalition can meet the same configured quorum but an independent non-coalition group can also meet it, the oversight domain is **not** marked compromised by this bounded predicate. This avoids equating coalition capability with exclusive control. It does not yet model agenda capture inside oversight, coercion, information control, retaliation or common-cause dependencies; those require the held-out adversarial and dependency-closure extensions.

The disposition is one of:
- `capture_detected`;
- `no_capture_observed` (only within the declared fixture);
- `execution_unavailable` (critical quorum cannot be met).

Malformed input, missing domains, duplicate domains, unauthorized available controllers and invalid quorum thresholds return a typed input error. They do not become `no_capture_observed`.

## 4. Initial known-answer cases

The Rust suite currently includes nine deterministic fixtures:

| Fixture | Expected disposition | Why |
|---|---|---|
| Cross-domain coalition | Capture detected | Coalition controls all critical execution domains and both oversight domains |
| Single-actor role concentration | Capture detected | One actor meets every configured execution and oversight quorum |
| Quorum suppression | Capture detected | Independent controllers retain nominal authority but are unavailable at the scenario cutoff |
| Independent audit and appeal | No capture observed | Coalition controls execution, but independent oversight can independently satisfy its quorum |
| Dual independent oversight quorums | No capture observed | Coalition and independent controllers can each meet the oversight quorum; coalition capability alone is not exclusive capture |
| Fragmented execution control | No capture observed | Coalition cannot meet each critical execution quorum |
| Coalition below execution quorum | No capture observed | One coalition member cannot meet the two-person critical quorums |
| Oversight captured without execution control | No capture observed | Oversight control alone does not satisfy the declared full-capture predicate |
| Critical execution quorum unavailable | Execution unavailable | Governance paralysis is reported separately from capture and from resilience |

These are purposive tests, not independent samples. Their pass count must not be converted to a capture probability or a confidence interval about real institutions.

## 5. What this contributes beyond a conventional governance dashboard

Existing governance and GovTech systems provide important primitives: voting, proposals, participation, identity, workflow and audit records. Formal power theory already provides Banzhaf and Shapley–Shubik measures of pivotality under declared voting rules. LISE should not claim to invent those foundations.

The proposed advance is an **end-to-end adversarial assurance experiment** that joins:
- formal rules and authority holders;
- effective coalition power under availability and quorum conditions;
- independent oversight capacity;
- event-level execution and outcomes;
- bitemporal reconstruction and contestability;
- an independently replayable evidence receipt.

The benchmark should test the composition, not merely count features. A permissions dashboard cannot establish whether an independent appeal path is usable. A vote count cannot establish whether appointments and agenda control were captured. A signed credential cannot establish that the holder has sufficient authority or that the proposition is supported. A digital twin cannot create authority.

## 6. Experiment matrix and metrics

Next S3 extensions should vary, independently and in combination:

- coalition size and overlap across domains;
- formal quorum and amendment thresholds;
- delegation chains and common appointment authority;
- attendance/availability and strategic abstention;
- treasury custody and payment release;
- audit independence, appeal access and remedy power;
- information asymmetry and evidence withholding;
- role turnover, succession and revocation;
- time-varying coalitions and delayed feedback.

Report a profile, not a universal score:

| Metric | Definition |
|---|---|
| Capture predicate sensitivity | Share of known-answer capture fixtures correctly detected |
| Benign-control false-positive rate | Share of known-answer non-capture fixtures incorrectly labelled captured |
| Execution availability | Whether configured critical quorums can be met |
| Cross-domain concentration | Owner-native power-index output by domain and coalition |
| Oversight independence | Independent available controllers relative to each domain's required quorum |
| Capture lead time | First observable point at which the declared predicate becomes true in a longitudinal trace |
| Recovery time | Time to restore independent oversight after a seeded loss/capture event |
| Contestability | Share of actions for which an affected party can actually invoke the defined challenge path |
| Remedy efficacy | Whether a successful challenge produces the specified correction or remedy |
| Evidence completeness | Share of transitions with canonical authority, attempt, observation and outcome references |
| Participant burden | Time, cost and access barriers to oversight and appeal |

Do not use raw attendance, comment count, credential count or aggregate voting weight as a substitute for meaningful control. Report distribution and subgroup effects where real participants are involved.

## 7. Beating the state of the art: a fair test

The benchmark comparison must be implementation-neutral and matched on scenario, information, participant capabilities, budget, deadlines and threat model.

Compare at least:
1. manual governance with a documented constitution and role register;
2. conventional workflow/permissions and audit tooling;
3. voting or participatory-governance software where the scenario is within its scope;
4. a formal power-analysis-only baseline using Symtropy's declared power-index outputs;
5. a Mycelix owner-native lifecycle implementation with the LISE S2/S3 assurance adapters;
6. a hybrid human process with independent external audit and appeal.

A baseline should not be marked as failed because it lacks a feature it was never designed to provide. Missing telemetry is recorded as unavailable, not assumed safe or unsafe. Use independent evaluators, preregistered hypotheses, held-out attack profiles and published implementation details.

### Proposed benchmark gates

- 100% known-answer agreement on the frozen synthetic S3 suite.
- 0 false capture findings on the locked benign-control fixtures.
- 100% typed rejection of malformed/missing critical inputs.
- Permutation invariance under actor-label renaming.
- Dual-quorum control: coalition access must not imply oversight capture when independent non-coalition controllers can also satisfy the configured quorum.
- Monotonicity checks: adding coalition control must not make a previously detected capture disappear; restoring sufficient independent oversight must not create a capture finding when all other inputs remain fixed.
- Replay of every receipt from frozen policy and scenario inputs.
- At least three independent implementations or evaluators before any interoperability or comparative-performance claim.
- Separate exact-head format, locked tests, strict Clippy and independent replay evidence.

These are proposed engineering gates for the benchmark, not claims already achieved. The present eight-case fixture only establishes the first executable contract.

## 8. Research questions that could produce real advances

1. **Capture as a temporal path:** Can early-warning conditions be detected before any one vote crosses a conventional threshold?
2. **Power that survives organizational charts:** How do appointment authority, delegation, agenda access, shared infrastructure and information control alter formal pivotality?
3. **Oversight capacity as a resource:** How much independent reviewer availability, expertise and appeal throughput is needed to prevent nominal rights from becoming unusable?
4. **Constitution-aware mechanism design:** Can alternative quorum and separation-of-duties mechanisms reduce coalition capture without making the institution nonfunctional?
5. **Plural institutions:** Can federated institutions preserve local constitutional sovereignty while detecting cross-organization cartelization and common-cause dependencies?
6. **Longitudinal digital twins:** Can an ensemble predict held-out capture pathways better than simple role-concentration and threshold baselines? Failure to beat those baselines narrows the twin's permitted claim.

## 9. Integration and evidence boundaries

- Mycelix #2953 and #3117 own decision/authority/execution lifecycles.
- Mycelix #6069 owns bitemporal reconstruction and challenge/correction history.
- Mycelix #3126 owns contested/unknown evidence semantics.
- Mycelix #4835 owns independent evaluator/oracle separation.
- Mycelix #6113 owns LISE integration.
- Symtropy #3684 owns formal governance power and role-concentration indices.
- Symthaea #3054/#3095/#1872 own game-theoretic and hostile-agent experiments.

The next adapter must map each S3 fixture to these owner-native references, not copy their schemas. A passing synthetic fixture does not mean Mycelix's production zomes enforce it. No production policy is changed by this benchmark.

## 10. Selected current research context

- OECD, *Digital Government Outlook 2026*: digital interoperability systems exist in 30 of 36 OECD countries, but only 23 of those report widespread adoption across over half of central-government institutions; the report emphasizes that interoperability is a maintained system property, not simply a deployed connection.
- Parisi et al., *Real-Time Group Dynamics with LLM Facilitation* (FAccT 2026): two incentive-compatible studies (N=879) found no significant consensus improvement, while facilitator strategies shifted individual charity allocations by up to 5.5 percentage points and did not improve measured participation equity.
- Li et al., *Improving Group Deliberation through AI-Guided Self-Reflection* (arXiv, 27 October 2026): a randomized experiment (N=1,173; 391 groups) reported improved Discourse Quality Index under a non-directive pre-deliberation reflection condition (d=0.31 versus no reflection and d=0.27 versus unassisted reflection). This is a promising experimental arm, not evidence that AI facilitation in general improves governance.
- W3C, *Securing Verifiable Credentials using JOSE and COSE* (Recommendation, 13 October 2026): provides standardized credential presentation security mechanisms. It does not make a credential's proposition true or grant institutional authority by itself.

References:
- https://www.oecd.org/en/publications/digital-government-outlook_0496b2bc-en.html
- https://deepmind.google/research/publications/224297/
- https://arxiv.org/abs/2610.37862
- https://www.w3.org/TR/2026/REC-vc-jose-cose-20261013/


## 13. Common-cause dependency integration status (2026-11-13)

S3 v0 does **not** currently compute common-cause or transitive dependency independence. The owner-native work is tracked by Mycelix #5762/#5695/#1243/#1371, but the #5762 issue remains open and its implementation PR #5696 is also open, based on the `research/assurance-portfolio-model-v0-1` research branch rather than merged `main`. Therefore the required dependency-closure/cut-set output is not yet a stable mainline integration contract. Until the owner implementation is merged, reviewed, and independently qualified, any S3 claim that audit and appeal are operationally independent across common causes must be **indeterminate / not evaluated**. S3's present identity/quorum fixture only tests the explicitly declared controller sets; it cannot prove failure-domain independence.
