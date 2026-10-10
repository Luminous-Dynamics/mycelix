# RFC: Scientific Due Diligence and Resource Allocation

**ID:** DESCI-SCI-ALLOCATION-001  
**Status:** Proposed  
**Target:** `mycelix-desci` canonical refoundation  
**Last updated:** 2026-10-10

## 1. Summary

This RFC proposes an evidence-grounded workflow connecting scientific questions, claims, evidence, assessments, resource requests, funding decisions, and observed outcomes. Symthaea may discover, summarize, challenge, and plan; an independently qualified evaluator assesses the evidence; Mycelix records signed events, authority, provenance, and policy-bound decisions; an authorized funding body retains control over commitments.

The objective is not to automate truth or generate a universal researcher/proposal score. It is to reduce the cost of finding and checking evidence, make rationales inspectable by non-experts, allocate scarce research resources against explicit objectives and constraints, and learn from independently assessed outcomes.

This is a design proposal. It does not imply the workflow or acceptance tests already exist or have passed.

## 2. Architectural fit

The current canonical DeSci path distinguishes a client-signed scientific event, actor/key/role authorization, an authority receipt, an append-only event stream, a deterministic projection, and a versioned evidence assessment. Legacy mutable claims and E0–E4 verification counts are not the authoritative scientific record. This RFC builds on that direction; it does not restore automatic tier promotion.

Relevant project documents:
- [Canonical Scientific Event API](CANONICAL_EVENT_API.md)
- [Scientific Authority Receipts](AUTHORITY_RECEIPTS.md)
- [Documentation index and legacy-document boundary](README.md)

Preserve these separations:

```text
authentic artifact != scientifically valid conclusion
signed contribution != independent corroboration
assessment != funding authority
funding commitment != disbursement or experiment performed
project completion != scientific or social impact
```

## 2.1 Source inventory and implementation implications (2026-10-10)

Static source inspection (not a fresh build or runtime qualification) found these reusable primitives and gaps:

- §scientific_events.rs§ defines canonical payloads for claim proposal/import, evidence attachment, attestations, corrections/withdrawals, supersession, and retraction. The inspected enum does not yet define first-class resource requests, allocation decisions, funding commitments, milestones, or outcomes.
- §EvidenceProfile§ already separates counts for artifacts, reviews, reproductions/replications, non-supporting results, critiques, conflicts, corrections, and withdrawals. §EvidenceAssessment§ carries policy ID/version, maturity, contestation, and reasons. That is a useful baseline, but it is not yet the multidimensional due-diligence report or portfolio decision record proposed here.
- The current §source_key§ uses organization identity when present and actor identity otherwise for replication/reproduction counting, while review counting uses actor identity. This helps deduplicate some reports but is not a full independence model. A person represented under multiple organizations may produce distinct source keys where otherwise-distinct attestations are admissible; multiple people in one organization can still be dependent. Before funding decisions consume such counts, qualify dependence across actor, organization, dataset, protocol, lab, and method.
- A follow-up audit of the event replay path found a more fundamental study-level gap: attestation validation checks that referenced evidence IDs exist in the event stream, but the projection stores attached artifacts as a bare list without preserving the attaching actor/organization or a study/collection identity beside each artifact. The IndependentReplication attestation kind therefore does not, by itself, establish that referenced evidence represents newly collected data rather than reuse of the original data. Artifact existence, a different uploader, a distinct organization, a different protocol label, and a different content hash are each insufficient on their own to prove independent collection. PR [#4953](https://github.com/Luminous-Dynamics/mycelix/pull/4953) narrows actor/source-attribution inflation, but explicitly does not close this gap. Before using replication counts in scientific or funding recommendations, add a versioned evidence-provenance/collection contract and test at least: original-data reuse mislabeled as replication; a copied artifact under a new ID; genuinely distinct collection; rerunning analysis on the same data; and partially shared datasets. Preserve uncertainty when a domain cannot support a definitive independence judgment.
- Related modules exist for citation metrics (§citation.rs§), Bayesian belief propagation (§bayesian.rs§), replication tracking (§reproducibility.rs§), disputes (§dispute.rs§), inference (§inference.rs§), and meta-claims (§meta.rs§). Their existence alone does not prove integration with the canonical path or scientific qualification; inspect call paths and tests before reuse.
- The legacy type layer contains E0–E4 and a unified confidence score. The documented import boundary correctly says historical tiers/counts are not validated evidence. The allocator must not silently consume legacy confidence or citation-influence fields.

**Implementation consequence:** start with a traceable mapping and adversarial qualification of existing evidence-independence semantics and assessment disposition behavior. Design resource-request, decision, milestone, and outcome event families only after avoiding duplicate schema/authority primitives.

## 3. Design principles

1. **Evidence before score.** Every material assessment finding references evidence, a source version, and an explicit reasoning rule. A numerical score cannot substitute for the evidence trail.
2. **Atomic claims.** Assess claims at the narrowest useful scope. Do not assign one confidence value to an entire paper, person, lab, or discipline.
3. **Visible uncertainty.** Preserve support, contradiction, unresolved questions, missing evidence, and assessor disagreement. Abstention is valid.
4. **Independent evaluation.** Proposal-generating systems cannot be their own authoritative validators. Reused sources, shared datasets, common labs, and correlated methods must be represented when assessing independence.
5. **Fail closed on authority and integrity.** Missing, stale, conflicting, or unverifiable authority/evidence remains indeterminate or rejected under a versioned policy; it does not become PASS by default.
6. **Human accountability.** Material allocation decisions are approved by a named authority under a declared policy. Automated recommendations remain advisory until a separately governed authorization profile explicitly permits a bounded action.
7. **Interoperability over reinvention.** Reuse persistent identifiers, FAIR metadata, and established provenance vocabularies where suitable. Domain concepts must be versioned and mapped explicitly.
8. **Outcome-aware but not outcome-naive.** Track negative and null findings, failures, replication, corrections, adoption, and harms. Do not equate publication, citations, or funding with correctness or value.
9. **Reproducible decisions.** Rebuild assessment projections from immutable input events and a pinned assessment policy/version. Corrections create new evidence and projections; they do not erase history.
10. **Proportionate process.** Automate mechanical checks and administrative compliance where safe, reserving expert attention for scientific judgments that require it.

## 4. Proposed domain model

The following are conceptual object/event families, not a statement that these schemas or event types already exist. Before implementation, map each item to the current canonical event envelope, schema-versioning rules, authority policy, and projection APIs.

### 4.1 Research question and project

A research question records a stable ID, statement, scope, domain, time horizon, intended beneficiaries, known constraints and assumptions, current candidate explanations, evidence gaps, and decision-relevant success/falsification criteria.

A versioned project references one or more questions and records the method, dependencies, roles, resource requests, budget/time ranges, risks, milestones, pre-registration where appropriate, and decision points.

### 4.2 Claim and evidence artifact

A claim records a precise proposition, scope, assumptions, and relationships to parent or derived claims. Evidence artifacts record persistent identifiers or content digests, licensing/access conditions, provenance, capture/version details, method, and the exact claims to which the artifact is relevant.

Evidence relationships should distinguish:
- supports;
- contradicts;
- qualifies or limits scope;
- replicates;
- fails to replicate;
- provides method/background context;
- corrects, withdraws, or supersedes.

Do not infer support from citation proximity or an abstract alone. Where feasible, bind relevant passages, table/figure identifiers, data, analysis outputs, or independently reproduced results.

For the initial cross-domain profile, keep computational reproducibility distinct from replicability. The National Academies' 2019 report defines computational reproducibility around the same input data, computational steps, methods, code, and analysis conditions; replicability concerns consistent results across studies that collected their own data. A replication attestation must therefore point to its own data or new experimental observations where applicable, rather than only re-signing the original artifact. Terminology can vary by discipline, so the active domain profile must state its definitions. A non-replication is an outcome to interpret against uncertainty and methodological quality, not automatic proof of misconduct or falsity.

### 4.3 Assessment profile

An assessment is a versioned, reproducible projection over exact evidence inputs and a named policy. Report separate dimensions as applicable:

- source integrity and provenance;
- claim-evidence fit;
- methodological rigor and controls;
- statistical or formal validity;
- reproducibility of computation or experiment;
- independent corroboration and evidence dependence;
- external validity and scope limits;
- conflicts of interest, ethics, privacy, security, and safety;
- unresolved uncertainty and evidence gaps.

Each finding includes disposition, rationale, relevance to the decision, evidence references, source/version, evaluator identity/type, policy/model/tool version where relevant, and steps required to resolve it.

No universal composite confidence or quality score is required. If a funding program uses a composite, its objective, weights, constraints, normalization, missing-data behavior, uncertainty treatment, and version must be declared and sensitivity-tested. Legal, ethics, safety, and integrity constraints must not be silently compensated for by a high impact estimate.

### 4.4 Resource request and decision

A resource request binds the exact project version and questions; amount/range, currency/unit and time profile; requested non-cash resources; alternatives considered (pilot-first, replication-first, defer, decline); major uncertainties and the cheapest feasible discriminating experiment; milestones, deliverables, acceptance criteria and stop/redirect/expand conditions; conflicts; related commitments; and shared dependencies or portfolio correlations.

A decision binds the exact request, assessment version, allocation policy version, eligible decision-makers, conflict declarations, deliberation record, chosen alternative, conditions, rationale, and authority receipt. A decision alone does not establish funds were reserved, transferred, or spent.

### 4.5 Outcome and learning record

Outcome records distinguish planned from observed measures and include protocol deviations, negative/null results, missingness, corrections, replication attempts, implementation/adoption, cost/time variance, and known harms. Causal claims about impact must be separated from temporal association and must name the identification assumptions and evidence supporting them.

## 5. Due diligence workflow

1. **Intake and identity:** authenticate submitter and organization, register the immutable project version, declare data-access/confidentiality constraints, and resolve conflicts of interest.
2. **Decompose:** turn the proposal into atomic claims, assumptions, milestones, and decision-relevant uncertainties. Keep a human-readable version beside machine-readable records.
3. **Retrieve:** search literature and structured sources through adapters. Preserve query, retrieval time, source identifier, version, and access/license constraints. Candidate sources are not evidence until resolved and inspected.
4. **Construct the evidence graph:** deduplicate artifacts, map claim support and contradiction, resolve corrections/retractions, and mark evidence-dependence groups. Ten reports repeating one press release do not count as ten independent confirmations.
5. **Challenge:** produce a structured critique covering alternatives, missing controls, baseline quality, statistical power/uncertainty, assumptions, feasibility, replication, and plausible failure modes.
6. **Independent check:** use deterministic checkers and a distinct evaluator path for identity, signatures, source resolution, executable arithmetic/statistics where possible, protocol completeness, and policy rules. Escalate out-of-scope judgments to qualified experts.
7. **Plan next information:** compare direct funding with a pilot, replication, measurement upgrade, or defer/decline alternative. Estimate whether each test could change the decision.
8. **Recommend, do not authorize:** issue a versioned assessment and recommendation with uncertainty, counterevidence, and explicit reasons. Funding authorization remains with the configured authority.
9. **Commit and track:** record approved amount/resources and milestone policy. Disbursement or execution, if connected later, requires its own narrowly scoped authority and idempotency model.
10. **Observe and update:** append outcome/correction events, rebuild projections deterministically, and evaluate calibration prospectively without retroactively rewriting the original decision.

## 6. Resource allocation method

Avoid ranking every proposal with one opaque score. First enforce eligibility and non-compensatory constraints, then expose trade-offs across objectives and propose portfolios under real resource constraints.

For each proposal, keep explicit distributions or ranges for:
- probability of achieving the stated result;
- impact if achieved, by declared dimension and time horizon;
- option value: follow-on work or capabilities the result could unlock;
- information value, including informative negative results;
- monetary, compute, equipment, researcher-time, and delay costs;
- harms, externalities, irreversibility, and opportunity costs.

These quantities may be uncertain, incomparable, or unsupported. Preserve that state rather than fabricating precision.

### 6.1 Value of information

Prioritize additional tests when their expected improvement to a specific decision is material relative to cost and delay. A test is valuable not just because it produces data, but because plausible outcomes could change whether, how, or when the system invests.

Where a credible probabilistic model exists, an evaluator may estimate expected value of sample information (EVSI) or a related decision-theoretic quantity. Record inputs, assumptions, sensitivity, and model limitations. Do not require a spurious Bayesian number when evidence is insufficient.

### 6.2 Portfolio optimization

Allocate across a portfolio, not a leaderboard. Consider:
- hard budget and capacity constraints;
- shared infrastructure and prerequisites;
- correlated technical risks and common assumptions;
- a declared balance across foundational, applied, exploratory, replication, and shared-infrastructure work;
- neglected questions or populations;
- equity of opportunity and concentration of prior awards;
- time-to-information and expected learning;
- resilience to individual project failure.

Present Pareto-efficient alternatives and explain what is gained or lost by choosing each. A single recommendation is acceptable only when objectives and trade-offs are explicit. Exploration budgets, randomized tie-breaks among comparably qualified proposals, and protected replication budgets are program-level policy choices, not hidden model defaults.

Do not infer merit solely from journal prestige, citation count, prior funding, affiliation, proposal-writing polish, model-generated reputation, or a trust graph.

### 6.3 Staged commitments

Where possible, split commitments into bounded tranches: pilot/validation milestone first, continuation based on predeclared evidence. Define in advance what justifies expand, redirect, pause, stop, or exceptional continuation. Assess quality of design and information gained independently of whether the hypothesis succeeded.

## 7. Non-expert due diligence brief

The UI must let a non-expert answer:
1. What is the project claiming in plain language?
2. What evidence supports and contradicts the claim?
3. What remains unknown, and what could make the conclusion wrong?
4. What will the requested resources buy, and which uncertainty will they resolve?
5. What are the defensible next choices and evidence behind each?

Every material assertion links to underlying sources or reproducible outputs. Distinguish **verified artifact**, **evidence supports claim**, **disputed**, and **unknown**; these are not interchangeable confidence tiers. Show coverage limits and missing evidence prominently. Provide an exportable audit bundle without exposing confidential source material to unauthorized recipients.

## 8. Threats and failure modes

Implementation and benchmark work must cover:
- fabricated, irrelevant, or overstated citations;
- source changes, retractions, corrections, missing metadata, dead links, and incompatible licenses;
- one source copied across many papers/organizations and falsely counted as independent evidence;
- compromised identity, key rotation/revocation, role change, forged assessor identity, duplicate attestations, and replay;
- conflicts of interest and coordinated or Sybil reviewers;
- p-hacking, selective reporting, leakage, contamination, weak baselines, underpowered studies, and post-hoc success criteria;
- hidden resource dependencies, unrealistic cost ranges, sunk-cost escalation, and proposal optimism;
- popularity/status feedback loops and allocation bias against novel or underrepresented work;
- agent collusion, correlated model errors, prompt injection in retrieved content, and assessor/evaluator/authority collapse;
- private/regulated data leakage, unlicensed dataset redistribution, and unsafe experimental recommendations;
- attempts to exploit the objective or optimize proxies rather than actual outcomes.

Retrieved documents and code are untrusted data, not instructions. Tool execution must be sandboxed, time/resource-bounded, logged, and separated from authority-bearing services.

## 9. Qualification plan: process-first, CI-independent

Progress is based on exact artifacts and observed results, not workflow status. Build a local deterministic synthetic corpus and independently inspectable reports before depending on external CI.

### Gate A — contract and fixtures

Create versioned fixtures for project, claim, evidence, assessment, resource request, decision, and outcome. Record canonical inputs and expected projection digests. Map them to the current event envelope. Do not introduce a parallel event schema or signing/authority mechanism.

### Gate B — deterministic evidence projection

For frozen inputs, independently replay the same events and require byte-identical canonical projection commitments under the same policy version. Reordering independent physical ingestion must not change semantic conclusions; a source-version change must change the relevant evidence identity.

### Gate C — adversarial tests

At minimum prove:
1. invalid signatures, stale policy, missing authority receipt, or invalid actor-role binding cannot yield an authorized decision;
2. duplicate reports from one provenance family do not increase independent-confirmation counts;
3. missing/contradictory sources yield indeterminate or explicit dispute, not silent PASS;
4. a correction/retraction creates a new projection and preserves old decision history;
5. changed project versions invalidate assessments bound to the prior version where required;
6. a request cannot reuse an assessment from a different request or policy;
7. a model cannot sign its own output as an independent evaluation;
8. resource authorization binds exact approved amount, recipient, purpose, project version, policy, and milestone;
9. a funding decision cannot claim disbursement or scientific success without separate evidence;
10. fabricated citations, prompt injection, missing data, and correlated reviewers do not silently increase confidence;
11. restricted data can be represented without exposing it, with rights/access restrictions explicit;
12. ties and score sensitivity are surfaced rather than disguised as precise rankings.
13. the same actor presented under multiple organization identities cannot silently count as multiple independent persons;
14. multiple actors within the same organization/data-source group are not treated as independent organization-level replications without an explicit profile;
15. changes to organization, protocol, dataset, lab, or method linkage cannot silently increase an independence count.

### Gate D — frozen benchmark and shadow decisions

Freeze cases before tuning. Include positive cases, negative/null findings, disputed claims, corrected/retracted papers, sparse-evidence cases, deliberately misleading proposals, and high-impact/high-uncertainty cases. Compare:
- structured human review;
- retrieval-grounded model analysis;
- retrieval-grounded analysis plus independent checks;
- a portfolio-aware allocation policy.

Record evidence precision/recall, claim-support accuracy, calibration/abstention, critical-error rates, reviewer time, and recommendation sensitivity. Measure outcome and fairness dimensions separately; do not combine them into one leaderboard.

### Gate E — prospective pilot

First run in shadow mode with no autonomous funding authority. Pre-register the objective, dataset, review process, primary outcomes, stopping criteria, conflicts policy, and analysis plan. Expand authority only after independent evaluation shows improved decision quality relative to a declared baseline without unacceptable harms.

CI can automate repeatable checks, but queued/running/passing CI is not evidence that these gates were met unless its exact head, inputs, outputs, and independent verification are recorded.

## 10. Initial measurable success criteria

Set numerical thresholds only after a baseline corpus is scored by independent adjudicators. Initial invariants:
- 100% of material brief assertions have a resolvable source/reproducible result, or are explicitly marked unsupported/unknown;
- zero cases where signature, reputation, citation count, or model confidence alone promotes a scientific claim to independently verified;
- zero cases where an assessment authority authorizes its own proposal outside the configured independent/governed path;
- 100% of decisions bind exact project, assessment, policy versions, and relevant authority evidence;
- 100% of corrections/retractions propagate to new projections without erasing prior event history;
- all critical errors in the frozen adversarial suite block promotion to the next operational stage;
- allocation recommendations disclose material objectives, constraints, uncertainty, and sensitive assumptions;
- prospective evaluation measures calibration, evidence quality, outcomes, cost, time, access/fairness, and negative results separately.

These are proposed acceptance criteria, not current measured results. Deployments involving human participants, clinical decisions, security-critical technology, hazardous materials, or regulated data require stronger domain-specific safeguards.

## 11. Implementation sequence

1. **Inventory and contract mapping:** inspect current event/payload types, projections, assessment code, and authorization policies. Produce a traceable mapping; do not code a parallel schema.
2. **Evidence projection and fixtures:** implement the smallest canonical assessment profile and independent replay fixtures.
3. **Diligence brief vertical slice:** connect one literature metadata adapter and one reproducible-code/data adapter; emit sources, support/contradiction links, unknowns, and an auditable brief.
4. **Decision record:** model requests, alternatives, milestones, conflicts, and authority-bound decisions; keep funds/execution out of scope until their own invariants are specified.
5. **DeSciBench:** run synthetic/adversarial and historical frozen cases, then shadow review. Publish an error taxonomy and baseline.
6. **Resource optimizer:** only after calibration and decision records exist, introduce constrained portfolio comparison, justified information-value analysis, and staged commitment recommendations.
7. **Prospective pilot:** authorize only a bounded experiment under human/governed control; assess outcomes independently.

The first implementation milestone is not an autonomous allocator. It is one reproducible path: **proposal → atomic claims → cited evidence map → independent assessment → bounded next-test recommendation → recorded human decision**.

## 12. Research and standards

- DARPA, [Research model](https://www.darpa.mil/research) and [Heilmeier Catechism](https://www.darpa.mil/about/heilmeier-catechism): focused objectives, explicit risk/cost, and intermediate/final tests.
- National Academies of Sciences, Engineering, and Medicine (2019), [Reproducibility and Replicability in Science](https://www.nationalacademies.org/read/25303/chapter/3): distinguishes reproducing computations from replicating a scientific finding with new data; useful for typed evidence relationships and benchmark fixtures.
- NIH, [Simplified Peer Review Framework](https://www.grants.nih.gov/policy-and-compliance/policy-topics/peer-review/simplifying-review/framework): separates importance, rigor/feasibility, and expertise/resources; aims to reduce reputational bias.
- GO FAIR, [FAIR Guiding Principles](https://www.go-fair.org/fair-principles/): persistent identification, machine-actionable metadata, interoperability, licensing, and provenance.
- DORA, [Guidance on responsible quantitative indicators](https://sfdora.org/resource/guidance-on-the-responsible-use-of-quantitative-indicators-in-research-assessment/): contextual use of metrics and caution against reducing research quality to one indicator.
- OpenAlex, [API reference](https://help.openalex.org/api/): linked metadata graph for discovery adapters; metadata is not scientific validation.
- *Nature* (2026), [OpenScholar](https://www.nature.com/articles/s41586-025-10072-4): retrieval-grounded scientific synthesis and citation evaluation, not a replacement for independent validation.
- *Research Evaluation* (2024), [Funding lotteries and fairness](https://academic.oup.com/rev/article/doi/10.1093/reseval/rvae025/7735322): lottery design involves trade-offs; random tie-breaks should be an explicit program policy, not a universal solution.
- Annual Reviews, [Value of Information Analysis](https://www.annualreviews.org/content/journals/10.1146/annurev-statistics-040120-010730): decision-theoretic methods can help prioritize further information collection when assumptions are credible.

## 13. Non-goals and claim ceiling

This RFC does not claim automatic determination of truth; full replacement of domain experts; a universally valid scalar quality/impact/trust score; that cryptographic provenance proves content true; that citations or replications alone establish validity; autonomous funding/disbursement/procurement/clinical action; or superiority to DARPA, NIH, or current funders before prospective comparative evidence.

A PASS against this RFC can establish only that specified evidence-grounded due-diligence and decision-recording invariants passed on the exact implementation, policy, and fixtures. Scientific and allocation superiority require separate prospective evidence.
