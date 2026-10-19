# Global building-code composition, evidence-led reform, and design freedom

**Status:** architecture and prototype contract. This is not a global building-code database, legal interpretation service, or claim of adoption by any authority.

## The governing principle

Make rules composable, inspectable, and challengeable without inventing one global legal hierarchy. A global engineering reference is reusable knowledge, not law by virtue of being imported. Binding requirements gain legal force only from a source and authority applicable to the project and assessment date. A national, provincial/state, municipal, land-use, site, and building-specific layer may contribute requirements, but geographic narrowness alone does not automatically grant precedence over a conflicting source.

This approach complements openBIM rather than replacing it. IFC 4.3.2.0 is an official buildingSMART release associated with ISO 16739-1:2024, while buildingSMART's regulatory information requirements work identifies machine-readable, vendor-independent permitting information as an interoperability need:
- https://technical.buildingsmart.org/standards/IFC/
- https://www.buildingsmart.org/standards/calls-for-participation/regulatory-information-requirements/

## 1. Composable modules

The Rust `composition` module introduces `RuleModule`, `RuleStatement`, `RuleModuleScope`, explicit override directives, composed results, and conflict records.

### Suggested module families

1. **Global reference modules:** common vocabulary, model requirements, engineering methods, open standards mappings, and non-binding reference baselines. These cannot directly declare binding law.
2. **International and regional instruments:** preserved as references with their actual legal force and adoption status in each jurisdiction. Publication by an international body is not itself proof of local adoption.
3. **National modules:** statutory law, national regulations, adopted standards, and national amendments.
4. **State/province/territory modules:** only the instruments that actually apply in that territory and their recorded relationship to national instruments.
5. **Municipal modules:** by-laws, planning schemes, local hazard or infrastructure requirements, and actual delegated authority.
6. **Parcel/site modules:** verified geotechnical data, flood/fire exposure, easements, connection constraints, overlays, and site-specific decisions. Site facts do not themselves rewrite legislation.
7. **Owner-intent modules:** preferences for style, spatial experience, materials, privacy, culture, accessibility, affordability, maintenance, resilience, and future change. These are design goals—not legal rules.

Modules identify their parents explicitly. Composition validates the selected dependency closure, rejects missing modules and cycles, and uses stable, deterministic ordering. Scope ancestry checks that a module does not import an unrelated or narrower jurisdiction as though it were a parent; scope is **not** a precedence rule. Regulatory-source reach is separate from module ancestry. The applicability evaluator uses exact matching by default; ancestor/descendant reach or explicit target jurisdiction IDs require an explicit reviewed scope mode. It does not treat a national-looking ID as automatically applicable everywhere below it, and never lets a sibling jurisdiction inherit scope by string similarity.

### No silent winners

For every binding requirement, the composer must return one of three meaningful outcomes:

- **Agreement:** applicable binding statements agree on the exact normalized value; source and clause references are retained.
- **Conflict:** applicable binding statements disagree. There is no selected effective value until a reviewed resolution is recorded.
- **Explicitly verified override:** a binding statement may supersede a named inherited binding statement only with an authority-decision reference, reviewer, rationale, and evidence digest. The metadata-only composer rejects overrides. The separate verified composer requires an external jurisdiction-specific verifier to authenticate the decision, reviewer, source bytes, and authority to vary that exact rule. The superseded source stays in the audit record.

A digest-shaped string or reviewer name is not authentication. Production must authenticate the module publisher, reviewer authority, override decision, source bytes, and signature/digest chain. An override in a prototype fixture is never permission to ignore a real code requirement.

Proposals, adopted-but-not-yet-effective instruments, guidance, engineering baselines, evidence-backed reform proposals, and owner preferences remain separately typed. The existing time-aware regulatory evaluator must run first. If source status is indeterminate, the composition must not fabricate a binding result.

## 2. A building code should express outcomes and preserve legitimate choice

Codes exist to protect public interests, including life safety, health, accessibility, structural performance, fire and egress, and resilience. But this does not mean every home must share one visual style, cultural vocabulary, construction method, floor plan, or smart-home philosophy.

The system should:

- record an owner's intended experience and hard constraints before proposing forms;
- generate alternatives rather than defaulting to the most conventional style;
- distinguish a legal minimum from a model recommendation, market convention, supplier preference, and owner's preference;
- support accepted prescriptive paths and reviewed performance-based / alternative-solution paths where the jurisdiction allows them;
- show exactly which objective a proposed alternative is required to demonstrate and what evidence is accepted;
- retain legitimate exceptions, variances, and authority decisions without presenting them as global precedent;
- surface conflicts between a preferred design and the applicable rule, then explain lawful redesign, alternative-design, appeal, or reform options rather than silently erasing the owner's goal.

Performance-based design is not an invitation to skip verification. The International Code Council's Performance Code describes an outcome-oriented framework for showing acceptable levels of performance, including safety and property/community objectives. Its application still depends on the code adopted, allowed methods, appropriate performance criteria, and the authority's review:
- https://shop.iccsafe.org/2024-icc-performance-coder-for-buildings-and-facilities.html
- https://codes.iccsafe.org/content/ICCPC2018/chapter-1-scope-and-administration

The software must never turn a visual rendering, owner preference, AI-generated rationale, or generalized research finding into evidence of compliance.

## 3. Evidence-backed change to codes
## 2A. Alternative means, variances, and appeals without false permission

A homeowner's preferred architectural expression is not itself a legal violation. But an owner preference does not waive a binding requirement either. The compiler should distinguish the conventional prescriptive path from a jurisdiction-authorized performance-based demonstration, variance request, or appeal. For each route, the selected jurisdiction adapter must establish that the route exists for the specific project category and target rule; the existence of a route is never a prediction of its outcome.

The Rust `alternative_solution` module is a new submission-dossier primitive. Each case ties a design revision to the exact jurisdiction module and rule statement it responds to, compares the prescriptive path, defines the protected performance outcomes and acceptance criteria, records the methods/versions/limits used, and links every assessment to verified evidence bytes. It requires a complete treatment of alternatives, expected benefits, foreseeable harms, affected groups, uncertainty, counter-evidence, and independent review of the exact case and design revisions.

A safety-critical failed or unknown criterion blocks submission. Non-critical unresolved outcomes may remain explorable but cannot be presented as a complete authority-submission package. Reviewers who disagree remain visible, and a request for more evidence downgrades the dossier to independent review. The strongest positive state is only `ReadyForAuthoritySubmission`: it is not proof of equivalence, code compliance, or permission to build.

This is the software expression of performance-based regulation: preserve the public outcome the rule is meant to secure, then evaluate an alternative method against explicit criteria. The jurisdiction's actual adopted code and authority must determine whether the path is available and what evidence is accepted. See the [ICC Performance Code](https://shop.iccsafe.org/2024-icc-performance-coder-for-buildings-and-facilities.html) and OECD's [Regulatory Impact Assessment principles](https://www.oecd.org/en/publications/regulatory-impact-assessment_7a9638cb-en.html).


Code reform is a first-class process, not an informal comment field and not an automatic side effect of a simulation.

The new `reform` module provides a `CodeReformDossier` that can record:
- target rule statements and jurisdiction;
- the problem and the rule's intended outcome;
- the proposed performance outcome and alternative rule;
- evidence items, study methods, contexts, outcome measures, findings, limitations, replication information, and source digests;
- evidence that challenges the proposal, not merely supportive findings;
- known uncertainty and search methodology for counter-evidence;
- foreseeable harms, benefits, trade-offs, affordability and lifecycle effects;
- the impact on affected groups, including accessibility and distributional impacts;
- conflicts of interest and independent reviews tied to the exact immutable dossier snapshot.

The dossier readiness states are deliberately narrow:

`Incomplete` → `ReadyForIndependentReview` → `ReadyForPublicConsultation`

The public-consultation state requires a complete dossier and at least two distinct reviewers who are not the author, with the review pinned to the exact dossier snapshot and at least one reviewer explicitly supporting public consultation. A reviewer who challenges the proposal or considers the evidence inconclusive remains recorded. The state **does not** mean that the evidence is conclusive, that the proposal is adopted, or that an active code has changed.

No universal numerical evidence score should declare a regulatory proposal scientifically conclusive across fire, structural engineering, indoor air quality, housing affordability, climate adaptation, and other domains. Appropriate evidence varies by problem. Specialists and authorized institutions determine sufficiency, and the dossier keeps uncertainty visible.

### Proposed reform lifecycle

1. **Observe:** record an adverse outcome, unnecessary constraint, novel material/system, or failure of a current rule to achieve its stated goal.
2. **Define the outcome:** identify what must remain protected and what change is being proposed. Compare no-change, prescriptive, performance-based, and non-regulatory alternatives.
3. **Reproduce:** version the datasets, analysis, assumptions, solver/tool versions, code, input model, reference cases, and known limitations. Simulation and synthetic data must be labelled as such.
4. **Seek disconfirming evidence:** search for counter-examples, failure modes, selection bias, confounding, regional/climate differences, unexpected consequences, and affected groups whose experience differs from the average.
5. **Independently review:** require reviewers with relevant expertise and disclosed conflicts; preserve dissent and revisions.
6. **Publish for consultation:** make a human-readable and machine-readable dossier available, including accessible summaries, methods, evidence artifacts, open uncertainties, and responses to comments.
7. **Submit through lawful authority:** the relevant regulator/standards body decides whether and how to adopt, amend, pilot, or reject a proposal.
8. **Close consultation and record the decision:** preserve the public notice, raw feedback archive, responses to comments, and consultation dates. Adoption metadata must reference the exact immutable dossier snapshot and the exact consultation-closeout digest; adoption cannot predate consultation closure. The closeout record itself pins the notice, feedback archive, response record, and dates.
9. **Version any adoption:** only after the authoritative act and effective date are authenticated should a new rule-module edition become binding in that jurisdiction. Preserve the prior rule and its historical use; do not rewrite old permit assessments.
10. **Monitor outcomes:** compare the expected result with post-adoption measurements; re-open review when evidence, incident patterns, costs, or technology change.

This follows the logic of regulatory impact assessment: assess the problem, options, positive and negative impacts, consultation, and evidence transparently rather than assuming every intervention is automatically beneficial. OECD guidance: https://www.oecd.org/en/publications/2020/02/regulatory-impact-assessment_0bf78a03.html

## 4. Global rollout without pretending all codes are the same

A machine-readable jurisdiction capability card is the rollout contract. It records bounded jurisdiction identity, status, domain-by-domain coverage, source registry snapshot, independent test corpus, last-checked date, update policy, accountable maintainer, known gaps and independent reviewers pinned to the same canonical subject digest. All eleven defined domains must be explicitly addressed before the system can accept a declared Supported label; a documented NotApplicable decision is different from an unknown/missing area.

A support card is only metadata-complete when its source registry is fresh under the chosen policy, tests and source snapshots are digest-pinned, and at least two distinct independent reviewers support the exact declared scope. A challenging or inconclusive review blocks a Supported metadata claim. Actual source bytes, reviewer identities, authority and test-corpus adequacy still require external verification and professional/legal governance. The card gate does not turn a profile into a legal certification.


Start with a small reviewed pilot in one municipality. Keep the registry global, but make each jurisdiction's support status explicit.

For every jurisdiction, publish a capability card:
- jurisdiction and boundaries, competent authority, applicable building/land-use/fire/energy/accessibility/heritage regimes;
- source coverage, source edition, effective dates, last-checked date, and missing sources;
- clauses and project categories supported by automation;
- required professional judgements and human/municipal approvals;
- accepted calculation methods, file/exchange formats, source licenses, and known limitations;
- independent test corpus, unresolved conflicts, false-positive/false-negative history, and governance owner.

A new country is a separate reviewed adapter and corpus, not a translation of another country's code with its label changed. Use common data models to reduce duplication, but preserve legal text, scope, language, local construction practice, hazard data, rights, and adoption records specific to each jurisdiction.

Potential interoperability spine: IFC for building information, IDS for information-delivery requirements, and provenance standards such as W3C PROV-O for tracing entities, activities, and agents. None of these standards on its own establishes building compliance:
- https://www.buildingsmart.org/standards/bsi-standards/information-delivery-specification-ids/
- https://www.w3.org/TR/prov-o/

## 5. Acceptance benchmarks

These are proposed targets, not measured results.

| Benchmark | Acceptance target |
|---|---|
| Cross-border legal leakage | Zero global reference modules accepted as binding solely because they are imported |
| Composition lineage | 100% of effective statements trace to module, source, clause, edition, and preserved artifact digest |
| Conflicts | 100% of disagreeing applicable binding statements surfaced; zero implicit geographic/numeric winner |
| Overrides | Every override names its target and review basis; invalid target, absent evidence, incompatible scope, or invalid source status blocks |
| Historical truth | 100% of historic assessments retain the exact rule-module revisions they used |
| Proposals | 0 proposals, guidance notes, or evidence-backed reform recommendations silently alter active legal requirements |
| Dissent | 100% of independent reviews and relevant counter-evidence preserved in the dossier |
| Consultation | Every prepared dossier contains the evidence, uncertainty, counter-evidence-search record, affected-group impacts, and immutable review version |
| Freedom of design | Curated briefs are evaluated against hard legal/safety constraints while preserving owner preferences separately; measure avoidable rejection of feasible alternatives |
| Global rollout | A jurisdiction is never labelled supported without source coverage, independent fixtures, documented gaps, and an update/governance owner |

## 6. Implementation status and limits

This prototype implements typed rule-module composition, explicit override metadata, conflict-preserving outputs, owner-preference separation, and code-reform dossier preparation gates. It does not provide authoritative legal ingestion, verified signatures, actual evidence-byte hashing, a global code registry, a full JSON Schema validator for every ruleset, or real authority adoption.

It should eventually integrate with Symtropy for design alternatives and immersive review, Sol-Atlas for versioned site/hazard context, Symthaea for explaining conflicts and generating alternative hypotheses, and Mycelix for provenance, governance, public review, and independent evidence records. Those are target integration boundaries, not claims that the end-to-end product exists today.
