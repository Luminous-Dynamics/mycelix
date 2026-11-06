# FIN-ASSURE-PROCEDURE-001 — Risk-responsive evidence procedures and stand-back v1

**Tracking:** [#5838](https://github.com/Luminous-Dynamics/mycelix/issues/5838)  
**Parent benchmark:** [#5782](https://github.com/Luminous-Dynamics/mycelix/issues/5782)  
**Evidence owner:** [#5798](https://github.com/Luminous-Dynamics/mycelix/issues/5798)  
**Workpaper DAG:** [#5803 / #5805](https://github.com/Luminous-Dynamics/mycelix/issues/5803)  
**Research input:** IAASB proposed revisions to ISA 330, ISA 500 and ISA 520, published 5 August 2026; comments requested by 15 December 2026.

This is a synthetic engineering research contract. The IAASB material is an exposure draft, not final or effective auditing standards. No "ISA compliant", audit, assurance or audit-opinion claim follows from this document or its fixtures.

## 1. Research thesis

An evidence artifact is not universally "trusted". Its relevance and reliability depend on:
- the assertion being examined;
- the intended purpose of the procedure;
- the source's origin, authority, integrity and independence;
- the reporting entity, period and unit;
- the population and denominator the procedure purports to cover;
- contradictions, limitations and evidence that arrived later.

The system should therefore evaluate a risk-to-assertion-to-procedure-to-evidence graph, not a document-level confidence score or checklist completion rate.

The target relationship is:

```text
assessed risk
  -> material assertion
  -> intended procedure purpose
  -> declared population / denominator
  -> performed procedure
  -> purpose-relevant evidence
  -> exceptions and contradictions
  -> reviewer disposition / residual risk
  -> stand-back result
```

A model may propose a procedure, explain an exception or draft a workpaper. The deterministic verifier checks whether declared prerequisites and results satisfy the machine-checkable contract. Authorized humans retain judgment, approval and acceptance of residual risk.

## 2. Standards research boundary

The IAASB exposure drafts published on 5 August 2026 propose revisions to:
- ISA 330 — The Auditor's Responses to Assessed Risks;
- ISA 500 — Audit Evidence;
- ISA 520 — Analytical Procedures.

The IAASB describes the project as strengthening principles-based, risk-based audit work for a digital environment. The public proposal emphasizes intended procedure purpose, relevance and reliability of evidence, professional skepticism and auditors' responses to assessed risks. The consultation closes 15 December 2026.

Primary source: https://www.iaasb.org/publications/proposed-revisions-audit-evidence-risk-response-isa-330-isa-500-isa-520

Engineering response:
1. Build a source-to-requirement research matrix; record exact paragraph/clause references from the downloaded exposure drafts, reviewer interpretation and unresolved questions.
2. Keep proposal text distinct from current effective standards and jurisdictional requirements.
3. Use qualified audit professionals to review any mapping. Do not infer compliance from similar vocabulary.
4. Preserve a dated source snapshot and digest when the exact exposure-draft documents are vendored.
5. Reconcile the final standard, if issued, as a new version; never silently rewrite historical procedure records.

## 3. Owner boundaries

| Concern | Owner | This work consumes |
|---|---|---|
| Assertion-specific evidence purpose and reliability | #5798 | Assertion/evidence relationships |
| Expected source-population completeness | #5790 / #5801 | Expected and observed populations |
| Approved policy and legal applicability | #5785 / #5792 | Approved policy revision and applicability |
| Durable workpaper dependencies and staleness | #5803 / #5805 | Parent commitments, frontier invalidation |
| Blinded candidate/oracle separation | #5799 | Candidate task and evaluator boundary |
| Bank statement semantics | #5812 / #5827 | Source-specific evidence semantics |
| Journal projection | #1123 / #5820 | Separate accounting representation |
| Reporting taxonomy validation | #5802 | Reporting facts after journal reconstruction |

This work owns the procedure coverage and stand-back semantics only. It does not create a second source registry, ledger, tax engine, audit engine, reviewer identity system or workpaper DAG.

## 4. Minimum domain contract

### 4.1 Required risk/assertion pair

Every material pair binds:
- stable risk ID and revision;
- reporting entity, period, framework and reporting scope;
- materiality / significance assessment and reviewer/source;
- financial-statement line or disclosure;
- assertion (existence, completeness, rights/obligations, occurrence, accuracy, cut-off, classification, valuation/allocation, presentation/disclosure);
- intended procedure purpose;
- requirement state and rationale.

### 4.2 Procedure record

A procedure binds:
- immutable procedure ID and revision;
- exact risk/assertion/purpose pair;
- procedure type (inspection, external confirmation, recalculation, reperformance, observation, inquiry, analytical procedure or governed alternative);
- declared population, sample frame, inclusion/exclusion predicates, denominator and selection method;
- execution state (planned, performed, not performed, failed, blocked);
- source artifact and workpaper references;
- execution time, executor/role, code/tool/configuration digest and parameters;
- result, deviations, exceptions and contradictory evidence;
- reviewer disposition and residual-risk state.

A planned procedure is not performed evidence. A procedure that used the wrong population or denominator is not cured by a numerically plausible output.

### 4.3 Evidence record

Evidence is purpose- and assertion-specific. At minimum preserve:
- exact source artifact/workpaper reference and digest where bytes exist;
- origin group and upstream source lineage;
- authenticated-source result and limitations;
- assertions and procedure purposes it can support;
- entity/period/unit binding;
- assertion/procedure-specific reliability requirements and rationale;
- separate evidence-attribute states for accuracy, completeness, authenticity, absence of bias and credibility;
- the exact attributes that the procedure needs from each evidence item, without a universal scalar trust score;
- contradictory or superseding artifacts;
- source creation, booking/value, capture and knowledge times separately.

Two exports from the same upstream register do not become two independent sources merely because they have different filenames or message IDs.

### 4.4 Population and analytical contracts

A population manifest binds the expected universe, observed universe, denominator, filters, grouping/join keys, cutoff, exclusions and source authority. Missing is not zero.

An analytical procedure additionally binds:
- formula and exact numerator/denominator workpapers;
- comparison period and population;
- segmentation and aggregation;
- unit/currency/FX profile;
- expectation model and tolerance;
- threshold for investigation;
- follow-up procedures and dispositions.

A ratio within tolerance is not sufficient when a material contradictory source remains unresolved. A zero denominator or missing population must not become zero/normal.

## 5. Stand-back state machine

Each required risk/assertion pair receives exactly one terminal state:

| State | Meaning |
|---|---|
| `addressed_with_qualified_evidence` | Required procedure was performed; population and relevant evidence meet the declared contract; no unresolved material contradiction |
| `addressed_with_explicit_limitation` | A scoped limitation is explicit and reviewable; it is not hidden as a pass |
| `exception_requires_resolution` | Contradiction, unexplained difference or unresolved exception blocks clearance |
| `insufficient_evidence` | Evidence, reliability, population, denominator or procedure purpose is inadequate |
| `procedure_not_performed` | Required procedure is planned, missing, blocked or failed without a qualified alternative |
| `not_applicable_with_authorized_rationale` | Non-applicability is explicitly authorized, entity/period specific and evidenced |
| `human_judgment_required` | Machine checks cannot determine the residual risk or policy judgment |

The stand-back result is an inventory of unresolved risk/assertion pairs, not a weighted confidence score.

An unqualified `ready_for_authorized_signoff` state is blocked by:
- material risk/assertion pairs without qualifying terminal evidence;
- missing expected populations or sample-frame gaps;
- unresolved material contradictions;
- unperformed required procedures without approved alternatives;
- failed relied-upon controls without an appropriate response;
- stale procedure/workpaper frontiers;
- unauthorised non-applicability or residual-risk decisions.

This is a software gate for review readiness only. It is not an audit conclusion or authorization to sign a financial statement.

## 6. Adversarial public synthetic corpus

The v1.0.0 corpus has 18 base cases; v1.1.0 extends it to 22 cases with pair-level results and entity/purpose/analytical controls; v1.2.0 extends it to 25 cases with pair-specific evidence-reliability attributes. All public oracle releases are regression references, not blind holdouts. The expected outcomes are published in a separate **public reference oracle** so implementers can reproduce tests. This is not a blind holdout or an independently adjudicated professional dataset.

| ID | Challenge | Required disposition |
|---|---|---|
| FAP-001 | Relevant reliable evidence for the correct assertion and purpose | addressed with qualified evidence |
| FAP-002 | Evidence supports occurrence but not completeness | insufficient evidence |
| FAP-003 | Bank statement proves booked debit, not recipient credit/discharge | insufficient evidence |
| FAP-004 | Supplier invoice supports a billing claim, not delivery/authenticity | insufficient evidence |
| FAP-005 | Balanced ledgers but expected population incomplete | insufficient evidence |
| FAP-006 | Sample frame omits an entire transaction class | insufficient evidence |
| FAP-007 | Wrong denominator coincidentally preserves ratio output | exception requires resolution |
| FAP-008 | Stale prior-period evidence used for current period | insufficient evidence |
| FAP-009 | Contradictory independent source evidence | exception requires resolution |
| FAP-010 | Two artifacts share one upstream origin but are counted as independent | insufficient evidence |
| FAP-011 | Required procedure planned but not performed | procedure not performed |
| FAP-012 | Required procedure absent with no authorized alternative | procedure not performed |
| FAP-013 | Control failure without a compensating procedure | procedure not performed |
| FAP-014 | Analytical threshold lacks an approved rationale | insufficient evidence |
| FAP-015 | Ratio is within tolerance but material contradictory evidence exists | exception requires resolution |
| FAP-016 | Assertion is not applicable with authorized scoped rationale | not applicable with authorized rationale |
| FAP-017 | Late evidence changes a completed workpaper frontier | exception requires resolution / stale |
| FAP-018 | Residual risk requires professional judgment | human judgment required | 
| FAP-019 | Two required pairs: one supported, one with an incomplete population | Overall insufficient; pair dispositions remain separate |
| FAP-020 | Procedure purpose differs from required assertion-purpose pair | insufficient evidence |
| FAP-021 | Evidence belongs to a different reporting entity | insufficient evidence |
| FAP-022 | Analytical procedure exceeds its authorized investigation threshold | exception requires resolution |

## 7. Qualification gates and measurements

Hard gates:
- exactly one terminal disposition for every required risk/assertion pair;
- zero material risks marked addressed without a performed qualifying procedure;
- zero evidence artifacts treated as universally reliable across unrelated assertions;
- zero unknown populations, denominators or exclusions silently treated as zero;
- zero false independence where artifacts share upstream source lineage;
- 100% detection of seeded wrong-denominator, wrong-period, stale-source and contradiction cases;
- 100% stale propagation to affected workpapers and zero invalidation of unrelated work;
- 100% of analytical differences over a declared threshold receive a follow-up disposition;
- zero automated elevation of candidate workpapers to audit opinion, assurance or approval authority.

Report separately:
- risk coverage and assertion coverage;
- procedure completion and exception rate;
- evidence purpose/reliability disposition;
- contradiction recall and false-positive burden;
- false assurance and stand-back recall;
- denominator/population error detection;
- human override/reversal rate;
- reviewer minutes, rework and cost per resolved risk/assertion pair.

Do not combine these into a single "audit readiness" score.

## 8. Comparative study

Run identical source bundles, scopes, permissions and budgets through:
1. checklist/rules-only baseline;
2. model-only;
3. model with conventional accounting tools;
4. assertion/evidence graph;
5. risk/procedure/evidence graph plus independent verifier;
6. blinded professional reviewer on a stratified subset.

Use deterministic scoring for machine-checkable invariants and expert adjudication for judgment-dependent cases. A single LLM judge is not the gold standard.

APEX, FINBALANCE and DualBench measure adjacent accounting workflow/reconstruction skills; they do not establish audit evidence sufficiency or compliance with ISA proposals. Preserve those benchmark distinctions.

## 9. Implementation sequence

1. Obtain and hash the IAASB exposure-draft documents and build a paragraph-level research matrix.
2. Freeze the procedure, population and terminal-disposition schemas.
3. Preserve public v1.0.0, v1.1.0 and v1.2.0 synthetic tasks and their version-specific schemas/oracles separately.
4. Implement a pure Rust stand-back verifier or a tightly scoped extension to the existing workpaper verifier.
5. Add mutation tests for wrong purpose, population, denominator, source origin, period, stale frontier and contradiction.
6. Produce an exact-head report binding source, policy, procedure schema, corpus and verifier digests.
7. Arrange qualified audit-professional review before publishing a standards mapping or piloting an assurance workflow.

## 10. Claim ceiling

A pass establishes synthetic procedure-coverage and stand-back behavior under the declared profile. It does not establish ISA compliance, audit evidence sufficiency in a real engagement, professional audit judgment, an audit opinion, regulatory compliance, fraud-free status or production assurance readiness.


## 11. v1.1.0 semantic hardening

The v1.1.0 input contract requires every evidence item to bind the reporting entity as well as the reporting period, and every procedure to bind the exact intended purpose. The verifier emits pair-level dispositions alongside the aggregate stand-back state. A case with multiple required assertions cannot be represented as clear merely because one pair is supported. Additional controls cover mismatched procedure purpose, cross-entity evidence and analytical differences exceeding a declared threshold. v1.0.0 fixtures, schema and public reference oracle remain separately versioned and replayable.

## 12. v1.2.0 multi-dimensional evidence reliability

The v1.2.0 input contract removes the global `reliability_status` scalar. Every evidence artifact carries independent states for `accuracy`, `completeness`, `authenticity`, `absence_of_bias` and `credibility`, each of which may be `qualified`, `limited`, `unqualified` or `unknown`. Every required risk/assertion/procedure-purpose pair declares its own `required_evidence_attributes`. The verifier only blocks the pair when an attribute required for that pair is not qualified; it does not downgrade an unrelated assertion because an irrelevant attribute is limited. FAP-023 demonstrates one artifact supporting an occurrence pair while failing a completeness pair; FAP-024 and FAP-025 independently test bias and credibility failures. These are synthetic contract attributes, not a normative interpretation of the proposed ISA text.
