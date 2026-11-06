# FIN-IFRS18-MPM-001 — Management-defined performance measure disclosure contract v1

**Parent:** [#5802 — XBRL / Inline XBRL reporting compiler](https://github.com/Luminous-Dynamics/mycelix/issues/5802)  
**Issue:** [#5885](https://github.com/Luminous-Dynamics/mycelix/issues/5885)  
**Status:** public synthetic conformance design; not a full IFRS compliance engine.

## 1. Research and effective-date boundary

The 2026 issued IFRS 18 text is the normative source for the v1 semantic contract.

- IFRS 18 becomes effective for annual reporting periods beginning on or after 1 January 2027; earlier application is permitted.
- The IFRS Accounting Taxonomy 2025 remains current for 2026 reporting periods. The next annual IFRS Accounting Taxonomy is expected in Q1 2027.
- IFRS 18 applies to financial statements prepared in accordance with IFRS Accounting Standards. Do not infer that an IFRS for SMEs entity applies IFRS 18.

Primary sources:
- https://www.ifrs.org/issued-standards/list-of-standards/ifrs-18-presentation-and-disclosure-in-financial-statements/
- https://www.ifrs.org/content/dam/ifrs/publications/pdf-standards/english/2026/issued/part-a/ifrs-18-presentation-and-disclosure-in-financial-statements.pdf?bypass=on
- https://www.ifrs.org/news-and-events/news/2026/02/ifrs-accounting-taxonomy-2025-to-remain-current-for-2026/

Relevant issued-standard paragraphs: 117–125, B113–B142, C1–C3. The public fixtures encode only selected machine-checkable obligations from these paragraphs. They do not encode the entire standard or replace professional interpretation.

## 2. MPM identification

A management-defined performance measure is a **subtotal of income and expenses** that:
1. the entity uses in public communications outside its financial statements;
2. communicates management's view of an aspect of the financial performance of the entity as a whole; and
3. is not a subtotal listed in paragraph 118 or specifically required to be presented or disclosed by IFRS Accounting Standards.

The verifier separates the raw identification facts from the candidate's claimed classification. A producer cannot make a measure an MPM by setting a boolean flag.

Important negative controls:
- ratios (for example, ROA) are not themselves MPMs;
- cash-flow/liquidity measures (for example, free cash flow) are not MPMs;
- a subtotal of income only (for example, adjusted revenue standing alone) is not an MPM;
- required IFRS subtotals in paragraph 118 are not MPMs;
- internal-only measures do not meet the public-communications criterion;
- a segment subtotal is not automatically an entity-wide MPM;
- a ratio's numerator/denominator may itself be an MPM if it independently meets paragraph 117.

The standard presumes that an income-and-expense subtotal used in public communications outside the financial statements communicates management's view, unless rebutted with reasonable and supportable information. A machine verifier can check that a rebuttal reference exists; it cannot decide whether the judgment is reasonable without expert review.

## 3. Disclosure contract

For each qualifying MPM, the note contract must preserve:
- clear, non-misleading label and description;
- what aspect of financial performance it communicates and why management considers it useful;
- calculation recipe, including included/excluded items;
- one coherent MPM note containing all MPM disclosure information for the entity and period;
- the statement that the measure reflects management's view and may not be comparable with similarly labelled measures at other entities;
- reconciliation to the most directly comparable subtotal listed in paragraph 118 or total/subtotal specifically required by IFRS;
- reconciling amounts linked to statement line items;
- income-tax and NCI effects for each reconciling item;
- the method used to determine each income-tax effect.

### Reconciliation arithmetic

For the synthetic verifier, each reconciling item is a signed exact integer string in the statement currency's minor unit. The contract is:

`MPM amount = comparable IFRS subtotal + Σ signed reconciling items`

No floating-point amount or implicit FX conversion is permitted.

### Tax effects

For each reconciling item, preserve the income-tax effect and the method reference. IFRS 18 paragraph B141 allows:
- the statutory tax rate(s) applicable to the underlying transaction(s) in the relevant tax jurisdiction(s);
- a reasonable pro-rata allocation of current and deferred tax; or
- another method achieving a more appropriate allocation.

When more than one method is used, the entity discloses how it determined the tax effects for each reconciling item. The verifier checks completeness and explicit method metadata; it does not judge tax law or the reasonableness of a tax allocation.

### Non-controlling interests

Each reconciling item includes an explicit NCI effect, including a zero amount where the reviewed determination is zero. Missing is not zero.

### Intermediate subtotal not presented

Where an MPM is reconciled to a subtotal not presented in the statement, paragraph B140 requires that subtotal to be reconciled to the most directly comparable subtotal presented in the statement. Tax and NCI effects are not required for that secondary bridge. The verifier must not confuse the secondary bridge with the primary MPM reconciliation.

## 4. Lifecycle and comparatives

A change in calculation, addition or cessation of an MPM, or change in how the tax effect is determined triggers the paragraph 124 disclosure set:
- explanation of the change/addition/cessation and its effects;
- reasons; and
- restated comparative information unless impracticable, with disclosure of that fact if impracticable.

The MPM identity and prior definition digest remain linked. Changing an MPM's label cannot erase the lineage or make a prior measure disappear from comparison.

## 5. Data and ownership boundaries

- #5802 remains owner of taxonomy packages, entry points, statement fact projection and XBRL/Inline XBRL compilation.
- #5792 owns bitemporal policy/framework applicability and approved policy interpretation.
- #5798 / PR #5836 owns evidence/assertion-purpose semantics.
- #5799 owns blinded candidate/evaluator separation and canonical benchmark run receipts.
- #5782 owns end-to-end benchmark protocol.
- This module owns the MPM definition, disclosure bridge and lifecycle contract only.

Every MPM value must trace to the same independently reconstructed statement facts. MPM presentation does not rewrite or authorize journal entries.

## 6. Public synthetic corpus v1.0.0

The corpus contains 26 cases. It is a public reference suite, not a blind holdout or model-performance benchmark. Cases exercise:
- valid disclosure;
- missing description, calculation, note, comparator and line-level reconciliation;
- arithmetic inconsistency;
- missing tax/NCI effects and method metadata;
- the B140 intermediate bridge;
- false-positive MPM classifications for ratios, liquidity/cash-flow measures, one-sided subtotals and IFRS-specified subtotals;
- internal-only and segment-only measures;
- change, addition, cessation and tax-method changes;
- valid early application, premature mandatory application and IFRS-for-SMEs separation.

All entities, measures, amounts and source references are synthetic. Oracle dispositions are machine-checkable semantic controls, not professional adjudications of real disclosures.

## 7. Qualification gates and measurements

Hard gates:
- zero false MPM classifications in seeded non-MPM controls;
- 100% exact reconciliation reconstruction on supported synthetic cases;
- zero missing required tax/NCI effects per item;
- zero accepted MPM with missing required label, description, usefulness, calculation or comparator;
- zero missing secondary bridge where paragraph B140 applies;
- zero silent lifecycle changes/cessations without the required comparative handling;
- zero mandatory IFRS 18 application before the effective date absent valid early application;
- zero mutation of canonical statement facts.

Report independently:
- MPM identification precision/recall by rule family;
- non-MPM false-positive rate;
- exact reconciliation parity;
- per-item tax/NCI coverage;
- lifecycle/comparative handling accuracy;
- profile/period selection accuracy;
- reviewer escalation and disagreement;
- compilation/validation latency and cost per MPM disclosure.

Do not combine these into one weighted score.

## 8. Claim ceiling

Passing establishes only deterministic behavior on the public synthetic contract. It does not establish full IFRS 18 compliance, underlying accounting correctness, tax correctness, audit sufficiency, CIPC/SARS filing acceptance, professional sign-off or production readiness. The official 2027 taxonomy/entry point and each applicable jurisdictional filing profile must be pinned before production qualification.
