# Accounting Evidence Benchmark v1

**Status:** Research protocol proposal; not an accounting-standard interpretation, certification, audit opinion, or measured performance claim.  
**Owner:** Mycelix Finance benchmark programme  
**Tracking:** [FIN-ACC-BENCH-001](https://github.com/Luminous-Dynamics/mycelix/issues/5782)  
**Primary accounting seam:** [FIN-ECO-006 / #1123](https://github.com/Luminous-Dynamics/mycelix/issues/1123)

## 1. Research thesis

Evaluate the complete, evidence-bound accounting path:

~~~text
source documents
  -> source/evidence graph
  -> qualified economic facts
  -> versioned accounting projection policy
  -> canonical journal
  -> period-close projection
  -> financial statements
  -> independent reconstruction and verification
~~~

The benchmark must test a system, not merely a language model's ability to answer accounting questions. A system only receives credit for a financial assertion when it can bind that assertion to the appropriate source, entity, period, unit/currency profile, accounting policy, journal lineage, and independent verification result.

The benchmark is an evaluation layer. It must compose existing Mycelix Finance, Symthaea and Symtropy work; it must not create a competing money type, ledger, benchmark receipt, economic authority model, or simulation engine.

## 2. Why the benchmark needs two external reference tracks

### 2.1 FINBALANCE — end-to-end document reconciliation

Tumpati et al., *FINBALANCE: A Multi-Document Accounting Reconciliation Benchmark*, Findings of EMNLP 2026 (October 2026), evaluates source-document bundles across eight industries, three period types and five difficulty levels. The deterministic generator supplies accounting policies, tax/FX treatments, document schemas, distractors and inconsistency templates; the benchmark defines 23 inconsistency-code labels.

The paper reports that six contemporary LLMs achieved at most 46% exact final-balance-sheet accuracy on a 710-record evaluation split. For four models, reconstructing balance sheets from their own journal entries scored 26–41 percentage points higher than the models' own reported balance sheets. The paper highlights source-document binding and aggregation failures.

Reference: https://aclanthology.org/2026.findings-emnlp.675/  
DOI: 10.18653/2026.findings-emnlp.675

**Implication:** the independent statement reconstructor is a first-class benchmark component. A model-produced statement must not be trusted merely because its individual entries look plausible or balance locally.

### 2.2 DualEntry DualBench100 — practical accounting task comparison

The October 2026 v1.0 public Accounting AI Benchmark evaluates 101 synthetic cases across eight task families: transaction classification, journal entries, accounts payable, accounts receivable, bank reconciliation, financial reporting, month-end close, and accounting knowledge.

Its public leaderboard reports a top overall score of 91.1%. The benchmark reports repeated runs, consistency, cost and duration. Its published case mix contains 37 easy, 36 medium and 28 hard cases. It reports 93.5% average performance on transaction classification versus 64.7% average performance on Accounts Receivable and Month-End Close. Many structured/numeric tasks use a 0.8 pass threshold and fixed tolerances, including 0.5% relative tolerance where applicable.

Reference: https://www.dualentry.com/benchmark

**Interpretation limit:** this is a vendor-published benchmark with synthetic cases. Its scores are useful as a public comparison point, not evidence of universal accounting competence or real-world audit readiness. We must preserve its exact scoring for comparison and also publish exact-value scores, per-category results and our own expanded tracks.

### 2.3 Field evidence and incumbents

- Choi and Xie, *Human + AI in Accounting: Early Evidence from the Field*, Journal of Accounting Research, June 2026, analyzes more than 200,000 transaction records at 79 SMEs. It reports productivity and reporting-quality gains, while finding that reliance on non-consensus AI recommendations can increase errors. Reference: https://www.gsb.stanford.edu/faculty-research/publications/human-ai-accounting-early-evidence-field
- Xero reported in August 2026 that its Auto Bank Reconciliation beta had automatically reconciled more than 100 million transactions and that nearly three in four customers reported time savings. This is a vendor-reported product result, not an independently controlled accuracy comparison. Reference: https://blog.xero.com/product-updates/auto-bank-rec-updates/

These sources point to a better target than generic AI adoption: preserve professional judgement where needed, while making routine work, evidence handling and quality assurance materially more efficient.

## 3. Existing architecture to compose

| Owner | Existing work | Benchmark use |
|---|---|---|
| Mycelix #1123 | Exact evidence-bound journal and economic projection boundary | Journal oracle contract; authority/accounting separation |
| Mycelix #5739 / PR #5761 | MoneyAmountV1 exact wire primitive and cross-language vectors | Exact monetary inputs and canonical serialization |
| Mycelix PR #5717 | Reversible evidence-bound bank migration | Import, cutover, duplicate and indeterminate cases |
| Mycelix PR #5738 | Matched payment-outcomes benchmark | Reuse matched cohort and cost/latency discipline |
| Symthaea #10370 | Benchmark evidence and task-universe accounting | Run receipts, terminal dispositions, anti-score-laundering |
| Symthaea #9735 | Canonical benchmark-run manifests | Dataset/code/model/config/resource provenance |
| Symthaea #7036 / #5612 | Financial-system function comparison and neutral capital-allocation research | Downstream financial decision and mechanism scenarios |
| Symtropy | Deterministic physical/operational simulation | Scenario input only; never the accounting system of record |

Keep these conceptual boundaries explicit:

~~~text
economic event / authorization != accounting representation
balanced journal != proof of authorization or settlement
bank/provider acknowledgement != final settlement
account balance != complete books
financial statement arithmetic != evidence completeness
simulation output != observed business fact
model confidence != accounting evidence
~~~

## 4. Benchmark tracks

### A. Source-to-evidence binding

Inputs: invoices, credit notes, purchase orders, contracts, bank statements, supplier statements, delivery/acceptance evidence, payroll records, tax artifacts, inventory records and general-ledger extracts.

Measure:
- document and line-item linkage precision/recall;
- counterparty and legal-entity resolution;
- reporting-period and cutoff accuracy;
- exact amount, currency, unit and tax-component extraction;
- evidence completeness and contradiction handling;
- duplicate and revised-document identity;
- missing, corrupted, forged or irrelevant evidence disposition.

Adversarial variants include near-duplicate invoices, altered bank details, missing pages, conflicting versions, date/time-zone ambiguity, OCR corruption, same amount/different counterparty, and prompt-injection text embedded in documents.

### B. Qualified economic fact to accounting projection

Evaluate source-event qualification separately from accounting classification. Each projection binds an explicit ledger profile, account taxonomy revision, projection-policy identity/revision, legal entity, period, asset/unit profile and exact source event.

Measure:
- account selection and account-role semantics;
- debit/credit direction;
- exact amount and sign;
- entity, subledger and reporting-unit assignment;
- period and effective-date selection;
- source-event admissibility;
- idempotency and conflict handling;
- reversal, correction and supersession lineage.

A balanced malicious journal must fail when its source event or policy is inadmissible.

### C. Journal-to-statements and period close

An independent evaluator reconstructs trial balance, statement of financial position, profit or loss, cash flow and account roll-forwards from the canonical postings.

Cover:
- opening/closing balance continuity;
- accruals, deferrals, provisions and allowances;
- depreciation and disposals;
- inventory and cost of goods sold;
- revenue cut-off and contract changes;
- FX recognition/translation under explicit policy;
- intercompany reconciliation and consolidation eliminations;
- retained earnings and equity movements;
- late adjustments and closed-period corrections.

Score exact numeric reconstruction separately from account classification, statement presentation, disclosure completeness and policy compliance. Do not collapse these into one pass/fail number.

### D. Reconciliation and exception handling

Cover bank-to-GL, AP, AR, intercompany, inventory, payroll, tax and payment/settlement observations.

Measure match precision/recall, false-match rate, partial matches, unmatched-item aging, timing differences, duplicate/replay behaviour and unresolved/indeterminate state handling. Exact amount equality alone must never authorize a match.

### E. Accounting policy and jurisdiction

Use versioned, effective-dated rule profiles. Track entity type, accounting framework, tax regime, jurisdiction, transition rule and applicability decision.

South Africa is the first localization candidate. SARS confirms that from 1 April 2026 the compulsory VAT registration threshold is R2.3 million and the voluntary threshold is R120,000; Turnover Tax has a separate regime and threshold. The benchmark must encode these as effective-dated rules rather than hard-coded model knowledge.

Reference: https://www.sars.gov.za/about/sars-tax-and-customs-system/budget/budget-2026-frequently-asked-questions/

The third edition of IFRS for SMEs is effective for annual periods beginning on or after 1 January 2027, with early application permitted. Full IFRS profiles must separately track applicable changes such as IFRS 18, also effective for annual periods beginning on or after 1 January 2027.

References:
- https://www.ifrs.org/issued-standards/ifrs-for-smes/
- https://www.ifrs.org/issued-standards/list-of-standards/ifrs-18-presentation-and-disclosure-in-financial-statements/

Do not claim statutory, tax, IFRS, GAAP, audit or assurance conformity from a generic benchmark result.

### F. Adversarial integrity and controls

Seed: duplicate/replay, same ID with changed fingerprint, evidence tampering, wrong entity, cross-currency substitution, stale policy, unauthorized approval, segregation-of-duties failure, backdating, omitted liabilities, unsupported accruals, malicious balanced journals, hidden related parties, altered audit history and partial-effect payment failures.

Measure both detection and safe disposition: reject, quarantine, request more evidence, open a suspense/exception case, or route to an authorized reviewer. A detected issue that is silently posted still counts as a failure.

### G. Operational value

Measure end-to-end duration; human review minutes; corrections/rework; exception aging; cost per accepted close; cost per correctly closed entity-period; p50/p95/p99 latency; and the all-in cost of OCR, models, connectors, verification and reviewer effort.

## 5. Matched experimental arms

Use the same frozen source bundles, entity context, accounting policy, allowed actions, time budget and resource budget:

1. Deterministic rules-only/reference baseline.
2. Strong model-only baseline without ledger feedback.
3. Model with conventional accounting-tool access.
4. Model plus deterministic accounting kernel.
5. Model plus source-bound evidence graph and accounting kernel.
6. Full Mycelix + Symthaea pipeline, with independent verification.
7. Qualified human accounting professional baseline on a stratified subset.

Include strong hosted and open-weight models. Pin exact model IDs/configuration, prompts, tool versions, seeds, source artifacts, policy snapshots, code commit, hardware, price schedule and run manifest. Repeat stochastic configurations. Human adjudication must be blinded where practical.

Ablations must show whether the evidence graph, deterministic ledger feedback, policy engine, verifier and Symthaea reasoning each contribute. No leaderboard score may be attributed to the model alone when it measures the whole tool/system configuration.

## 6. Metrics and proposed targets

Targets below are **research ambitions, not present results**.

### Correctness gates

- 100% of accepted journal entries pass checked exact debit/credit equality within every permitted asset/unit domain.
- 100% of accepted entries pass account, entity, period, profile, source-reference and idempotency invariants.
- Zero silent acceptance of seeded material contradictions, unsupported source substitutions or unauthorized postings in the frozen adversarial corpus.
- 100% of accepted material statement line-items trace to canonical postings and source/policy lineage.
- Independent verifier reconstructs the same canonical output from frozen inputs. Producer self-attestation does not qualify.
- Every task has an explicit terminal disposition: accepted, rejected, quarantined, indeterminate, needs-evidence or human-review. No silent drops.

### Quality targets

- DualBench100-compatible track: >=95% overall task pass and >=90% in each task category, while also reporting exact underlying correctness and the benchmark's original tolerance-based metric.
- FINBALANCE-compatible track: reproduce the published baseline first; then target >=70% exact final-statement accuracy as an initial stretch gate and >=85% as a later research target. Report document-linking metrics and each inconsistency class separately.
- Report selective risk/coverage: quality at the automation coverage actually achieved, not only accuracy on the easy cases accepted automatically.
- Demonstrate statistically supported gains against the strongest matched baseline. Report confidence intervals and paired comparisons; do not overstate small score differences.

### Economic targets

- >=50% reduction in human handling time on a bounded routine-close cohort without increasing material error or exception leakage.
- Lower fully loaded cost per correctly closed entity-period than the matched workflow baseline.
- Publish cost/quality/latency Pareto frontiers for hosted and open-weight configurations.
- Track reviewer override and downstream correction rates; fewer human touches is not a win if error escapes increase.

## 7. Dataset governance and anti-leakage

- Review FINBALANCE and DualBench100 license/terms before importing, embedding or redistributing cases. If rights do not allow reuse, create a rights-cleared reproduction/extension and label it accurately.
- Separate training, development, public evaluation and hidden holdout sets. Commit exact dataset hashes and freeze scoring versions.
- Use deterministic synthetic generation for exhaustive edge cases; expert-authored cases for accounting ambiguity; and real de-identified cases only with written rights, privacy review and controlled access.
- Pre-register metrics, tolerances, exclusions, baselines and statistical stopping rules.
- Use independent finance professionals to author and adjudicate ambiguous cases; preserve disagreement and the policy rationale.
- Release evaluator code, schemas, public fixtures and machine-readable outputs where rights/security permit.
- Exclude customer credentials, tax identifiers, bank account details and personal data from public fixtures.

## 8. Implementation tranches

**S0 — Protocol and baseline freeze**
- Confirm benchmark versions, methodology and reuse rights.
- Define the task ontology and canonical task/result schema.
- Select a small representative case portfolio; do not maximize case count for its own sake.

**S1 — Independent deterministic oracle**
- Freeze source bundles, policy snapshots, oracle events, journal and statement outputs.
- Build a standalone Rust scorer/reference projector that does not import producer validation code.
- Add canonical hashes, replay protection, exact amounts, and clear task dispositions.

**S2 — Property and adversarial campaign**
- Test posting permutation invariance, amount/account/entity mutation, duplicate replay, changed source fingerprint, missing evidence, policy revision, FX mismatch, reversal lineage, hidden liability, period cutoff and statement reconstruction.
- Mutation-test the verifier and include negative controls that must fail.

**S3 — Baselines and published run**
- Run deterministic, model-only, tool-assisted and full-stack arms.
- Emit machine-readable run receipts binding source, data, policy, code, model, configuration, tools, costs and outputs.
- Publish exact-head results; queued CI is not a pass.

**S4 — External and field validation**
- Expand to multi-industry cohorts and bounded South African rules.
- Conduct a rights-cleared pilot with professional reviewers.
- Add enterprise consolidation and Symtropy operating scenarios only after the accounting evidence and ledger layers are qualified.

## 9. Claim ceiling

A benchmark pass does not establish audit assurance, statutory compliance, tax correctness outside the frozen profile, legal validity, valuation truth, completeness of real-world books or production readiness. Those require separate standard/jurisdiction profiles, professional review, rights-cleared source evidence, operational security assessment and appropriate approvals.

The success criterion is not “AI processed more transactions.” It is fewer material errors, fewer unsupported assertions, better financial-statement reconstruction, lower cost per correctly closed entity-period, and trustworthy abstention when evidence is insufficient.

## 10. Primary references

1. Tumpati et al. (2026), FINBALANCE, Findings of EMNLP: https://aclanthology.org/2026.findings-emnlp.675/
2. DualEntry Labs (October 2026), Accounting AI Benchmark / DualBench100 v1.0: https://www.dualentry.com/benchmark
3. Choi & Xie (June 2026), Human + AI in Accounting, Journal of Accounting Research: https://www.gsb.stanford.edu/faculty-research/publications/human-ai-accounting-early-evidence-field
4. Xero (August 2026), Auto Bank Reconciliation product update: https://blog.xero.com/product-updates/auto-bank-rec-updates/
5. SARS, Budget 2026 VAT/Turnover Tax thresholds: https://www.sars.gov.za/about-sars-tax-and-customs-system/budget/budget-2026-frequently-asked-questions/
6. IFRS Foundation, IFRS for SMEs third edition and effective date: https://www.ifrs.org/issued-standards/ifrs-for-smes/
7. IFRS Foundation, IFRS 18: https://www.ifrs.org/issued-standards/list-of-standards/ifrs-18-presentation-and-disclosure-in-financial-statements/


## 11. Cross-system implementation owners discovered during repository audit

The benchmark protocol is coordinated with these existing and newly scoped owners; none of them is a replacement for the core Mycelix general ledger:

- Symthaea [#5398](https://github.com/Luminous-Dynamics/symthaea/issues/5398) and [#5399](https://github.com/Luminous-Dynamics/symthaea/issues/5399) own evidence-grounded financial intelligence and trajectory evaluation. New accounting specialization: [SYM-ACCT-001 / #10422](https://github.com/Luminous-Dynamics/symthaea/issues/10422). It produces source-bound candidate workpapers, exception explanations, and close-readiness findings only; it does not post journals or acquire ledger authority.
- Symthaea [#5611](https://github.com/Luminous-Dynamics/symthaea/issues/5611) and [#5613](https://github.com/Luminous-Dynamics/symthaea/issues/5613) own stock-flow consistency and counterparty-bound closure for financial-system scenarios. These protect macro/sector simulation identities; they are not substitutes for a business entity's general ledger.
- Symthaea [#7036](https://github.com/Luminous-Dynamics/symthaea/issues/7036) and [#5612](https://github.com/Luminous-Dynamics/symthaea/issues/5612) own financial-function comparison and neutral capital-allocation scenario work.
- Symtropy [PR #3091](https://github.com/Luminous-Dynamics/symtropy/pull/3091) contains a separate exact-integer mutual-credit prototype with pooled demurrage and explicit reserve-funded debt relief. Treat it as a research/reference model, not the production Mycelix ledger and not a direct repair to the legacy Bevy economy path.
- Symtropy [#3043](https://github.com/Luminous-Dynamics/symtropy/issues/3043) tracks a concrete legacy TEND demurrage rounding defect. The default Symtropy source currently applies an independent one-unit minimum adjustment to each positive/negative balance. The case [+100, -50, -50] can become [+99, -49, -49], changing the zero-sum total by +1. Do not use that simulation path as financial authority; correct and qualify its explicit economic semantics first.

The existing Symthaea benchmark infrastructure (#10370 and #9735) remains the preferred run-evidence and reproducibility substrate. Do not create a second benchmark receipt schema or duplicate the task universe.


## 12. Expanded 2026 benchmark landscape — do not collapse scores

A November 2026 review found a broader set of public and semi-public finance/accounting evaluation tracks. The benchmark suite should use each only for the capability it actually measures.

### A. APEX-Accounting — professional month-end close workflows

Mercor and Ramp's APEX-Accounting evaluates simulated-company bookkeeping and month-end-close work across accounting systems, spreadsheets, PDFs and other files. The public board snapshot checked 1 November 2026 reports a leading 71.5% mean score over criteria passed. The benchmark page describes 10 worlds / 100 leaderboard tasks / 1,475 grading criteria; the associated paper describes 160 private tasks across 10 worlds. The open release is only one world / 10 tasks. It excludes tax, audit, consolidation, multi-entity and multi-currency accounting, and clarification-seeking. Its rubric judge is an LLM judge, so we should preserve its native score for comparison while independently checking deterministic numeric and journal invariants.

References:
- https://www.mercor.com/blog/introducing-the-ai-productivity-index-for-accounting/
- https://benchmarks.finance/benchmarks/apex-accounting/

### B. BlueFin — spreadsheet dynamic correctness

BlueFin evaluates professional spreadsheet tasks for workbook synthesis, manipulation and interrogation, emphasizing whether formulas continue working when inputs change. Its 2026 v1 reports 131 tasks / 3,225 rubric criteria, 11 public tasks and 120 held-out evaluation tasks. The highest reported result in the checked May 2026 results was 49.6% criteria-weighted score. Its grader is an LM judge validated against expert ratings (reported Krippendorff alpha 0.826). It does not cover accounting close or live data.

References:
- https://arxiv.org/abs/2605.30907
- https://benchmarks.finance/benchmarks/bluefin/

The arXiv identifier must be resolved from the linked primary benchmark paper before any benchmark reproduction or paper citation is frozen. The benchmark directory is a discovery aid, not a substitute for checking original rights and methods.

### C. DualEntry DualBench100 — controlled task-level accounting

Retain its exact v1.0 case set and deterministic checks where rights permit. Report its original repeated-run task-pass and tolerance metrics, but add exact-value and category-level scores. Its synthetic 101-case profile is useful for common tasks, not a proxy for real-world financial-statement completeness.

Reference: https://www.dualentry.com/benchmark

### D. FINBALANCE — evidence-to-statements correctness

Retain this as the primary public reference for multi-document source binding, journal projection, statement aggregation and inconsistency detection. Its exact-statement score is not comparable to the rubric or task-pass metrics above.

Reference: https://aclanthology.org/2026.findings-emnlp.675/

### E. General finance indexes and benchmark registries

Artificial Analysis' Finance & Accounting Index currently combines agentic knowledge work (30%), business knowledge (25%), reasoning (15%), non-hallucination (15%), agentic tool use (10%), and long-context work (5%). This is useful for model selection and broad financial literacy/agent capability, but it is not an accounting-ledger correctness benchmark.

Reference: https://artificialanalysis.ai/indexes/finance-and-accounting/

The third-party benchmarks.finance directory reported 47 finance benchmarks and 419 sourced scores across 234 models as of 3 November 2026, with separate metrics rather than a universal aggregate. Use it to discover relevant benchmark candidates and provenance, not as an authoritative industry standard or a score to optimize directly.

Reference: https://benchmarks.finance/

### F. Minimum benchmark portfolio for Mycelix

Keep a small, complementary portfolio:
1. FINBALANCE-compatible track — source documents to independently reconstructed statements.
2. APEX-compatible track — multi-step close and bookkeeping workflows.
3. DualBench100-compatible track — common structured accounting tasks and consistency.
4. BlueFin-compatible track — spreadsheet formulas and dynamic correctness.
5. Mycelix adversarial track — evidence integrity, authorization, exact-money and replay semantics.
6. Field/pilot track — rights-cleared historical workflows with professional adjudication and measured human handling time.

Do not sum scores across these tracks. Report the capability, metric, split, date, source and evaluator for each.

## 13. Competitive reality as of 4 November 2026

The incumbent category is moving quickly and already includes many capabilities that were previously proposed as differentiators:

- Intuit announced Agentic Books Close in Intuit Accountant Suite, workflow playbooks, portfolio intelligence and a unified platform across QuickBooks / Enterprise Suite / Accountant Suite. Its 4 November 2026 announcement reports more than 250,000 accounting firms adopted Intuit Accountant Suite; Intuit describes an ecosystem spanning 60+ specialized language models, 100+ AI skill agents, 100+ workflows and 74 financial domains. These are vendor-reported figures and product claims, not independent benchmark results.
- NetSuite 2026.2 describes an AI transaction-matching assistant, Intelligent Flux Analysis, close-management tasks, payment automation and reporting workflow capabilities.
- QuickBooks' 2026 documentation describes Accounting AI that asks for context and can update transactions automatically with user permission.
- Sage announced embedded AI agents across finance, HR and operations in April 2026.

Primary/vendor references:
- https://investors.intuit.com/news-events/press-releases/detail/1329/intuit-expands-intelligence-platform-to-power-the-next-era-of-accounting-firm-and-business-growth
- https://www.netsuite.com/portal/resource/articles/financial-management/netsuite-2026-2-adds-ai-capabilities-for-smarter-bank-reconciliation-close-management-labor-insights-and-more.shtml
- https://quickbooks.intuit.com/learn-support/en-za/help-article/accounting-bookkeeping/overview-agents-quickbooks-online/L9irCAtK4_ZA_en_ZA
- https://www.sage.com/en-us/news/press-releases/2026/04/sage-expands-ai-agents-across-finance-hr-and-operations-to-automate-workflows/

**Strategic consequence:** do not claim that continuous close, AI transaction matching, natural-language finance assistance, or agentic workflow automation is uniquely ours. Incumbents have distribution, integrations, customer context, existing accounting systems and deeply embedded workflows.

The defensible differentiation hypothesis is narrower and harder:
- source-to-statement provenance with independent reconstruction;
- explicit separation of economic authorization/settlement from accounting representation;
- exact-money and cross-language canonical semantics;
- adversarially qualified evidence and safe exception dispositions;
- policy snapshots with effective dates and reproducible treatment;
- portable evidence and review workpapers across existing accounting systems;
- business-operational digital twins that are reconciled to observed accounting facts rather than mixed into the ledger.

The product strategy should initially be **an intelligence, evidence and independent-verification layer that works with incumbent systems**, not a costly rip-and-replace accounting suite.
