# Financial Close Completeness Benchmark v1

**Status:** Synthetic research protocol and public development fixtures. Not an audit standard, accounting opinion, legal interpretation, or measured product result.  
**Tracking:** [FIN-ACC-COMPLETE-001 / #5790](https://github.com/Luminous-Dynamics/mycelix/issues/5790)  
**Parent:** [FIN-ACC-BENCH-001 / #5782](https://github.com/Luminous-Dynamics/mycelix/issues/5782)  
**Composes:** [Accounting Evidence Benchmark v1](ACCOUNTING_EVIDENCE_BENCHMARK_V1.md), [FIN-ACC-IMPL-001 / #5789](https://github.com/Luminous-Dynamics/mycelix/issues/5789)

## 1. Research thesis: qualify the negative space

A balanced trial balance proves only that the posted debits and credits agree under the selected ledger rules. It does not prove that every expected bank account, supplier population, receivable channel, subsidiary, payroll liability, inventory location, or related-party population was represented.

The benchmark therefore changes the question from:

> Did the system process every document it received?

to:

> Was the expected evidence universe independently enumerated, and does every in-scope population have a qualified disposition for this entity and period?

This is a distinct capability from transaction classification, source-to-journal linking, arithmetic correctness, or statement reconstruction. It is a necessary condition for a defensible close, not a guarantee that all real-world liabilities or events have been discovered.

## 2. State model and normative vocabulary

Each close must preserve these separate sets:

1. **Expected populations:** populations discoverable from authoritative control inventories and the reporting boundary.
2. **Declared populations:** populations named in the close-scope manifest.
3. **Observed populations:** source feeds or evidence populations actually received.
4. **Processed populations:** observations parsed and linked by the candidate system.
5. **Qualified populations:** observations that satisfy identity, authority, period coverage, integrity, and independence requirements.
6. **Excluded populations:** populations explicitly not applicable or outside the reporting boundary, supported by evidence and authorized review.
7. **Unresolved gaps:** missing, partial, conflicting, late, unavailable, or unknown populations and evidence.

These sets must not be collapsed into a single document-count or percentage-coverage figure.

### Close dispositions

- `complete`: every in-scope required population has a qualified period disposition; all exclusions are explicitly supported; no unresolved material gap or late evidence requires reopening.
- `incomplete`: an authoritative inventory identifies an in-scope population that is omitted, partial, or unsupported. A seeded material omission must never receive `complete`.
- `indeterminate`: the available evidence cannot establish whether the required population is complete. Absence of a feed is unknown, not zero.
- `reopen_required`: evidence captured after a prior close is valid for the closed reporting period and the close must be reopened or separately adjusted through an authorized append-only process.

A system may report multiple issues and a primary disposition, but it must not average a hard failure away. Preserve all population-level findings in the receipt.

## 3. Source-universe contract

The versioned close-scope manifest binds:

- legal/reporting entity and single-entity or consolidated boundary;
- reporting period, close cut-off and knowledge frontier;
- source population identifier, evidence class and source-system identity;
- authoritative inventory reference and evidence authority;
- period coverage and completeness test;
- materiality band, where defensible;
- required number of independent confirmations, where applicable;
- accountable owner and disposition;
- explicit not-applicable / out-of-scope rationale, evidence reference, reviewer role and approval time.

Authoritative inventories are control sources such as a bank-account register, supplier/customer master, entity consolidation perimeter, payroll roster, inventory-location register, related-party register, or independently obtained bank confirmation. A model-generated list is not an authoritative inventory. Each inventory's authority, period, capture time, identity and independence group must be explicit. Do not assume an inventory is exhaustive merely because it is labelled authoritative: where an independently sourced control is available, reconcile the inventories and preserve disagreements as population gaps. If the evidence universe rests on a single control source, the close receipt must disclose that limitation rather than imply independently verified exhaustiveness.

The candidate input schema is deliberately closed. Unknown fields are rejected rather than silently ignored. Public development fixtures are metadata-only synthetic examples: they are not byte-backed source snapshots and do not qualify cryptographic source integrity. A later runner must bind real fixture bytes to content digests and source capture receipts before making integrity claims.

## 4. Critical distinctions

### 4.1 Missing feed is not zero activity

A missing bank feed, payroll export, supplier statement, or inventory file is `unknown` or `unavailable`. It cannot be translated into zero transactions or zero liability.

A zero-activity conclusion requires affirmative evidence for the whole relevant period, from a source with appropriate authority. A count of zero in a manifest or a model statement is not enough.

### 4.2 Not applicable is not missing

A population may be classified `not_applicable` only when the supporting facts and period are evidenced and a named authorized role approves the disposition. Example: an HR roster plus a controller-approved attestation establishes that there were no employees during the period.

### 4.3 Out of scope is not omitted

A population outside a single-entity or consolidation boundary must be supported by evidence about the reporting perimeter and the relevant period. Scope exclusions are explicit records, not silent deletions from the manifest.

### 4.4 Two copies are not two independent sources

A mirror, cache, export, or downstream copy of the same root source belongs to the same independence group. Repeated copies can improve availability but do not create independent corroboration. Count independence groups, not files or connectors.

### 4.5 Late evidence preserves history

A source captured after close but valid for the closed period must be surfaced against the prior close receipt. The system may not silently rewrite the old journal or close. The disposition is `reopen_required` or an explicitly authorized adjustment/reopening workflow with preserved history.

### 4.6 The control inventory can itself be incomplete

A register is evidence about the expected population, not proof of its own completeness. The benchmark includes a case where an internal treasury bank-account register omits an account that appears in an independent bank confirmation. The candidate must use the union of independently supported in-scope populations, surface the conflict and refuse an unqualified close. When independent inventories disagree, the system may not silently pick the smaller universe or treat the internal register as the unquestioned denominator.

### 4.7 Balanced books can still be incomplete

Every public seed case deliberately reports a balanced trial balance. This prevents a candidate from using debit/credit balance as a proxy for evidence completeness.

## 5. Public development corpus

The v1.0.0 public development corpus contains 13 synthetic cases:

| Case | Challenge | Expected result |
|---|---|---|
| FCC-001 | Omitted bank account | incomplete |
| FCC-002 | Omitted supplier/AP population | incomplete |
| FCC-003 | Omitted customer/AR channel | incomplete |
| FCC-004 | Omitted subsidiary in a consolidated group | incomplete |
| FCC-005 | Missing payroll-liability population | incomplete |
| FCC-006 | Omitted inventory location | incomplete |
| FCC-007 | Omitted related-party population | incomplete |
| FCC-008 | In-period invoice arrives after close | reopen_required |
| FCC-009 | Mirrored bank feed falsely presented as independent | incomplete |
| FCC-010 | Missing bank feed incorrectly treated as zero | indeterminate |
| FCC-011 | No employees, affirmatively evidenced | complete |
| FCC-012 | Zero-activity bank statement, affirmatively evidenced | complete |
| FCC-013 | Subsidiary supported as outside reporting boundary | complete |
| FCC-014 | Independent bank confirmation reveals an account omitted from the internal control register | incomplete |

Candidate-visible inputs and evaluator-only labels are stored separately. The labels are included for reproducibility in this public development split; future hidden evaluation must use separately permissioned oracle artifacts. Do not train or prompt-tune on hidden labels.

## 6. Metrics and hard gates

Report each metric with its numerator, denominator, population, split, and missingness. Never publish only a composite score.

### Hard gates

- **Material completeness false-assurance rate = 0.** Numerator: material-omission cases declared complete. Denominator: all seeded material-omission cases.
- **Unknown-to-zero conversion rate = 0.** Numerator: absent/unavailable feed cases classified as zero activity without affirmative evidence. Denominator: cases with a required population and no qualifying observation/zero-activity evidence.
- **Independence inflation errors = 0.** Numerator: cases declared independently corroborated using only one independence group. Denominator: cases requiring multiple independent sources.
- **Late-evidence history violations = 0.** Numerator: cases where in-period late evidence is silently absorbed without reopening/authorized adjustment. Denominator: late-in-period evidence cases.
- **Control-inventory omission rate = 0** on seeded cross-inventory mismatch cases. Numerator: independently evidenced in-scope populations omitted from the candidate's expected universe. Denominator: all such populations in scored cases.
- **Unauthorized exclusion acceptance = 0.** Numerator: not-applicable/out-of-scope cases accepted without evidence and authorized approval. Denominator: exclusion cases.

A single critical hard-gate failure blocks a qualified completeness claim.

### Quality and operating metrics

- **Expected-population recall:** correctly identified authoritative in-scope population IDs / all authoritative in-scope population IDs.
- **Disposition exact match:** cases whose primary disposition equals the oracle disposition / scored cases.
- **Exclusion precision and recall:** correct exclusions / all proposed exclusions, and correct exclusions / all oracle exclusions.
- **Coverage by evidence class, entity, period and materiality band:** publish item-count denominators. Publish material-value coverage only when an independent, defensible control-total denominator exists.
- **Indeterminate rate and selective risk:** report accepted coverage alongside error rate; do not reward a system for guessing to maximize coverage.
- **Detection latency:** elapsed time from source-scope availability to gap identification; resolution time measured separately.
- **Human handling:** reviewer minutes, correction/rework, and escalation count per entity-period.
- **Economics:** fully loaded cost per correctly qualified entity-period, including model, connector, ingestion, deterministic verification, and human review.
- **Reliability:** repeated-run exact disposition stability and p50/p95/p99 latency.

## 7. Targets and comparison design

### Initial gates

- 100% correct disposition on this 14-case public development corpus.
- Zero material false assurances, zero unknown-to-zero conversions, zero independence inflation, zero unauthorized exclusions, and zero silent late-evidence history violations.
- 100% exact population-ID recall on this finite seed corpus, including the cross-inventory control.

These are corpus conformance targets, not field-performance claims.

### Research targets for a blinded expert-reviewed set

After expanding to at least 50 cases across multiple entity types and sectors, including source-inventory drift and disagreement:

- at least 95% expected-population recall, with confidence intervals reported;
- zero false-complete dispositions on seeded material-omission cases;
- at least 98% correct handling of positive controls (valid exclusion and affirmative zero activity);
- a pre-registered, statistically supported improvement over the strongest matched baseline on missing-population detection;
- at least 50% lower human handling time on the bounded routine-close cohort without increased material-error or exception leakage.

Do not claim SOTA from the public seed set. Use paired case-level comparisons and bootstrap confidence intervals; publish exact per-class errors and abstention/coverage trade-offs.

### Matched experimental arms

Use the same frozen source inventories, observed populations, reporting boundary, period, tool permissions and budgets:

1. deterministic rules-only completeness checker;
2. model-only reasoning without a deterministic verifier;
3. conventional accounting workflow plus model assistant;
4. evidence inventory + deterministic completeness verifier;
5. Symthaea read-only exception analyst + Mycelix verifier;
6. blinded human accounting professional, with independent adjudication.

Pin candidate versions, prompts, models, tool settings, random seeds, source snapshots and cost schedules. Candidate answers must include population IDs, source references, disposition, rationale, and explicit uncertainty. A language model must not grade primary accounting correctness.

## 8. External benchmark and industry positioning

FINBALANCE (Findings of EMNLP 2026) evaluates multi-document reconciliation and exact statement reconstruction. Its paper reports at most 46% exact final-balance-sheet accuracy across six evaluated LLMs and a 26–41 percentage-point gap for four models between their own statements and statements reconstructed from their journal entries. This supports independent reconstruction and source binding, but is not a completeness score.

DualEntry's October 2026 DualBench100 release reports 91.1% as its leading overall model score; its published area table shows material variation by task, including month-end close. It uses 101 synthetic tasks, deterministic reference grading and three runs. It is a useful practical comparison, but it does not establish that a system has enumerated every expected source population. Do not compare its overall score directly with this benchmark.

Industry accounting suites remain the ledger/workflow systems of record. Mycelix should integrate with their exports, bank feeds, supplier/customer masters, payroll systems, inventory systems and structured e-invoices rather than build a competing generic ERP. The differentiator to test is the cross-system evidence universe, independence-aware completeness, temporal reopening and independently reproducible close receipts.

References:
- FINBALANCE: https://aclanthology.org/2026.findings-emnlp.675/
- DualBench100: https://www.dualentry.com/benchmark

## 9. Architecture boundaries

- **Mycelix Finance:** owns versioned evidence-population manifests, source authority, evidence qualification, completeness dispositions and close receipt references.
- **Accounting projection/journal boundary (#1123):** remains the only authority for journal creation and posting.
- **FIN-ACC benchmark (#5782):** composes task scoring; it does not create a ledger.
- **Rust reference verifier (#5789 / PR #5791):** remains the independent arithmetic/source-binding smoke verifier. Completeness is a separate contract to integrate after its base PR is reviewable; do not duplicate its schema, digest, receipt or canonical amount machinery.
- **Symthaea #10422:** may suggest likely missing sources and explain exceptions read-only. It cannot authorize completeness or post journals.
- **Symtropy:** may provide operational corroboration (e.g. inventory or production events), never accounting truth by simulation alone.

## 10. Claim ceiling

Passing this benchmark establishes only that a candidate handled the declared synthetic population and temporal cases correctly. It does not prove that every real-world liability, bank account, supplier, subsidiary or transaction has been discovered; that a source is authentic; that accounting policy is correct; that books are audit-ready; or that the system complies with IFRS, tax law or any regulator's requirements. A field claim requires rights-cleared records, security and privacy review, professional adjudication, pre-registration and prospective evaluation.
