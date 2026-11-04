# Financial Close Completeness Benchmark v1.0.0

This directory contains a synthetic, public-development corpus for the completeness extension tracked in [Mycelix #5790](https://github.com/Luminous-Dynamics/mycelix/issues/5790).

## Files

- `close_completeness_input_v1.0.0.json`: candidate-visible synthetic inputs.
- `close_completeness_input_v1.0.0.schema.json`: closed Draft 2020-12 input schema.
- `close_completeness_oracle_v1.0.0.json`: evaluator labels; do not pass this file to candidate systems.
- `close_completeness_oracle_v1.0.0.schema.json`: closed oracle schema.
- `../../docs/FINANCIAL_CLOSE_COMPLETENESS_V1.md`: protocol, disposition semantics, metrics, hard gates, targets and claim limits.

## Seed corpus

Thirteen cases cover bank, AP, AR, consolidation, payroll, inventory, related-party, late-arriving evidence, duplicate mirrors, missing-feed-versus-zero, valid non-applicability, affirmative zero activity and supported scope exclusion.

All cases have balanced debit/credit totals by construction. Seven material omissions plus the source-independence and missing-feed controls ensure that ledger balance alone cannot earn a complete disposition.

## Important limits

- All data is synthetic. The input set is a public development split, not a hidden test set or evidence of product performance.
- These inputs are metadata-only examples. They do not include raw source bytes or validated SHA-256 snapshots; no source-integrity claim may be drawn from them.
- Oracle labels are separated from candidate inputs in the file layout. The public development labels are visible by design; keep future hidden evaluation artifacts separate and access-controlled.
- Not-applicable and out-of-scope statuses require positive evidence and an authorized reviewer. Missing evidence is never zero activity.
- No financial statements, journals, production postings, legal interpretation or audit opinion are produced by this corpus.

## Validation

A future Rust evaluator should:
1. validate both files against their respective full Draft 2020-12 schemas;
2. reject unknown fields and duplicate IDs;
3. derive expected populations from authoritative inventory evidence rather than trusting the scope manifest alone;
4. count independent evidence groups, not mirrors or duplicate files;
5. preserve close-time and capture-time semantics;
6. emit case-level findings and a deterministic machine-readable report.

Do not use an LLM judge for population IDs, disposition, arithmetic, authority, or temporal correctness.
