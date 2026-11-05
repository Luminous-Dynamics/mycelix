# Global Accounting Portability Benchmark v1.0.0

Public synthetic development package for [Mycelix #5811](https://github.com/Luminous-Dynamics/mycelix/issues/5811).

## Files
- profile_reference_v1.0.0.json: reference inventory and maturity matrix.
- profile_reference_v1.0.0.schema.json: closed Draft 2020-12 schema with explicit qualification-intersection records.
- qualification_intersections: exact profile/jurisdiction/entity/time combinations; only one synthetic positive-control intersection exists.
- global_portability_input_v1.0.0.json: candidate-visible synthetic cases.
- global_portability_input_v1.0.0.schema.json: closed candidate-input schema.
- global_portability_oracle_v1.0.0.json: evaluator-only behavior labels; do not pass to candidate systems.
- global_portability_oracle_v1.0.0.schema.json: closed oracle schema.
- Locale/time reference pins: Unicode CLDR 49 and IANA tzdb 2026f.
- ../../docs/GLOBAL_ACCOUNTING_PORTABILITY_V1.md: architecture, benchmark, targets and claim ceiling.

## Scope
The 22 fixtures test global context resolution and safe interoperability. They use synthetic jurisdictions, currencies, tax profiles and framework aliases. They encode no tax rate, legal conclusion or statutory rule. Locale/time cases pin Unicode CLDR 49 and IANA tzdb 2026f as reference releases only. The IFRS for SMEs 2025 effective-date fact is a sourced reference, not an executable accounting rule.

## Honest coverage status
The profile catalog lists 22 profile references. Real standards and infrastructure are research/reference inputs only. One synthetic-only positive-control intersection exercises the resolver; zero real jurisdiction + framework + tax + invoice + reporting combinations are qualified. One exact synthetic-only intersection exists solely to exercise the resolver's positive path; it is not a real-world qualification. South African initial research remains review-required; see Mycelix #5785 and #5792.

## Candidate/evaluator separation
Candidates receive the input JSON only. The oracle is public because this is a public development split, not a hidden benchmark. Future hidden splits require access controls and independent expert labels. Do not tune against hidden labels.

## Validation
A the standalone Rust resolver/evaluator must:
1. run full Draft 2020-12 validation over input, oracle and profile catalog;
2. reject duplicate case/profile IDs and unknown fields;
3. score required and prohibited behaviors deterministically;
4. check exact currency arithmetic and temporal semantics without floating point;
5. report correct abstention separately from incorrect completion;
6. publish per-family results and declared profile scope, not a global average that masks critical errors.

No test, compile, CI, legal, tax or accounting-standard qualification is implied by these authored fixtures.
