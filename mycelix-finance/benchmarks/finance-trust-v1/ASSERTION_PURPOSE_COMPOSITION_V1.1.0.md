# Assertion-purpose composition for FIN-TRUST v1.1.0

**Canonical owner:** FIN-ACC-EVIDENCE-001 / #5798 and [PR #5836](https://github.com/Luminous-Dynamics/mycelix/pull/5836)  
**Additional temporal owner:** #5792 (bitemporal policy and knowledge frontier)  
**Consumer:** FIN-TRUST-BENCH / #5857  
**Claim ceiling:** compatible synthetic conformance projection only

## Why FIN-TRUST v1.1.0 changes

v1.0.0 treated a candidate's `qualifies` boolean as a compact assertion that a source was usable. That was too coarse: it could not express *why* an artifact should or should not support a specific assertion and procedure purpose. A bank debit may be valid evidence that a debit was observed while being irrelevant to whether the beneficiary received funds. A copied invoice can preserve the original bytes while adding no independent corroboration. A signed document can be authentic without its issuer having authority for the assertion being tested.

v1.1.0 removes the self-attested binary `qualifies` field. It requires a typed evidence profile per source / requirement binding and a separate disposition for every requirement.

## Vocabulary crosswalk

The v1.1.0 task and response schemas use the evidence dimensions already defined by the canonical assertion-purpose contract in #5836:

| FIN-TRUST v1.1 field | Canonical assertion-purpose meaning |
|---|---|
| `assertion_kind` | Assertion category being evaluated |
| `required_obligations` | Procedure obligations that must be met for a supported disposition |
| `min_independent_origins` | Minimum number of distinct upstream origins when independent corroboration is required |
| `evidence_class` | Document/evidence class, not a universal trust tier |
| `polarity` | Supports, contradicts, neutral, duplicate or derived |
| `purpose_fit` | Whether this evidence is relevant to this particular assertion and procedure purpose |
| `source_authentication` | Whether the source itself is authenticated |
| `source_authority` | Whether the issuer/source has authority for the assertion |
| `temporal_fit` | Whether the evidence is valid for the reporting period |
| `transformation_state` | Whether original bytes are preserved and transformations have lineage |
| `origin_group_ref` | Shared upstream origin used to avoid false independence |

The additional `recorded_at` field is composed with the task's `knowledge_at` frontier. Period validity and when evidence became known are different dimensions; a late-recorded artifact must not leak into a historical reconstruction simply because its valid date is in the period.

## Authority boundary

- **#5836 remains the canonical owner** of assertion-purpose semantics and its full evaluator. FIN-TRUST does not create a competing global reliability score or replace that verifier.
- FIN-TRUST is an interaction benchmark. It measures whether the candidate asks the right clarification, directs it to the correct recipient, preserves unknown/contradictory evidence, resumes from a new evidence frontier, and then emits only the supported journal and statement result.
- The FIN-TRUST verifier performs narrow positive-disposition guards: it will not accept a `supported_within_declared_scope` result when its declared obligations are visibly unmet, when independent origins collapse to one upstream source, when a population is incomplete, or when the evidence is outside entity/currency/time/knowledge/lineage constraints.
- The per-requirement expected dispositions and reason codes are adjudicated in the FIN-TRUST public S0 oracle. The separate canonical assertion-purpose corpus remains independently versioned and qualified.

No dimension is collapsed into one confidence or reliability number. Authenticity, authority, purpose fit, polarity, timing, transformation history, population completeness and independent origins are different claims.

## v1.1.0 additional invariants

1. Every required accounting assertion/purpose pair has one explicit disposition and reason-code set.
2. A supported requirement cites evidence IDs that are bound to exact source ID, revision, SHA-256, locator and recorded time.
3. A positive disposition requires all declared obligations, including authorization, transformation lineage, independent corroboration and population completeness where applicable.
4. A duplicate or mirror from the same origin cannot satisfy an independent-corroboration obligation.
5. Qualified contradictory evidence cannot be relabelled as support.
6. An artifact outside the historical knowledge frontier cannot support a historical conclusion.
7. A journal must cite the exact expected source IDs and only decision-grade evidence associated with a supported requirement.
8. A statement is independently reconstructed from the accepted journal output; a balanced journal alone is not a complete-books claim.

## Research boundary

The vocabulary is designed to be compatible with FIN-ACC-ASSERTION-PURPOSE v1.0.0. It is not a claim of ISA 500 compliance, audit sufficiency or a professional audit conclusion. The public FIN-TRUST corpus remains a ten-task seed; the issue's S0 acceptance target remains 50 tasks. Blind model evaluation is still owned by #5799.
