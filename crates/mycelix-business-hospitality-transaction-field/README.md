# Mycelix Business Hospitality Transaction Field

`mycelix-business-hospitality-transaction-field` is a read-only evidence composition layer for transaction-level hospitality forecast qualification.

It closes two major evaluation loopholes:

1. the model cannot choose the forecast windows after seeing outcomes; and
2. the evaluator cannot supply the actual outcome used to score the model.

The path is:

`registered protocol -> exact target plan -> exact source files -> canonical campaign replay -> preregistered aggregation -> derived actuals -> model scoring -> transition-aware slices -> field-quality gate`

## Inputs

The evaluator accepts only:

- the preregistered hospitality pilot registration;
- a preregistered transition-aware time schedule;
- the exact forecast target plan;
- the preregistered actual-projection specification;
- the read-only adapter and extraction campaign;
- the exact campaign files;
- candidate/baseline forecast submissions for every planned target; and
- additional unresolved limitation identifiers.

There is no caller-supplied raw actual, derived actual, slice verdict, or field-quality count.

## Derived actuals

Transaction observations are replayed through the canonical import path and aggregated over half-open `[start, end)` target windows. `DerivedMetricActual` remains a distinct type from `MetricObservation`; deterministic inference never promotes itself into raw observation truth.

Empty planned windows produce evidence-backed zero actuals rather than disappearing from the denominator.

## Field quality

The v0.1 field evidence is conservative:

- parser/normalization rejections count as missing;
- duplicate source-event rejections count as conflict evidence;
- future-dated accepted rows fail the evaluation;
- the exact maximum observed ingestion delay is preserved; and
- if that maximum exceeds the registered freshness ceiling, the full accepted set is conservatively marked stale rather than guessing how many rows were late.

## Remaining limitations

A strong report always preserves:

- `limitation:upstream-export-completeness-unverified:v1` — exact replay proves what was in the export, not that the provider exported every real transaction;
- `limitation:time-rule-source-authority-unverified:v1` — the local-time schedule is digest-bound, but this layer does not certify the external timezone/jurisdiction source; and
- `limitation:aggregation-semantic-authority-unverified:v1` — deterministic `Sum` semantics do not by themselves certify that the provider mapping means the intended economic quantity.

The weaker fixed-offset and direct-actual-membership limitations are rejected rather than carried forward.

## Non-claims

Passing this layer does not grant execution authority, prove causal financial benefit, certify a POS provider, establish legal/accounting truth, or prove upstream export completeness.
