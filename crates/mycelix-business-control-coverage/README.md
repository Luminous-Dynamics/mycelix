# Mycelix Business Control Coverage

`mycelix-business-control-coverage` proves that an exact preregistered set of matching control-total windows covers a larger qualification interval contiguously.

It exists because one matching daily control report can support only one exact scope/time window. It must not silently narrow a limitation attached to a week, month, or full field pilot.

## What the plan freezes

A `ControlCoveragePlan` fixes before the qualification interval:

- connector/schema/mapping identity;
- source input, metric, scope, unit and scale;
- aggregation semantics;
- control-source identity and class;
- qualification start/end; and
- the exact ordered set of control windows.

The windows must cover the full qualification interval with no gaps or overlaps.

## What evidence must prove

Every planned window must have exactly one matching reconciliation under the same frozen semantics. A mismatch in even one window blocks full-interval coverage.

Each `WindowCoverageEvidence` is independently self-consistent: it binds its exact start/end bounds, contract, statement, reconciliation, exactly two distinct scoped-transition digests, and its own receipt digest. The outer coverage digest is accepted only after every window receipt validates.

## Qualification-wide limitation transitions

Only complete coverage may emit qualification-interval transition candidates:

- `limitation:upstream-export-completeness-unverified:v1` -> `limitation:control-source-external-reality-unverified:v1`
- `limitation:aggregation-semantic-authority-unverified:v1` -> `limitation:control-metric-semantic-authority-unverified:v1`

These transitions remain bound to the exact scope and qualification interval. They do not prove physical truth, provider authenticity, legal/accounting correctness, or business execution authority.

## Safety

This crate is read-only. It grants no authority, owns no business state, and contains no provider mutation surface.
