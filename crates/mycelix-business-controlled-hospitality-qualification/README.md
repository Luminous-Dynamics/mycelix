# Mycelix Controlled Hospitality Qualification

`mycelix-business-controlled-hospitality-qualification` creates an additive evidence envelope around an existing transaction-derived hospitality report.

It never rewrites the original report, scorecard, slice evidence, field decision, or model verdict.

## Required bindings

The envelope verifies that the original report, pilot registration, target plan, transaction projection and full control-coverage evidence all refer to the same:

- registration and shadow protocol;
- connector/schema/mapping lineage;
- campaign/replay lineage;
- sales input;
- business scope;
- metric, unit and fixed-point scale;
- `Sum` transaction projection; and
- complete evaluation interval.

The control-coverage plan must have been preregistered no later than the hospitality pilot registration.

## Limitation evolution

The original transaction report remains immutable and retains its original broad limitations.

When complete control coverage validates, a new envelope replaces only:

- `upstream-export-completeness-unverified` with `control-source-external-reality-unverified`; and
- `aggregation-semantic-authority-unverified` with `control-metric-semantic-authority-unverified`.

It additionally records `control-source-authenticity-unverified` and preserves unrelated limitations such as time-rule provenance.

The before/after limitation sets, exact qualification transition digests, original report digest and control-coverage digest are all bound into one envelope digest.

## Non-claims

This envelope does not turn a failed model into a passing model, does not establish physical truth, does not authenticate the provider, and grants no business execution authority.
