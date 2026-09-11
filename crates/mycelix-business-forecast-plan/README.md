# mycelix-business-forecast-plan

Preregistered exact target coverage for Mycelix Business shadow qualification.

A forecast model must not improve its apparent quality by choosing only easy windows after seeing outcomes. This crate freezes the exact forecast target set before evaluation and proves that the submitted case set covers every planned target exactly once.

## Guarantees

A target plan binds:

- the exact shadow protocol digest;
- profile and capability;
- candidate and baseline model lineages;
- evaluation window;
- preregistration time; and
- a canonical list of exact target identities/windows.

The plan must be frozen no later than the shadow protocol itself.

Targets are validated to be inside the evaluation window, semantically unique, and non-overlapping for the same metric/scope. Coverage verification rejects omitted targets, replacement with unplanned windows, duplicate case weighting, lineage drift, and forecasts issued before the protocol existed.

Candidate abstention does **not** remove a target from the denominator: the case still has to exist and remains subject to the shadow scorer's conservative abstention treatment.

## Boundary

This crate proves only target preregistration and exact coverage. It does not establish actual-observation provenance, score model quality, authorize an action, or execute a business write.
