# mycelix-business-decision

`mycelix-business-decision` preserves evidence-bearing lineage across business decisions without owning authority, business-domain state, external-system truth, or execution.

Its purpose is to keep these stages distinct:

`evidence -> decision -> authorization -> execution attempt -> provider acknowledgement -> economic reconciliation -> outcome`

A provider acknowledgement is not an economic-success claim. A recommendation is not authorization. An execution attempt is not proof that an external effect occurred. Unknown external outcomes remain unknown until reconciliation evidence establishes otherwise.

The crate also keeps generated human explanations non-authoritative and provides retry guidance that defaults to reconciliation before any potentially duplicative retry.

## Boundaries

- no authority minting;
- no domain-state ownership;
- no implicit promotion of inference to observation;
- no inference from HTTP/provider acknowledgement to economic finality;
- compensation preserves the original attempt and links a distinct compensation attempt;
- outcome/calibration records bind back to the exact decision and reconciliation lineage.

This crate is a contract surface only. Passing its tests does not establish field qualification or legal/economic correctness of a real integration.
