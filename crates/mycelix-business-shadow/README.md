# mycelix-business-shadow

`mycelix-business-shadow` is a read-only qualification harness for Business Fabric intelligence.

It exists to answer a narrow question before any autonomous business authority is considered: **does a model or recommendation process produce useful, calibrated evidence under a preregistered evaluation protocol?**

The crate cannot express an execution mode. Shadow capabilities are limited to observation, estimation, forecasting, and recommendation. It mints no authority, creates no `PreparedAction`, reserves no scarce state, and calls no external system.

## Core rules

- external adapters are witnesses, not truth sources;
- raw provider payloads are represented by digests rather than copied into the qualification layer;
- a provider event identity cannot silently change payload content;
- forecast issuance must precede the target window, preventing outcome leakage;
- evaluation protocols are registered before their evaluation window;
- candidate forecasts are compared against an explicit baseline on the same cases;
- abstention is first-class and cannot improve measured error by dropping hard cases: an abstention conservatively receives the baseline error for that case;
- recommendation review is evidence about usefulness/safety, not proof of causal business impact;
- shadow qualification never grants authority or claims field qualification.

## Promotion discipline

A passing shadow scorecard may justify continued A0-A2 evaluation (`observe`/`explain`/`recommend`). It is not evidence for A3-A5 execution authority. Any later write-capable integration must separately satisfy Action Contract, authority, coordination, revalidation, execution receipt, reconciliation, and field-qualification requirements.

The harness deliberately does **not** compute hypothetical savings from recommendations that were never executed. Counterfactual economic benefit requires a separate causal design rather than being inferred from shadow predictions.
