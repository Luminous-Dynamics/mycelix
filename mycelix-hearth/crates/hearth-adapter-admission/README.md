# hearth-adapter-admission

HTH-AUTO-004D freezes the pure admission contract that must sit between a semantic Hearth plan and any real device/service adapter.

The core rule is:

> An adapter claiming `supports(action)` is not authority to execute that action.

A physical/digital adapter must be admitted under an exact reviewed profile that constrains the semantic surface it may serve.

## Profile binding

Each `AdapterProfile` binds:

- stable adapter identity;
- exact implementation reference;
- exact profile reference;
- transport class;
- one or more exact action rules.

Each `ActionRule` constrains:

- capability + operation;
- target entity kinds;
- whether the operation may be a primary action and/or compensation;
- minimum and maximum consequence class;
- allowed reversibility classes;
- timeout and retry ceilings;
- verification window, confidence, independent-source, and freshness floors;
- typed argument names and value/range constraints;
- whether unknown arguments are forbidden.

The profile contains references and semantics only. It never contains device credentials or secrets.

## Deterministic selection

For every plan step, admission searches the supplied qualified profile set.

```text
0 matching profiles -> NoQualifiedAdapter
1 matching profile  -> exact AdapterAdmission
2+ matches          -> AmbiguousQualifiedAdapters -> REJECT
```

Registration order is never a tie breaker.

Compensation is independently admitted. A valid primary action does not authorize an unrestricted compensation action.

## Why consequence is checked here

`ActionSpec` deliberately contains semantic capability/target/operation, while consequence and reversibility live on `PlanStep`. Adapter admission therefore evaluates the **whole step**, preventing a dangerous operation from being made executable merely by labeling its `ActionSpec` as something an adapter supports.

Example:

```text
home.access.unlock
profile minimum consequence = CriticalAct
plan declares consequence = ReversibleAct
-> REJECT
```

## Typed argument constraints

Profiles may require exact deterministic ranges/types such as:

- booleans;
- signed/unsigned integer ranges;
- basis-point ranges;
- fixed-point values at one exact scale/range;
- bounded text;
- enumerated text;
- bounded text lists.

Unknown argument keys are rejected by default. No floating-point policy values are introduced.

## Relationship to the edge runtime

This crate is intentionally pure and side-effect free. HTH-AUTO-004D freezes and tests the admission theorem first.

A later integration tranche must make `hearth-edge` consume the resulting exact admissions before dispatch, bind the actual registered adapter implementation/profile to the admission, and route real hardware only through the HTH-AUTO-004C exclusive physical wrapper.

## Nonclaims

This tranche does not:

- prove any adapter implementation satisfies its profile;
- connect to Matter, Home Assistant, MQTT, OCPP, OpenADR, or vendor services;
- authenticate an implementation/profile reference;
- hold device credentials;
- grant household authority;
- replace the AUTO-002 policy kernel;
- execute a side effect;
- qualify remote/network transports.

Adapter implementation qualification and runtime enforcement remain separate gates.
