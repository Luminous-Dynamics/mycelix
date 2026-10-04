# Mycelix Policy Enforcement Point Decision Boundary v0.1

**Status:** executable reference-model composition  
**Claim ceiling:** `ReferenceModelOnly`

This artifact is the final decision join between independently appraised security dimensions and a runtime enforcement point.

It is intentionally pure and side-effect-free.

## Decision algebra

The PEP consumes closed input states:

```
PASS
DENY
INDETERMINATE
```

and combines them using:

```
ANY DENY
    >
ANY INDETERMINATE
    >
ALL PASS
```

Therefore:

- one required denial cannot be overridden by positive evidence;
- uncertainty cannot be converted into permission without an explicitly qualified degraded-mode profile;
- only all-required positive inputs produce `PASS`.

## Inputs

The v0.1 join covers:

- subject identity;
- device posture;
- workload identity;
- security domain;
- policy version;
- freshness;
- purpose;
- resource authorization;
- releasability;
- export control;
- delegation.

These values are results from other qualified boundaries. The PEP does not pretend to validate a TPM quote, time provider, identity credential, export-control policy, or resource database itself.

## Evidence does not bypass local policy

The critical negative theorem is:

```
valid attestation
+ invalid local resource policy
-> DENY
```

Likewise:

```
valid release receipt
+ current local authorization failure
-> DENY
```

and:

```
network reachability
+ no resource authorization
-> DENY
```

The PEP therefore prevents trust evidence from becoming a universal bearer authorization.

## No reusable bearer output

The v0.1 decision is a bounded evaluation result.

It does not create:

- a bearer capability;
- a portable authorization token;
- a reusable release receipt;
- a new identity;
- an implicit cacheable authorization.

A later runtime-specific implementation may need a narrowly scoped lease, consumable permit, or state transition. Those mechanisms require their own qualifications.

## Determinism

The evaluator has no ambient clock, network, filesystem authority, or side effects.

Identical frozen inputs must yield the identical decision.

Input map/member ordering does not affect semantics.

## Adversarial corpus

24 vectors cover:

- each individual authorization dimension failing;
- freshness denial and ambiguity;
- valid attestation with revoked local policy;
- valid release receipt without current local authorization;
- network reachability without resource authorization;
- DENY + INDETERMINATE precedence;
- all inputs INDETERMINATE;
- workload/device mismatch;
- attempted bearer-token promotion;
- repeat determinism;
- input-order permutation;
- unknown input state;
- attestation-result-alone denial.

## Qualification ceiling

A green run establishes deterministic fail-closed composition for this reference model.

It does not establish correctness of the input evidence, the enforcement mechanism, kernel isolation, hardware security, cryptographic-module validation, CMMC/classified authorization, or production readiness.
