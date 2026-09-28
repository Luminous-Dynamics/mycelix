# Economic OS Policy Kernel v1

The kernel owns semantic invariants; profiles own domain-specific policy.

## Kernel

The kernel guarantees:
- evidence does not silently become entitlement;
- policy is explicit and versioned;
- authority is explicit before effects;
- settlement eligibility is distinct from authorization;
- current evidence is required for current settlement;
- claim ceilings cannot widen during projection;
- foreign recognition does not imply local issuance;
- corrections preserve history;
- retries preserve logical economic identity.

## Profiles

### Integral ITC

Defines the rules by which eligible contribution evidence can produce an ITC-specific entitlement. The kernel does not define the ITC formula.

### Valueflows

Provides an external ontology mapping for Intent, Commitment, Claim and EconomicEvent. Valueflows describes EconomicEvent as an actual flow and `corrects` as a relationship that leaves the original event unchanged. [Valueflows specification](https://www.valueflo.ws/specification/all_vf/)

### Mutual credit

Defines issuer, credit limits, backing/obligation policy, issuance and settlement rules.

### TEND

Defines the domain-specific instrument and policy. The kernel does not turn TEND into a universal currency.

### Accounting projection

Produces derived accounting representations. It cannot promote a derived record into source physical evidence.

## Monotonic profile rule

A profile may restrict the generic kernel. It may not weaken it.

```text
profile_transition ⊆ kernel_transition
```

A profile cannot define:

```text
evidence → settlement
observation → authority
recognition → local issuance
accounting → physical evidence
```

## Integral interoperability implication

Integral's public technical specifications currently mark the OAD→COS, COS→ITC and FRS→CDS interface contracts as pending, including versioning, authentication, retry/idempotency and delivery semantics. [Integral Technical Specifications](https://integralcollective.io/documents/specifications.html)

The Economic OS should therefore act as a semantic guard at those seams, not pretend that the underlying Integral API contracts are already ratified.

## Claim ceiling

This is a policy-kernel conformance design, not evidence that any economic profile is effective, fair, legally valid, financially compliant, or production-ready.