# Mycelix RATS Trusted-Time Binding v0.1

**Status:** executable reference-model composition  
**Claim ceiling:** `ReferenceModelOnly`

This artifact binds RATS freshness decisions to the existing EVID-TIME semantic boundary instead of treating a host wall clock as an authoritative time source.

## Core theorem

```
wall clock
!= authenticated time observation
!= policy-trusted time
!= freshness
!= current authorization
```

The underlying EVID-TIME structural crate deliberately represents time as a closed interval and keeps structural observation separate from source verification and policy trust. This qualifier consumes the later semantic positive `PolicyTrustedTimeInterval` conceptually; it does not implement an NTS, Roughtime, TPM, or hardware-clock provider.

## Conservative interval rule

Suppose trusted now is:

```
[earliest, latest]
```

and an Attestation Result is valid for:

```
[issued_at, expires_at)
```

The qualifier reasons over the entire interval.

Therefore:

- if every possible trusted-now value is before issuance, the result is **DENY** as future-dated;
- if every possible trusted-now value is at/after expiry, the result is **DENY** as stale;
- if the interval crosses issuance or expiry, the result is **INDETERMINATE**;
- if some possible trusted-now values satisfy the maximum-age limit and others do not, the result is **INDETERMINATE**;
- a favorable subset of the uncertainty interval may never be selected merely to obtain PASS.

This makes clock uncertainty security-relevant rather than cosmetic.

## Request binding

The trusted-time observation is request-bound through an exact commitment reference. A time value from another request cannot silently satisfy this request's freshness theorem.

The profile also retains the RATS nonce and audience bindings. Nonce validity alone does not convert ambiguous temporal evidence into PASS.

## Qualification corpus

17 deterministic vectors cover:

- canonical validity;
- time entirely after expiry;
- time entirely before issuance;
- expiry ambiguity;
- maximum-age ambiguity;
- unavailable time source;
- source-profile substitution;
- trust-domain substitution;
- verifier-profile substitution;
- excessive uncertainty;
- local-clock masquerading;
- nonce-valid temporal ambiguity;
- fresh nonce with stale time;
- missing time evidence;
- missing request binding;
- future issuance inside uncertainty;
- reversed intervals.

## Boundary with EVID-TIME-001A

The existing structural time crate proves only structural compatibility and intentionally leaves cryptographic verification, trust, replay resistance, freshness, and currentness false. This profile therefore does not weaken that boundary by calling the structural object `trusted`.

## Qualification ceiling

A green result establishes deterministic interval reasoning for the named synthetic profile only.

It does not establish:

- trusted UTC or universal time;
- NTS/Roughtime/provider correctness;
- TPM or secure-clock correctness;
- hardware security;
- current revocation state;
- runtime authorization;
- CMMC/classified authorization;
- FIPS validation;
- deployment readiness.

The next concrete step remains a separate provider-qualified time adapter, followed by a real TPM/attestation Evidence adapter.
