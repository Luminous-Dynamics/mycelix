# SYM-CIVIC-007 — verification-result and policy provenance v1

Status: synthetic research provenance qualification only

Parent: qualified SYM-CIVIC-006 / `00156dc7b7f4d2709e6563db0e7b84d31c4a4f8a`

Tracking issue: #3853

## Purpose

Test whether a verification result is itself reproducible provenance rather than an opaque assertion of authority.

The conceptual separation is:

`AttestationAuthenticity != VerificationResult != VerificationPolicy != ScientificTruth != CivicAuthority`

The profile is informed by the current in-toto Simple Verification Result (SVR) predicate, which records a verifier, policy references, a verification time, and verified properties. This benchmark extends that vocabulary with explicit policy identity/version/scope and historical validity. It is alignment research, not a replacement for the in-toto specification.

## Contract

A verification result is accepted as provenance only when:

- its exact subject set is bound by immutable digest;
- verifier identity is explicit and materially versioned;
- every referenced policy has an immutable identity/digest rather than a mutable locator;
- the exact policy digest evaluated is the digest recorded by the result;
- the evaluated policy version and declared scope match the result;
- policy validity is evaluated at the stated verification time;
- every reported property is within the declared policy scope;
- verification time is explicit and cannot be backdated against the verifier execution context;
- replay against a different subject identity is rejected;
- reproducibility claims include the policy bundle or explicitly record an empty policy set;
- if a policy dependency changes, the verification result receives a new immutable identity;
- a result may faithfully report an uncertain or failed property without promoting it into scientific truth or civic authorization.

Historical verification is admissible only when the result identifies the historical policy version that was valid at its recorded verification time.

## Typed dispositions

- `REJECT_VERIFICATION_PROVENANCE`: the verifier, subject, policy, scope, temporal, or reproducibility binding fails.
- `VERIFIED_CONTENT_UNQUALIFIED`: verification provenance is intact, but the reported property is explicitly uncertain or failed; no scientific truth is inferred.
- `VERIFIED`: provenance is intact and the result reports a verified property, without any inference of scientific truth or civic authorization.

## Adversarial corpus

V-01 verification subject digest mismatch
V-02 unknown verifier
V-03 verifier version omitted
V-04 mutable latest policy locator
V-05 policy digest mismatch
V-06 evaluated policy version differs from referenced version
V-07 policy invalid at verification time
V-08 reported property outside policy scope
V-09 verification result replayed against a different subject
V-10 verification timestamp backdated against execution context
V-11 policy bundle omitted while reproducibility is claimed
V-12 policy dependency changed without new result identity
V-13 authenticated verification with uncertain property
V-14 authenticated verification reporting failed property without authorization
V-15 exact subject/verifier/policy/time/property binding
V-16 explicit empty policy set
V-17 historical verification with explicit historical policy version
V-18 scoped property with exact policy version
V-19 mutable policy URI plus digest mismatch
V-20 multi-subject set silently narrowed
V-21 result identity changed without policy/subject provenance change
V-22 authenticated verification with unsupported property

The qualifier derives dispositions from the semantic contract; fixture files contain no expected verdicts.

## Qualification ceiling

PASS establishes only that this synthetic corpus preserves provenance of verification results and keeps verifier/policy evidence distinct from scientific truth, authorization, and civic authority.

It does not establish policy correctness, verifier competence, production cryptographic security, scientific correctness, legal or civic legitimacy, or deployment readiness.

No runtime implementation is proposed.
