# MYC-CAP-002A — Fixed Preferred Return Envelope v1

Status: executable candidate; qualification is exact-head CI evidence, not this document.

## Purpose

This profile is the first deliberately narrow executable Capital-to-Commons accounting theorem. It reconstructs one financial claim state from one frozen project profile and one ordered event history.

It does not implement general project finance, IRR, indexed yield, legal title, democratic governance, asset condition, tax, securities, or accounting-standard semantics.

## Frozen financial semantics

The initial maximum investor entitlement is:

```text
initial principal
+ fixed preferred-return cap
```

Only two event classes can increase that entitlement after genesis:

```text
QualifiedNewCapital
ApprovedRecoverableLifecycleCost
```

Both are bounded by explicit profile caps.

The claim is retired by:

```text
InvestorDistribution
PrincipalRedemption
Impairment
```

Therefore:

```text
created entitlement
- retired claim
= remaining claim
```

`GrantOrSubsidy` is recorded but cannot increase the investor claim under this profile.

There is no time field and no implicit compounding. Time passing cannot silently grow the claim. Indexed or time-dependent return belongs in a separate future profile.

## Exact event lineage

Every event binds:

- exact project ID;
- exact profile SHA-256;
- exact contiguous sequence number;
- unique event ID;
- previous-event SHA-256;
- event kind and integer amount;
- authority and evidence references.

This creates a profile-bound hash chain. Mutation, insertion, deletion, reordering, project substitution, and profile substitution fail closed.

The references are evidence pointers only. v1 does not authenticate their external source.

## Reserve precedence

Investor distributions and principal redemptions are forbidden while the frozen required reserve is underfunded.

This is a narrow executable expression of the Capital-to-Commons rule that maintenance/lifecycle resilience ranks ahead of discretionary investor extraction. It is not a complete infrastructure reserve model.

## Financial no-op lineage events

The following events are recorded but cannot change the claim in v1:

```text
RefinanceReplaceClaim
OperatorChange
ControlChange
TransitionCheckpoint
```

They must carry zero amount.

This proves, in the narrow profile, that a refinance/operator/control event cannot itself reset the return envelope.

## Positive result boundary

`RETURN_ENVELOPE_SATISFIED` means exactly:

```text
remaining_claim_units == 0
```

It does not mean:

```text
legal title transferred
handback accepted
community governance valid
asset safe
operator replaced
accounting treatment correct
```

The receipt therefore fixes `legal_transition_complete=false` and `handback_accepted=false`. Those require separate evidence/authority profiles.

## Integer authority arithmetic

All monetary quantities are non-negative integer units under one profile-defined `unit`. Floating point is forbidden. The profile freezes a hard maximum amount and event-count bound.

## Future profiles deliberately excluded

Separate theorems should own, rather than silently extend this profile:

- indexed/inflation-linked preferred return;
- bounded IRR/time-value semantics;
- multiple seniority waterfalls;
- secondary claim ownership and securitization;
- DFI/public guarantee effects;
- handback condition and residual useful life;
- legal redemption/title transition;
- community governance activation.

## Nonclaims

PASS proves arithmetic and state-machine consistency for the supplied frozen profile/history only. It does not establish the truth of external accounting entries, legal enforceability, investor suitability, solvency, infrastructure performance, democratic legitimacy, or public-interest optimality.
