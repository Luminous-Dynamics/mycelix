# South African ShapID and Alias Lifecycle Qualification v1

**Status:** synthetic reference contract; no live directory connector, telecom lookup, bank integration or payment dispatch.

## Research finding

A ShapID based on a cellphone number is a temporal alias, not a permanent person or bank-account identity. Nedbank's public FAQ describes a primary cellphone-number ShapID, bank-qualified aliases, a requirement to deregister a default ShapID at another bank before making the number default at Nedbank, and automatic deletion of a ShapID that no longer matches the primary cellphone number at end of business day. Standard Bank documents primary versus bank-qualified aliases and edit/delete operations.

ICASA's 2 July 2026 announcement describes final Fourth Amendment Numbering Plan Regulations standardising inactive-number churn, deactivation and recycling, with a transition period. Its 29 October 2026 Fifth Amendment is a draft for comment; proposed biometric checks must not be treated as final law.

These sources do not establish a universal bank alias deletion SLA, operator-specific recycling interval, or number-reassignment cooling-off period. These remain unknown unless supported by the exact provider/operator profile.

## Governing theorem

`phone number assignment != SIM control != bank account ownership != ShapID registration != payer consent != payment authority`

Reusing the same telephone number creates a new alias-binding generation. It must never inherit the prior holder's bank account, payment history, reputation, consent or pending payment request.

## Lifecycle and evidence

Model alias registration, provider identity/account binding, ownership evidence, resolver currentness, phone changes, deregistration, revocation, suspension, number release/reassignment, reactivation, cross-bank default transfer and bank-qualified alias creation as distinct append-only events.

A resolution receipt binds alias namespace, provider, alias generation, beneficiary commitment, directory epoch, freshness and expiry. A positive result is usable only when its generation and beneficiary match the active profile and no later revocation, reassignment, dispute or directory-epoch change invalidates it.

Do not guess a numeric cooling-off interval. Unknown remains unknown and high-risk reassignment is quarantined until the responsible provider confirms the current binding. A telecom number assignment does not prove that a bank account may be attached to the number.

## Provider-specific profiles

- Nedbank snapshot: primary number ShapID, bank-qualified aliases, default alias uniqueness, old mismatch deletion at end of business day per FAQ; no universal recycling cooldown stated.
- Standard Bank snapshot: one primary alias among bank-qualified ShapIDs; edit/delete operations documented; exact automatic invalidation/cooldown timing not qualified.
- ICASA final Fourth Amendment: churn/deactivation/recycling framework; exact operator implementation/effective date not inferred from the news post.
- ICASA Fifth Amendment: draft proposal only; not enforced as current legal requirement by this evaluator.
- Synthetic harness: explicit test fixture only, with no live participant or telecom claim.

## Benchmark

30 synthetic cases cover active binding, number change, revocation, recycling, SIM-swap signals, account closure, default alias collision, bank-qualified alias migration, stale directory cache, directory epoch advancement, new-generation rebind, conflicting positive generations, display mismatch, enumeration controls and historical-payment immutability.

Hard gates:
- 0 resolution against revoked, expired, conflicted, stale or reassignment-quarantined binding.
- 0 new number holders inherit a prior bank account or consent.
- 0 SIM-control evidence promoted to bank-account ownership.
- 0 bank alias registration inferred from telecom number assignment alone.
- 0 old cached positive results survive a later authenticated revocation or newer directory epoch.
- 0 unqualified default alias collisions silently resolved.
- 0 payer consent survives a material beneficiary change.
- 0 public alias enumeration beyond an explicitly permitted disclosure/rate-limit profile.
- 100% alias bindings carry generation, source, authority and currentness evidence.
- 100% old bindings remain immutable after reassignments.
- 100% unknown cooling-off and implementation dates remain explicit.

Measure alias-resolution latency, revocation propagation, stale-cache hit rate, old-owner misdirection prevention, false-positive quarantine, appeal reversal, cross-bank default migration completion, duplicate/replay suppression and enumeration leakage.

## Claim ceiling

A passing synthetic suite demonstrates only that the evaluator distinguishes telecom number assignment, bank alias registration, beneficiary binding, payer consent and payment effect. It proves no actual mobile subscriber identity, bank account ownership, directory freshness, real-time revocation or PayShap production integration.
