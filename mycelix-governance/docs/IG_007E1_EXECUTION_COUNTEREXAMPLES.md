# IG-007E1 — Execution authority counterexamples

Status: **MeasurementOnly / source-control-flow counterexamples**

Tracks: #909. Parent: #905 / draft #908.

## Bound profile

`mycelix-execution-observed-fca2c107-v1`

Profile SHA-256:

`c977bdcef9e5faac83351050999451432b618d5cc523bece804eba5dd1ae81f6`

## Corpus identity

Schema:

`mycelix-observed-execution-counterexamples-v1`

Corpus SHA-256:

`0c6669e44d6d18396ede43324f5cf3abbb25ddd3a2a9f59abb2c8a3699ba5fd4`

## CE-TL-01 — unbound construction predicates

A syntactically valid one-hour EmitEvent fixture is used only to show which predicates are absent from the observed creation contract:

```text
proposal lookup             NoneObserved
proposal status binding     NoneObserved
proposal actions binding    NoneObserved
policy duration binding     NoneObserved
```

The fixture is not executed.

## CE-TL-02 — creator-only Ready transition

For:

```text
source status = Pending
caller = TimelockCreator
```

observed local predicates permit transition toward `Ready` without a threshold-signature predicate inside `mark_timelock_ready`.

No live transition is performed by this research tranche.

## CE-TL-03 — signature control-flow differential

```text
Ready:
  threshold signature lookup = NoneInBranch
  assumption = ReadyImpliesPreviouslyVerified

Pending:
  threshold signature lookup = Attempted
  unavailable authority = WarnAndContinue
```

This is a control-flow comparison, not an end-to-end exploit claim.

## CE-TL-04 — executable action surface

The stored timelock payload is source-visible as dispatching to:

```text
TransferCredits -> governance_bridge::transfer_credits
UpdateParameter -> constitution::update_parameter
EmitEvent       -> emit_signal
```

Downstream authorization remains outside the evidence scope.

## CE-TL-05 — unavailable signing authority

For an expired Pending timelock with unavailable threshold-signing authority, the observed branch emits warning `threshold_signing_unavailable` and continues source control flow without established signature verification.

The counterexample does not execute an action.

## Successor use

The corrected implementation should create a new profile/corpus lineage where the old authority gaps no longer reproduce:

- construction derives the exact authorized proposal/actions/policy subject;
- Ready is backed by retained signature authorization evidence;
- unavailable signature authority fails closed;
- execution proves it is dispatching the exact authorized bytes.

## Non-claims

No live exploit, financial mutation, constitutional mutation, downstream authorization failure, deployment-currentness, or governance-safety claim is made.
