# IG-007CP1 — Constitution parameter authority counterexamples

Issue: #1006

Parent: IG-007CP0 / draft #1005

Security finding: #1002

## Purpose

Freeze five deterministic source-contract counterexamples against the exact CP0 profile before any production repair.

The corpus is measurement-only. No live parameter mutation is performed.

## Bound profile

```text
mycelix-constitution-parameter-observed-fca2c107-v1
770552d12489df1d2cdf8b0af676b01ea9a3da21940f70ed8a71910deaa35009
ObservedSourceBound
```

## Corpus identity

```text
schema    mycelix-constitution-parameter-counterexamples-v1
authority MeasurementOnly
SHA-256   b37be9d2e3fd0cbec3696a19327c26dc4ba7062ec089ad92ad99bc28a11fea8e
```

## CE-CP-01 — execution existing-parameter mismatch

The observed execution action dispatches only `parameter + value`, so Constitution receives no proposal ID from that path.

For an already-existing parameter the observed `set_parameter` gate rejects `proposal_id=None`.

Frozen result:

`ObservedExecutionDispatchCannotPassExistingParameterPresenceGate`.

This is explicitly **positive containment evidence**. It is not a claim that the downstream parameter boundary is generally safe.

## CE-CP-02 — new parameter without proposal linkage

For a previously absent parameter with nonempty name and valid JSON value, the observed coordinator gate does not reject `proposal_id=None`.

Frozen result:

`NewParameterCreationAllowedWithoutProposalLinkage`.

This is a pure contract fixture, not a live mutation.

## CE-CP-03 — proposal-ID presence without authority reconstruction

For an existing parameter, model a caller-supplied `Some("proposal:fixture")` while proposal existence/status/type, exact parameter/value authorization, and caller authority remain unobserved.

Frozen result:

`ProposalIdPresencePassesObservedCoordinatorGateWithoutAuthorityReconstruction`.

The fixture proposal is not asserted to exist.

## CE-CP-04 — integrity shape validity is not mutation authority

Model a nonempty parameter name and JSON-valid value with arbitrary/absent governance metadata.

The observed integrity theorem checks shape but does not bind action author or verify `changed_by_proposal` authority.

Frozen result:

`IntegrityShapeValidityDoesNotEstablishParameterMutationAuthority`.

## CE-CP-05 — timestamp projection is not fork authority

Model two same-name parameter publications with different link timestamps.

The read projection chooses maximum timestamp while no explicit authoritative competing-publication rule is observed.

Frozen result:

`TimestampSelectedProjectionNotAuthoritativeForkResolution`.

## What the corpus establishes

Together the fixtures separate five distinct propositions:

```text
execution payload completeness
!= existing-parameter gate
!= new-parameter authorization
!= proposal authority verification
!= integrity authority
!= authoritative currentness
```

This prevents any one positive or negative result from being stretched into an end-to-end governance claim.

## Successor criterion

A corrected implementation should create successor evidence in which the relevant old fixtures stop reproducing because:

- exact mutation authority is content-bound;
- bootstrap is explicit and bounded;
- proposal/action/currentness evidence is qualified;
- caller/executor authority is explicit;
- parameter projection handles competing publications fail-closed.

Historical CP0/CP1 remains immutable evidence of the old production subject.

## Non-claims

No live unauthorized parameter mutation, deployment exploit, legal/constitutional invalidity, authoritative currentness, or governance-safety verdict.
