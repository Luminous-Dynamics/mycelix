# FORGE-009D — Protected Ref Transition Contract

This crate defines the provider-neutral protected-ref compare-and-swap boundary for Mycelix Forge.

It separates three facts that must not be collapsed:

```text
source-qualified merge request
!= permission to mutate a repository
!= proof an atomic mutation happened
```

## Pre-execution intent

`ProtectedRefTransitionIntentV1` is constructed only from:

```text
SourceQualifiedProtectedMergeRequestV1
+ exact ProtectedMergeRequestV1
```

Construction rechecks that the positive source qualification belongs to the exact protected merge request, project and proposal. The intent binds:

- project;
- proposal;
- exact protected merge request;
- exact FORGE-009B source-qualification evidence;
- target ref;
- expected old object;
- requested new object;
- protected-merge nonce.

The type intentionally has no `Deserialize` implementation. Arbitrary serialized digests cannot be promoted into a transition intent.

This is still not merge authorization.

## Raw provider observation

`ProtectedRefCasObservationV1` is a deserializable, non-authoritative observation carrying:

- provider identity;
- exact transition-intent digest;
- target ref;
- expected-old argument;
- provider-observed old value;
- requested-new argument;
- provider-observed new value;
- merge nonce;
- provider operation id;
- provider receipt commitment;
- applied / stale / rejected outcome.

The raw observation can never become positive simply because its fields look correct.

## Provider-verified post-execution receipt

`verify_protected_ref_transition_receipt_v1(...)` requires an independent `ProtectedRefCasReceiptVerifierV1` and fails closed unless all protocol cross-links match exactly:

```text
provider == verifier identity
observation.intent == exact transition intent
observation.ref == intent.ref
expected_before == intent.expected_base
observed_before == intent.expected_base
requested_after == intent.proposed_revision
observed_after == intent.proposed_revision
merge_nonce == intent.merge_nonce
outcome == Applied
provider-specific receipt verifier accepts evidence
```

Only then can Forge construct:

`ProviderVerifiedProtectedRefTransitionReceiptV1`.

## Critical non-claims

A positive receipt proves a post-execution CAS fact **relative to the supplied provider verifier**. It does not establish:

- that the provider identity is trusted by project policy;
- that M0 OfflineEvidence is qualified;
- that review/merge authority permitted the mutation;
- that the transition should have been executed;
- trusted wall-clock time;
- one-time merge-nonce consumption;
- permanent finality of the resulting ref;
- release authorization.

In particular:

```text
old -> new atomic CAS succeeds
then ref later returns to old
then replay might become mechanically possible again
```

Therefore replay protection / one-time request consumption remains a separate durable-state theorem.

## Execution API deliberately absent

This tranche does **not** expose `git update-ref` or another repository mutation API. The concrete executor should consume a later M0 + authority positive authorization type before it is allowed to mutate a protected ref.

That type gate is stronger than exposing a mutation method now and relying on callers to remember a policy comment.

## Tests

The focused tests prove:

- exact applied CAS evidence can become a provider-verified receipt;
- stale observed-old values fail before the provider verifier runs;
- merge-nonce substitution fails closed;
- stale/rejected outcomes never become positive receipts;
- provider identity substitution fails closed;
- raw provider observations round-trip as non-authoritative data;
- the public intent constructor requires `SourceQualifiedProtectedMergeRequestV1`.

## Next boundary

After the independent M0 rejoin exists, construct the positive merge-execution authorization type that deliberately composes:

```text
MergeProtected authority
+ review-policy-satisfied protected basis
+ source-qualified protected merge request
+ exact qualified M0 OfflineEvidence
+ durable one-time request consumption policy
```

Only that positive type should unlock a concrete Git protected-ref mutation adapter.
