# Response Effect Safety Invariants

This crate joins one non-deserializable `QualifiedResponseExecutionIntent` with one non-deserializable `QualifiedEffectSafetyPolicy`.

It is a composition theorem, not a new policy source and not an effect runtime.

## Exact pending-subject discharge

A safety qualification contributes only if its exact generic `EffectSafetyPolicy` subject is present in the response intent's pending authority set.

Similar policy IDs, equivalent-looking action labels, same adapter profile, or same institution are insufficient. Full `AuthoritySubjectRef` equality is required, including canonical policy identity digest/profile.

Only that exact subject is removed. Every other pending authority subject remains visible.

## Cross-domain equality

After exact subject matching, the composer independently requires:

```text
policy.action_class           == response intent action_class
policy.actions_digest         == exact response action digest
policy.actions_digest_profile == exact response action profile
policy.jurisdiction           == response intent jurisdiction
policy.adapter_profile        == response intent adapter profile
```

These checks are intentionally redundant with the policy identity. They prevent a future identity-profile regression from silently broadening response/effect compatibility.

## Time conservation

Composition occurs at an explicit positive `now_ms`.

The executor-authority lease embedded in the response intent and the effect-safety qualification must both still be live.

Final validity is:

```text
min(response executor-provider lease, effect-safety qualification lease)
```

No child layer may widen either horizon.

## Stable identity and evidence

Stable response-effect-safety qualification commits:

- immutable response proposal/digest/decision/option identity;
- exact execution action digest/profile;
- adapter profile;
- current executor-authority identity;
- exact effect-safety subject identity; and
- stable current effect-safety qualification identity.

Dynamic evidence additionally commits the safety evidence identity and the local composition verification window.

## Automatic effects remain explicit

The result exposes `automatic_effects_allowed()` from the exact qualified policy. A later automatic attempt-admission theorem must require it explicitly.

`all_declared_authority_subjects_satisfied()` means only that RESPONSE-AUTH's declared semantic subject set has no remaining unresolved members. It is deliberately **not** equivalent to execution permission.

## Still not execution permission

Even with an empty pending subject set, this layer does not prove:

- that authority/freshness/adoption/adapter receipts came from their designated live providers in this invocation;
- that the authority/currentness/coordinator verifier code is the approved live deployment;
- that the exact adapter state remains unchanged after its observation;
- that an attempt ID is unique;
- that an idempotency reservation or precondition fence is actually held;
- that a crash-safe claim exists; or
- that an external effect may occur.

The positive result is serializable for audit but not deserializable as a positive object, and `grants_execution_authority() == false` remains permanent here.
