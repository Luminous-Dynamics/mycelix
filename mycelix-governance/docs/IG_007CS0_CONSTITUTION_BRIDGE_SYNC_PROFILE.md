# IG-007CS0 — Constitution-to-runtime governance-config sync profile

Issue: #1053

Parent evidence context: CP1 / draft #1008; source subject remains `fca2c107a1ea5108823ce617ba4111b6f7f77230`.

## Purpose

Freeze the cross-zome synchronization contract tracked by #944 before repair. This is distinct from ConstitutionParameter mutation authority (#1002) and governance-config mutation authority (#943).

## Frozen profile

```text
id        mycelix-constitution-bridge-sync-observed-fca2c107-v1
revision  1
authority ObservedSourceBound
SHA-256   60daae86044098561fa8e41bcdf6f695ab41234760be4b7e235d2780271b681e
```

## Bound source

Semantic production subject:

`fca2c107a1ea5108823ce617ba4111b6f7f77230`

Tree-equivalent evidence source:

`31ede2365b81365bb119cd9351b2739119974130`.

The profile binds the exact constitution coordinator blob and all eight bridge coordinator modules. The module census is part of the theorem because the key observation is an **absence** claim: `update_phi_config` is not present anywhere in that frozen coordinator surface.

## Observed caller

`sync_phi_parameter_to_bridge` maps eight Phi-related ConstitutionParameter names and makes a best-effort call to:

```text
governance_bridge::update_phi_config
```

The ConstitutionParameter write happens first. Failure/unavailability of the bridge call emits `PhiConfigSyncWarning`; it does not roll back the ConstitutionParameter write.

Therefore:

```text
ConstitutionParameter write success
!= runtime GovernanceConsciousnessConfig synchronization established
```

## Bridge surface

Across the exact eight-module coordinator census:

```text
update_phi_config occurrences = 0
visible updater              = update_consciousness_config
```

The visible updater is **not** treated as a safe drop-in replacement. Its separate source-bound authority gap is tracked by #943 / IG-007C0/C1.

## Reconciliation boundary

The frozen helper contains no observed content-bound:

- unsynchronized-state receipt;
- retry theorem;
- idempotent reconciliation contract;
- proof that ConstitutionParameter revision and runtime config revision agree.

This does not establish that operational reconciliation never happens elsewhere. It freezes only what this exact source contract proves.

## Qualification

The exact-head workflow:

1. verifies the PR subject and parent lineage;
2. binds the constitution coordinator and all eight bridge coordinator blobs;
3. independently checks the exact bridge module census;
4. proves `update_phi_config` is absent across that census;
5. proves `update_consciousness_config` is present in the frozen config module;
6. checks the constitution caller contains the mapped sync helper, best-effort call, target function and warning behavior;
7. runs the profile validator twice byte-identically;
8. asserts the exact content commitment and issue dependencies;
9. verifies checkout immutability.

Hosted PASS is not claimed until that exact subject executes.

## Successor rule

A repair must create successor evidence. Do not rewrite this profile to make the historical source look synchronized.

A correct successor must also respect #943: mechanically renaming `update_phi_config` to `update_consciousness_config` is insufficient unless the runtime updater's authorization theorem is corrected/qualified.

## Non-claims

No live constitution/runtime divergence, live config mutation, deployment exploit, deployment currentness, policy-correctness, or governance-safety claim is made.
