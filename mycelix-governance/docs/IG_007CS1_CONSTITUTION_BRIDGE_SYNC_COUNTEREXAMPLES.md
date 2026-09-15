# IG-007CS1 — Constitution-to-runtime-config synchronization counterexamples

Issue: #1061

Parent: IG-007CS0 / #1053 / draft #1059.

## Purpose

Turn the frozen #944 synchronization observations into deterministic source-contract counterexamples before any production repair.

No live ConstitutionParameter or GovernanceConsciousnessConfig mutation is performed.

## Bound profile

```text
id        mycelix-constitution-bridge-sync-observed-fca2c107-v1
authority ObservedSourceBound
SHA-256   60daae86044098561fa8e41bcdf6f695ab41234760be4b7e235d2780271b681e
```

## Corpus

```text
schema    mycelix-constitution-bridge-sync-counterexamples-v1
authority MeasurementOnly
SHA-256   2ca6d79212cfd0acae1e974c8390d564bab06821050d8eacbc32f437630ef60b
issues    #943, #944
```

## CE-CS-01 — missing target in the exact coordinator census

The frozen constitution helper calls:

```text
governance_bridge::update_phi_config
```

The exact eight-module bridge coordinator census contains zero occurrences of `update_phi_config`; the visible runtime updater is `update_consciousness_config`.

Frozen result:

`TargetEntrypointAbsentFromObservedBridgeCoordinatorCensus`.

This is a source-census theorem, not a universal statement about every deployment/build.

## CE-CS-02 — successful parameter write does not establish runtime sync

The modeled ordering is exactly:

```text
ConstitutionParameter write = SucceededBeforeSyncAttempt
bridge sync                = UnavailableOrFailed
call semantics             = BestEffort
failure effect             = warning + retained parameter write
```

Frozen result:

`ConstitutionParameterSuccessDoesNotEstablishRuntimeConfigSynchronization`.

No live divergence is asserted.

## CE-CS-03 — rename alone crosses the #943 boundary

The absent target could superficially be replaced by the visible `update_consciousness_config` entrypoint, but that updater has its own independent source-bound authorization gap in #943 / C0/C1.

Frozen result:

`EntrypointRenameAloneWouldBypassSeparateAuthorizationTheorem`.

The fixture deliberately does **not** claim the visible updater cannot be used after its own authorization theorem is corrected and qualified.

## CE-CS-04 — no content-bound reconciliation evidence

The frozen helper exposes neither an explicit unsynchronized-state receipt nor an explicit retry/reconciliation contract.

Frozen result:

`NoObservedContentBoundReconciliationEvidence`.

This does not prove that operators or another subsystem never reconcile state; it describes the bound helper contract only.

## Exact-head qualification

The workflow:

1. verifies the exact child subject and three-file delta over CS0;
2. binds the same constitution + eight-module bridge source census as CS0;
3. re-runs CS0 profile validation twice byte-identically;
4. runs CS1 self-test twice byte-identically;
5. emits the CS1 corpus twice byte-identically;
6. asserts exact profile/corpus identities and all four fixture results;
7. verifies checkout immutability.

Hosted PASS remains unclaimed until Actions executes this exact subject.

## Successor boundary

A correct sync successor should stop reproducing CE-CS-01, CE-CS-02 and CE-CS-04. CE-CS-03 remains a dependency constraint: synchronization must consume a corrected/qualified governance-config mutation authority rather than bypass #943.

## Non-claims

No live runtime divergence, live mutation, deployment exploit, deployment currentness, governance safety, fairness, or constitutional legitimacy is established.
