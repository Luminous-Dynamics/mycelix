# Authority Coordinator Deployment Composer v0.1 — Normative Invariants

Status: **pure target/release/observation composition theorem; live provenance and effect admission remain out of scope**

## 1. Three independent inputs remain distinct

Positive composition requires all three:

1. `TargetCellSelection` — the exact CellId selected for admission;
2. `QualifiedCurrentCoordinatorRelease` — the exact currently Active approved release; and
3. `ObservedCoordinatorDeployment` — the exact installed coordinator set observed for one CellId.

No input may silently choose or replace another.

## 2. Target selection is evidence/data, not authority

`TargetCellSelection` is intentionally deserializable. It records an exact DNA hash, agent public key and selection reference, but pure validation does not establish why that CellId was selected.

A live admission path must establish target-selection provenance/currentness independently.

## 3. Release authority does not choose the installation-specific agent

The authenticated/current release remains DNA/code scoped. The composer may specialize it to the independently supplied target agent only after requiring the target DNA to equal the release DNA.

## 4. #262 remains the exact code-set matcher

The composer must construct `RequiredCoordinatorDeployment` from the current release + target agent and call `match_required_coordinator_deployment` from the coordinator-deployment contract.

It must not reimplement subset/equality rules locally.

Missing, substituted, duplicate or unexpected coordinators therefore retain #262 fail-closed semantics.

## 5. Exact CellId equality is checked before positive composition

The specialized required deployment and the #262 matched deployment must both equal the exact target DNA hash + exact target agent public key.

A correct release for one DNA cannot be silently specialized or matched to another CellId.

## 6. Positive composition is non-deserializable

`QualifiedCoordinatorDeploymentComposition` derives `Serialize` but not `Deserialize`.

It is constructed only through the local exact join.

## 7. Composition identity commits every local theorem input

The composition digest commits:

- current-release qualification digest/profile;
- target-selection digest/profile;
- specialized required-deployment digest/profile;
- #262 exact-match digest/profile; and
- the final composed evidence window.

Changing any of those facts changes composition identity.

## 8. Lease composition is monotone

`verified_at = max(current release, conductor observation)`

`valid_until = min(current release, conductor observation)`

The target selector carries no lease in this pure v0.1 contract because its live provenance/currentness is intentionally delegated to the future native admission boundary. A live consumer must not infer target-selection freshness from this pure composition.

## 9. Pure equality is not provenance

A successful composition does not prove that:

- `TargetCellSelection` came from trusted host/application policy;
- `ObservedCoordinatorDeployment` came from the #264 native observer;
- release signature proof came from the designated signature verifier;
- registry-head/status receipts came from their designated verifiers.

Live provenance must be established separately before this composition can participate in effect admission.

## 10. No coordinator-update atomicity claim

A successful composition is a snapshot theorem. It does not prove coordinator code cannot change immediately afterward through `UpdateCoordinators`.

A before/after or equivalent native update-race fence remains mandatory.

## 11. No execution/effect authority

`QualifiedCoordinatorDeploymentComposition` is deployment evidence only. It grants no lifecycle, executor, effect-safety or external-effect permission.

## 12. Provisioning remains blocked

Before effect-capable provisioning, the stack still requires independently qualified live verifier provenance, target-selection provenance, real-conductor observation qualification, coordinator-update race fencing and final lifecycle/effect admission.
