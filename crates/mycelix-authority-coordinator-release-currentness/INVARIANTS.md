# Authority Coordinator Release Currentness v0.1 — Normative Invariants

Status: **pure currentness/withdrawal theorem; live registry-head/status verifier provenance not yet implemented; provisioning remains blocked**

## 1. Signed authenticity is not currentness

`QualifiedCoordinatorReleaseRequirement` from #269 proves one exact release was authenticated. It does not prove that release remains deployable.

This crate adds a separate currentness theorem.

## 2. Current registry head and release status are separate facts

Positive currentness requires both:

1. `VerifiedCurrentReleaseRegistryHeadProof`; and
2. `VerifiedCoordinatorReleaseStatusAtHeadProof`.

A status verifier does not choose the current registry head. A head verifier does not choose the status of an individual release.

## 3. v0.1 wire semantics are exact, not profile-selectable

One protocol version must mean one exact evidence representation.

`VerifiedCurrentReleaseRegistryHeadProof.registry_head_profile` MUST equal the fixed `REGISTRY_HEAD_PROFILE`.

`VerifiedCoordinatorReleaseStatusAtHeadProof` MUST use:

- the exact coordinator release `MANIFEST_PROFILE`;
- the exact fixed `REGISTRY_HEAD_PROFILE`; and
- the exact fixed `STATUS_RECORD_PROFILE`.

Nonempty alternate profile strings deny. A verifier cannot reinterpret the same v0.1 protocol ID as another registry-head/status-record format.

## 4. Status proof must bind the exact independently verified head

The status proof must exactly match the head proof on:

- release-policy digest/profile;
- registry generation;
- registry head digest; and
- registry head profile.

An `Active` proof for an older or different head denies.

## 5. Status proof binds the exact authenticated release

The status proof must name the exact authenticated release manifest digest and the exact release manifest profile.

A proof for another release cannot be reused.

## 6. Only Active qualifies

`Withdrawn` and `Superseded` are explicit fail-closed states.

There is no best-effort fallback from non-Active status.

## 7. Release-policy identity must agree end to end

The independently verified head must name the same release-policy digest/profile committed by the signed release manifest.

The status proof must name that same policy identity.

This proves policy identity equality only; it does not by itself prove the release policy is the globally/currently authorized policy lineage. That provenance remains a later verifier boundary.

## 8. Positive current release is non-deserializable

`QualifiedCurrentCoordinatorRelease` derives `Serialize` but not `Deserialize`.

It is constructed only by the local pure join over authenticated release + current-head proof + exact status-at-head proof.

## 9. Currentness lease is monotone

`verified_at = max(release, head, status)`

`valid_until = min(release, head, status)`

No component proof can be silently widened through currentness composition.

## 10. Head/status verifier provenance remains separate

Both proof inputs are intentionally deserializable evidence-shaped objects.

A live deployment composer must obtain them from independently qualified verifier roles and must never treat caller-supplied proof bytes as positive currentness merely because the pure join accepts their shape/equality.

## 11. Registry completeness is a verifier theorem

This crate does not discover a "latest" release record or infer currentness from local absence of later data.

The current-head verifier must independently prove that its head is current/complete under the chosen release-registry design. DHT/local-cache ordering heuristics are forbidden as substitutes.

## 12. Current release still is not installed deployment

A current authenticated release describes approved code. It does not prove the conductor is running it.

The native #264 observation and #262 exact whole-set matcher remain mandatory independent boundaries.

## 13. Current release still is not update atomicity

Even a current release exactly matching one conductor observation does not prove coordinator code cannot change before an external effect.

Update-race/atomicity admission remains separate.

## 14. No effect authority

Current release status does not establish current operational authority, lifecycle/executor authority, effect safety or external-effect permission.

## 15. Provisioning remains blocked

Before effect-capable provisioning, the stack still requires independently qualified:

- release-signature verifier provenance;
- current registry-head verifier provenance/completeness;
- exact status-at-head verifier provenance;
- target CellId selection;
- native conductor observation;
- exact release/deployment matching;
- coordinator update-race fencing; and
- later effect authorization.
