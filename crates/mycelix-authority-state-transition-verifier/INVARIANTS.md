# Authority-State Transition Verification v0.1 — Normative Invariants

Status: **pure qualification kernel; no runtime provisioning and no external effects**

This layer closes one ambiguity in the current freshness runtime:

> an immutable transition record being authentic is not the same fact as the institution having authorized that transition.

The legacy `VerifiedAuthorityStateTransition` ABI represents both facts in one evidence-shaped object. This kernel makes the two proof domains explicit before that compatibility ABI may be projected.

## 1. Transition bytes are candidate semantics

`AuthorityStateTransition` remains immutable candidate data.

Record existence, DHT visibility, source publication, author identity, timestamp ordering, reputation, Phi or model output do not make the transition institutionally valid.

The pure kernel recomputes the exact transition identity digest under `TRANSITION_IDENTITY_PROFILE` before either verifier receipt is accepted.

## 2. Record proof and institutional authority proof are different facts

A positive transition requires two independent evidence-shaped receipts:

- `VerifiedTransitionRecordProof` — proves the exact immutable transition record/content proof; and
- `VerifiedTransitionAuthorityProof` — proves the exact institutional/rulebook authorization referenced by that transition.

Neither receipt may stand in for the other.

A future runtime MUST obtain them from different verifier roles. One omnibus transition verifier must not construct both receipts.

## 3. Both proof domains bind the exact transition digest

Each receipt commits the exact transition digest/profile.

The record verifier additionally binds the exact:

- `status_record_ref`; and
- `record_proof_ref`.

The institutional authority verifier additionally binds the exact:

- `authority_ref`; and
- `authority_proof_ref`.

Any mismatch denies.

## 4. Authoritative source identity is independent

The proof verifiers do not choose `authoritative_source_ref`.

That value is a separate input to `qualify_authority_state_transition` and must come from the independently authenticated source-head/coverage path in the runtime.

The stable transition-verification identity commits the exact authoritative source together with the exact transition identity.

Thus:

`record proof validity ≠ institutional authorization validity ≠ authoritative source identity`.

## 5. Positive construction only through the pure kernel

The positive path is:

`AuthorityStateTransition`
→ recomputed transition digest
→ exact record-proof receipt
+ exact institutional-authority receipt
+ independent authoritative source
→ `qualify_authority_state_transition`
→ non-deserializable `QualifiedAuthorityStateTransitionVerification`
→ compatibility projection to `VerifiedAuthorityStateTransition`.

`QualifiedAuthorityStateTransitionVerification` derives `Serialize` but not `Deserialize`.

## 6. Stable verification identity and dynamic evidence are separate

`qualification_digest/profile` commits only:

- exact transition identity digest/profile; and
- exact authoritative source ref.

`evidence_digest/profile` additionally commits both verifier identities, verification references, verification timestamps and validity horizons.

Re-verifying the same exact transition under the same source may refresh evidence without inventing a new semantic transition-verification identity.

## 7. Causal verification ordering

Neither verifier may claim verification before the transition becomes effective.

For each proof:

`transition effective_at <= proof verified_at <= qualification now < proof valid_until`.

The projected verification time is the maximum of both verifier times.

## 8. Leases only narrow

Projected `lease_until_ms` is the minimum of record-proof and institutional-authority-proof validity.

No qualifier or projection may lengthen either verifier's evidence horizon.

The projected legacy receipt is revalidated before return.

## 9. Complete lineage/currentness remains elsewhere

This kernel proves one transition only.

It does not choose the current generation, prove the transition set is complete, choose by timestamp, or establish that no later revocation exists.

#91 must still project one contiguous parent-digest-linked lineage whose endpoint equals the independently covered source head.

A valid transition prefix is not current authority.

## 10. Pure separation

This crate contains no HDK/Holochain calls, DHT lookup, persistence, source-head selection, witness classification, lifecycle mutation, external effect, Phi, reputation, stake, Guardian override or model-score authority.

## 11. Runtime acceptance target

The current-freshness runtime follow-on must replace the omnibus `authority_state_transition_verifier` with:

1. candidate transition discovery only;
2. exact transition record-proof verification;
3. exact institutional transition-authority verification;
4. authoritative source identity taken from the independently authenticated source head;
5. local `qualify_authority_state_transition` for every candidate; and
6. the existing #91 complete-lineage/head endpoint check.

Provider or verifier outage must deny rather than fall back.
