# Authority Current Freshness Verifier Runtime v0.1 — Normative Invariants

Status: **implemented composition candidate; deliberately unprovisioned in the binding governance DNA**

## 1. Binding constitutional authority is reused

Current constitutional truth comes only from `constitution_transition::get_verified_current_constitution(())`.

The current-freshness coordinator does not introduce a parallel constitutional authority plane. It locally rechecks statement digest consistency and requires `legacy_constitution_authoritative = false`.

## 2. Root manifest discovery is not root authority

`authority_state_bootstrap_root_manifest_provider` returns only one semantic `AuthorityStateBootstrapRootManifest` candidate.

It cannot supply constitutional currentness, proof verification, #111 adoption, or a qualified bootstrap root.

## 3. Root-adoption proof verification is not root-adoption authority

The runtime constructs `BootstrapRootAdoptionClaim` locally through #148 from the exact binding current `ConstitutionStatement` and exact candidate root manifest.

Only that claim is sent to `authority_bootstrap_root_adoption_proof_verifier::verify_bootstrap_root_adoption_proof`.

The proof verifier returns only `VerifiedBootstrapRootAdoptionProof`.

It MUST NOT return #111's `VerifiedBootstrapRootAdoption`, a `QualifiedBootstrapRootAdoption`, or a qualified bootstrap root.

## 4. #111 adoption is constructed locally

After proof verification and the post-adoption constitutional fence, the coordinator calls #148 `qualify_bootstrap_root_adoption` itself.

That pure kernel recomputes the current statement/rulebook/root adoption claim and exact-binds the verifier proof before projecting the #111 adoption ABI.

Only that locally projected adoption receipt may enter `qualify_bootstrap_root`.

Runtime status freezes:

- `root_adoption_proof_verifier_separate = true`; and
- `root_adoption_constructed_locally = true`.

## 5. Constitutional TOCTOU is fail-closed at root and return

The root path is:

`binding constitution BEFORE → manifest → local adoption claim → proof verifier → binding constitution AFTER → exact equality → host time → #148 adoption qualification → #111 root qualification`.

A constitutional change during adoption proof verification denies.

After #115/#116/#117 and all operational evidence work, the binding constitution is projected again. A constitutional change anywhere during the complete composition also denies.

## 6. Leases only narrow

The locally constructed current-constitution receipt is composition-bounded. The final runtime freshness is capped to the short post-constitutional-fence reuse window.

#148 additionally caps adoption validity to the proof-verifier/root-manifest intersection, and #111 narrows again against constitution/root-policy horizons.

No composer may lengthen any evidence or authority lease.

## 7. Positive authority is reconstructed locally

The positive chain is:

`binding constitution → #148 adoption → #111 root → #115 policy currentness → #116 operational policy context → #117 operational currentness`.

Provider deserialization alone never creates a `Qualified*` object.

## 8. Probe/source/witness/transition roles remain independent

Evidence planning is discovery only. #114 owns probe provenance. #131 independently re-verifies probe provenance before source authentication. Witness/trust and transition verification remain separate roles. #91 still owns exact covered endpoint projection.

A valid transition prefix is not current authority.

## 9. No latest-record heuristic

Highest generation, newest timestamp, arrival order, absence of a later DHT record, author identity, reputation, Phi or model output cannot establish current authority.

## 10. Fail closed

Missing constitution/adoption-proof/probe/source/witness/transition verifier, decode failure, stale proof, changed constitutional head, wrong adoption claim, failed pure qualification or expired lease denies.

## 11. Deliberately unprovisioned

The current-freshness verifier remains absent from `dna.yaml`. The root-adoption proof verifier is also not provisioned by this tranche.

No external effect is enabled and `operational = false` remains explicit.

## 12. Required qualification before provisioning

At minimum:

- native/Clippy/WASM qualification;
- direct binding-constitution calls;
- local #148 `build_adoption_claim` and `qualify_bootstrap_root_adoption` on the positive path;
- proof verifier output restricted to `VerifiedBootstrapRootAdoptionProof`;
- no direct provider-supplied #111 adoption receipt;
- constitutional before/after/final fences;
- conservative lease narrowing;
- proof-verifier outage/stale/wrong-claim denial;
- old constitution/wrong root/wrong authority/proof-ref denial;
- source/witness/transition substitution denial; and
- adversarial multi-agent constitutional-race and adoption-proof replay tests.
