# Authority Current Freshness Verifier Runtime v0.1 — Normative Invariants

Status: **implemented composition candidate; deliberately unprovisioned in the binding governance DNA**

This runtime answers one question:

> Does one exact operational authority subject have a current, covered, generation-bound freshness state under the exact current constitution-rooted policy plane?

It does not own policy, constitution, state transitions, source truth, witness truth, probe entropy, lifecycle claims, or external effects.

## 1. No serialized positive-authority shortcut

Cross-zome providers may return only evidence-shaped inputs.

The coordinator MUST reconstruct positive authority locally through the pure kernels:

1. `qualify_bootstrap_root` (#111);
2. `qualify_control_plane_subject_freshness` (#115);
3. `qualify_operational_policy_context` (#116); and
4. `qualify_operational_subject_freshness` (#117).

A provider MUST NOT be able to return a deserialized `Qualified*` value and have it treated as authority by existence.

## 2. Probe verification is a separate boundary

Generic evidence bundles contain an exact `probe_action: ActionHash`, not a `VerifiedCoverageChallenge`.

The coordinator MUST call:

`authority_state_challenge::verify_issued_authority_state_probe`

and use the resulting `VerifiedCoverageChallenge`.

This preserves #114's private-entropy provenance boundary. A generic source/evidence provider cannot fabricate probe validity merely by serializing a receipt.

Probe authorship remains provenance only and grants no institutional authority.

## 3. Exact bootstrap root is reconstructed locally

The root provider returns:

- exact `AuthorityStateBootstrapRootManifest`;
- independently verified current constitution receipt; and
- independently verified exact root-adoption receipt.

The coordinator MUST call `qualify_bootstrap_root` itself.

Old constitution/root evidence, decode failure, provider outage, mismatched adoption, or expired root denies.

## 4. Operational policy semantics and policy currentness are separate

The operational policy provider returns semantic `VerifiedAuthorityCoveragePolicy` and `VerifiedCoverageTrustContextPolicy` receipts.

Currentness of those exact policy objects comes only from #115 control-plane freshness proofs rooted in #111.

The coordinator obtains at most three control-plane evidence bundles, re-verifies each probe, and calls `qualify_control_plane_subject_freshness` for each. #116 then reconstructs the exact two- or three-policy closed set and denies missing, extra, revoked, superseded, stale, wrong-root, or wrong-namespace policy evidence.

## 5. Covered namespace and registry namespace remain distinct

This runtime MUST NOT derive policy currentness from the operational policy's covered namespace.

That semantic distinction belongs to #116:

- `coverage_policy.namespace` controls which target subjects may be covered;
- `root.control_plane_namespace` controls where the policy objects' own authority state lives.

The runtime delegates that exact check to `qualify_operational_policy_context` rather than duplicating or weakening it.

## 6. Current operational subject requires fresh dynamic evidence

A current policy context is not enough to prove one operational subject current.

The operational evidence provider supplies:

- exact probe action;
- independently verified source-head receipt;
- independently verified witness receipts when required;
- independently verified trust bindings when required; and
- complete independently verified transition receipts.

The coordinator re-verifies the probe and then calls #117, which requires #96 context-bound coverage plus #91 complete current projection.

A valid transition prefix is not current authority.

## 7. No latest-record heuristics

The runtime MUST NOT infer currentness from:

- highest observed generation;
- newest timestamp;
- DHT arrival order;
- absence of a later record;
- cached local state;
- first/last link ordering; or
- author reputation / Phi / model output.

There is no generic DHT lookup in this coordinator.

## 8. Evidence fan-in is bounded

v0.1 accepts at most:

- 3 control-plane evidence bundles;
- 64 witness receipts per bundle; and
- 256 transition receipts per bundle.

Operational transition evidence must be non-empty.

These bounds are denial-of-service controls, not authority semantics.

## 9. Stable authority and evidence provenance remain separate

The runtime returns both #117:

- stable `authority_digest/profile`; and
- fresh `evidence_digest/profile`.

A new probe may refresh evidence without changing the authority domain when the exact policy context and exact authority generation remain unchanged.

Any policy-currentness or authority-generation change changes the stable authority digest.

## 10. Fail closed on every provider boundary

Missing zome/function, non-OK call response, decode failure, malformed evidence, failed pure qualification, stale lease, or mismatched subject MUST deny.

Provider availability is not inferred from code presence.

`current_freshness_runtime_status` performs no synthetic provider probe and reports `operational = false` in this tranche.

## 11. Verifier receipts remain trust boundaries

This tranche composes evidence; it does not implement every cryptographic verifier.

`VerifiedCurrentConstitutionReceipt`, `VerifiedBootstrapRootAdoption`, policy receipts, source-head receipts, witness/trust receipts, and transition receipts remain explicit independent verifier-boundary outputs. Their future providers must be separately qualified and fail closed.

Cross-field pure qualification prevents substitution/rebinding, but does not replace cryptographic verification performed by those boundaries.

## 12. Historical/live separation

Only current projection paths may contribute to this runtime.

Historical/as-of authority evidence cannot become a live `VerifiedAuthorityFreshness` receipt.

Later revocation blocks new authority while preserving historical audit truth.

## 13. Deliberately unprovisioned

The coordinator is added to `mycelix-governance/Cargo.toml` only for native/Clippy/WASM qualification.

`mycelix-governance/dna/dna.yaml` MUST NOT contain `authority_current_freshness_verifier` in this tranche.

The required root/policy/source/witness/transition provider set is not yet production-qualified, so the binding governance DNA must not expose this runtime.

## 14. No external-effect authority

This coordinator cannot:

- create grants;
- mutate authority-state transitions;
- sign governance actions;
- create lifecycle claims;
- execute actions; or
- authorize an external effect.

Phi, consciousness, reputation, stake, Guardian status and Symthaea output are not currentness authority.

## 15. Required qualification before provisioning

At minimum:

- rustfmt;
- native `cargo check` / warnings-denied Clippy;
- WASM build;
- hidden-later-revocation denial;
- truncated transition prefix denial;
- source-head mismatch denial;
- wrong/root-rotated policy denial;
- wrong probe/private-entropy proof denial;
- direct-source and witness-quorum paths;
- revoked/superseded target denial at closed-set consumers;
- provider outage/decode-failure denial;
- proof that no latest-DHT heuristic exists; and
- multi-agent Sweettests showing a stale partition cannot produce live execution authority.

External effects remain disabled.
