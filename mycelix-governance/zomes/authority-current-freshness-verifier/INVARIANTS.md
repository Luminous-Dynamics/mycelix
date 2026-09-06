# Authority Current Freshness Verifier Runtime v0.1 — Normative Invariants

Status: **implemented composition candidate; deliberately unprovisioned in the binding governance DNA**

This runtime answers one question:

> Does one exact operational authority subject have a current, covered, generation-bound freshness state under the exact current constitution-rooted policy plane?

It does not own policy, constitution, state transitions, source truth, witness truth, probe entropy, lifecycle claims, or external effects.

## 1. No serialized positive-authority shortcut

Cross-zome providers may return evidence-shaped inputs only.

The coordinator MUST reconstruct positive authority locally through:

1. `qualify_bootstrap_root` (#111);
2. `qualify_control_plane_subject_freshness` (#115);
3. `qualify_operational_policy_context` (#116); and
4. `qualify_operational_subject_freshness` (#117).

No provider may return a deserialized `Qualified*` value and have it treated as authority by existence.

## 2. Discovery and verification are different roles

`authority_state_evidence_plan_provider` is discovery-only.

It may return exact probe action references, but:

**evidence-plan selection grants no authority**.

The plan provider MUST NOT supply verified source-head, witness/trust, transition, or probe-validity receipts.

## 3. Probe verification is independent

Every planned probe action MUST be sent to:

`authority_state_challenge::verify_issued_authority_state_probe`.

Only the returned `VerifiedCoverageChallenge` may enter #96/#115/#117.

A plan/source/witness/transition provider cannot manufacture private-entropy proof validity.

Probe authorship remains provenance only.

## 4. Source-head verification independently reconstructs probe validity

The source-head verifier receives the exact `probe_action: ActionHash`, not this coordinator's serialized positive `VerifiedCoverageChallenge`.

`authority_state_source_head_verifier::verify_source_head` must independently call #114 and reconstruct its own positive challenge receipt before authenticating a source response.

This coordinator also verifies the same probe action for its own #96/#115/#117 composition. The two receipts may have different dynamic verification timestamps, but their immutable `CoverageChallenge` identity must be the same because both derive from the same probe action.

A provider-supplied positive challenge receipt MUST NOT cross this source-head boundary as authority.

The later #96/#91 path independently cross-binds the source head to this coordinator's challenge, policy and transition endpoint.

## 5. Witness/trust verification is independent

The exact verified challenge + exact verified source head are sent to:

`authority_state_witness_verifier::verify_witness_evidence`.

Its output is bounded to at most 64 witnesses and 64 trust bindings.

Witness count alone is not independence. #96 remains responsible for exact trust-policy/verifier binding, trust-domain diversity and per-domain limits.

## 6. Transition verification is independent

The exact target subject + exact verified source head are sent to:

`authority_state_transition_verifier::verify_transition_lineage`.

Its output is bounded to 1–256 transitions.

The transition verifier does not establish source-head completeness by itself. #91 MUST still require one contiguous, fork-free, parent-digest-linked chain whose endpoint equals the independently verified covered head.

A valid transition prefix is not current authority.

## 7. Exact bootstrap root is reconstructed locally

The root provider returns:

- exact `AuthorityStateBootstrapRootManifest`;
- independently verified current constitution receipt; and
- independently verified exact root-adoption receipt.

The coordinator calls `qualify_bootstrap_root` itself.

Old constitution/root evidence, decode failure, provider outage, mismatched adoption, or expired root denies.

## 8. Qualification time follows evidence production

Every pure qualifier MUST receive a clock sample taken after the evidence-producing calls whose timestamps it judges.

Concretely:

- bootstrap-root provider call → root qualification time → `qualify_bootstrap_root`;
- control-plane probe/source/witness/transition verification → control-plane qualification time → `qualify_control_plane_subject_freshness`;
- policy provider + qualified control-plane set → context qualification time → `qualify_operational_policy_context`; and
- operational probe/source/witness/transition verification → current qualification time → `qualify_operational_subject_freshness`.

A single timestamp captured before asynchronous provider/verifier work MUST NOT be reused for later qualification. Doing so can make newly generated `verified_at_ms`, `responded_at_ms`, or observation timestamps appear to come from the future and spuriously fail closed.

This is causal correctness, not clock tolerance.

## 9. Operational policy semantics and policy currentness are separate

The policy provider returns semantic `VerifiedAuthorityCoveragePolicy` and `VerifiedCoverageTrustContextPolicy` receipts.

Currentness of those exact objects comes only from #115 proofs rooted in #111.

The plan provider may nominate at most three control-plane probes. Each is independently reverified through probe/source/witness/transition boundaries before `qualify_control_plane_subject_freshness` runs. #116 then reconstructs the exact two- or three-policy closed set.

Missing, extra, revoked, superseded, stale, wrong-root, or wrong-namespace policy evidence denies.

## 10. Covered namespace and registry namespace remain distinct

The runtime MUST NOT derive policy currentness from the operational policy's covered namespace.

That distinction belongs to #116:

- `coverage_policy.namespace` controls which target subjects may be covered;
- `root.control_plane_namespace` controls where policy objects' own authority state lives.

## 11. No latest-record heuristics

The runtime MUST NOT infer currentness from:

- highest observed generation;
- newest timestamp;
- DHT arrival order;
- absence of a later record;
- cached local state;
- first/last link ordering; or
- author reputation / Phi / model output.

There is no generic DHT current-record lookup in this coordinator.

## 12. Stable authority and evidence provenance remain separate

The runtime preserves #117:

- stable `authority_digest/profile`; and
- fresh `evidence_digest/profile`.

A new probe may refresh evidence without changing authority identity when the exact policy context and exact authority generation remain unchanged.

Any policy-currentness or authority-generation change changes the stable authority digest.

## 13. Fail closed on every provider boundary

Missing zome/function, non-OK call response, decode failure, malformed evidence, failed pure qualification, stale lease, mismatched subject, impossible evidence fan-in, or unavailable independent verifier MUST deny.

Provider availability is not inferred from code presence.

`current_freshness_runtime_status` performs no synthetic provider probe and reports `operational = false`.

## 14. Verifier receipts remain explicit trust boundaries

This tranche composes evidence; it does not implement every cryptographic verifier.

Current constitution, root adoption, semantic policy, source-head, witness/trust and transition-verification receipts remain explicit role-specific verifier outputs.

Cross-field pure qualification prevents substitution/rebinding, but does not replace cryptographic verification performed by those boundaries.

No one source/witness/transition aggregator is permitted to stand in for all three roles.

## 15. Historical/live separation

Only current projection paths may contribute to this runtime.

Historical/as-of authority evidence cannot become a live `VerifiedAuthorityFreshness` receipt.

Later revocation blocks new authority while preserving historical audit truth.

## 16. Deliberately unprovisioned

The coordinator remains in the Rust workspace only for native/Clippy/WASM qualification.

`mycelix-governance/dna/dna.yaml` MUST NOT contain `authority_current_freshness_verifier` in this tranche.

The root/policy/source/witness/transition providers are not yet production-qualified, so the binding governance DNA must not expose the runtime.

## 17. No external-effect authority

This coordinator cannot create grants, mutate authority transitions, sign governance actions, create lifecycle claims, execute actions, or authorize an external effect.

Phi, consciousness, reputation, stake, Guardian status and Symthaea output are not currentness authority.

## 18. Required qualification before provisioning

At minimum:

- rustfmt;
- native `cargo check` / warnings-denied Clippy;
- WASM build;
- source-head request ABI proof: probe action only, no serialized positive challenge;
- qualification-time ordering audit at root/control-plane/context/operational boundaries;
- hidden-later-revocation denial;
- truncated transition prefix denial;
- source-head mismatch denial;
- wrong/root-rotated policy denial;
- wrong probe/private-entropy proof denial;
- plan-provider substitution denial;
- source-verifier substitution denial;
- witness/trust-verifier substitution denial;
- transition-verifier substitution denial;
- direct-source and witness-quorum paths;
- provider outage/decode-failure denial;
- proof that no latest-DHT heuristic exists; and
- multi-agent Sweettests showing stale/partitioned evidence cannot produce live execution authority.

External effects remain disabled.
