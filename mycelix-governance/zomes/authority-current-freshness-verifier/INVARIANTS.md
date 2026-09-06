# Authority Current Freshness Verifier Runtime v0.1 — Normative Invariants

Status: **implemented composition candidate; deliberately unprovisioned in the binding governance DNA**

## 1. Binding constitutional authority is reused, not reinvented

Current constitutional truth comes only from the existing local:

`constitution_transition::get_verified_current_constitution(())`.

That runtime reconstructs DNA genesis, re-verifies reachable constitutional transition candidates against binding-tally and threshold-verifier boundaries, enforces nonce/fork rules and runs the verified constitutional lineage projector.

This coordinator MUST NOT introduce or trust a parallel `authority_current_constitution_verifier` service.

The returned wire object is locally rechecked for exact statement digest consistency and `legacy_constitution_authoritative = false` before it can become #111 evidence.

## 2. Bootstrap manifest discovery is not bootstrap authority

`authority_state_bootstrap_root_manifest_provider` returns one `AuthorityStateBootstrapRootManifest` candidate only.

It cannot supply current constitutional truth, root-adoption verification, or a qualified bootstrap root.

`root_manifest_provider_grants_authority = false`.

## 3. Root adoption remains separate

`authority_bootstrap_root_adoption_verifier` receives the exact candidate manifest plus the exact current statement digest/profile returned by the binding constitutional plane.

It does not receive `VerifiedCurrentConstitutionReceipt` as positive authority.

#111 locally recomputes the exact statement, rulebook, manifest and adoption bindings.

## 4. Constitutional TOCTOU is fail-closed at the root

Root qualification requires:

`binding constitution BEFORE → manifest → adoption verifier → binding constitution AFTER`.

The BEFORE and AFTER projections must have identical:

- DNA hash;
- exact `ConstitutionStatement`;
- statement digest; and
- verified transition count.

If constitutional authority changes while root adoption is being checked, root qualification denies.

Only after the second projection is stable does the coordinator sample qualification time and construct a short-lived `VerifiedCurrentConstitutionReceipt` for local #111 qualification.

## 5. The constitutional fence spans the complete authority composition

After #115/#116/#117 and all source/witness/transition evidence have been qualified, the coordinator calls `constitution_transition::get_verified_current_constitution(())` a third time immediately before returning positive freshness.

That final projection must equal the root's constitutional projection. Any constitutional advancement during policy or operational evidence collection denies.

The exported `VerifiedAuthorityFreshness` is then narrowed to at most `CURRENT_CONSTITUTION_RETURN_LEASE_MS` after this final fence and its dynamic verification reference commits the exact constitutional statement digest.

Thus a long root/provider lease cannot silently widen constitutional-currentness reuse.

## 6. Constitutional verification leases are bounded

The locally projected #111 current-constitution receipt is bounded to `CURRENT_CONSTITUTION_COMPOSITION_LEASE_MS`; it is not an indefinite assertion of currentness.

The final runtime output is narrower still: at most `CURRENT_CONSTITUTION_RETURN_LEASE_MS` after the final constitutional projection.

A composer may shorten a lease but never lengthen one.

## 7. Positive authority is reconstructed locally

The coordinator reconstructs positive authority through #111 → #115 → #116 → #117.

No provider may serialize a `Qualified*` object and have it trusted by existence.

## 8. Probe/source/witness/transition roles remain independent

The evidence-plan provider chooses probe references only. #114 verifies probe provenance. #131 independently reconstructs probe verification for source authentication. Witness/trust and transition verification remain separate zome boundaries.

A valid transition prefix is not current authority; #91 still requires exact endpoint equality with the independently covered head.

## 9. Qualification time follows evidence production

Each pure qualifier receives a host-clock sample after the evidence-producing calls whose timestamps it judges.

The root clock is sampled only after the post-adoption constitutional projection. Final output time is sampled only after the final constitutional projection.

## 10. No currentness-by-observation shortcut

Highest observed generation, newest timestamp, DHT arrival order, absence of a later record, cached local state, author identity, reputation, Phi or model output cannot establish current authority.

The constitutional plane may inspect its append-only candidates, but authoritative constitutional truth comes from its verified lineage projector—not a latest-record heuristic.

## 11. Fail closed on every role boundary

Missing constitution/probe/source/witness/transition/adoption verifier, non-OK call, decode failure, malformed receipt, stale lease, changed constitutional head, ambiguous lineage, or failed pure qualification denies.

## 12. Historical/live separation

Historical constitutional or authority-state evidence cannot be converted into a live freshness receipt.

## 13. Deliberately unprovisioned

`authority_current_freshness_verifier` remains absent from `mycelix-governance/dna/dna.yaml`.

No external effect is enabled and `operational = false` remains explicit.

## 14. Required qualification before provisioning

At minimum:

- rustfmt, native check, warnings-denied Clippy and WASM build;
- exact dependency on `mycelix-governance-constitution`;
- exact call to `constitution_transition::get_verified_current_constitution(())`;
- no abstract current-constitution verifier service;
- recomputed constitutional statement digest validation;
- legacy constitutional authority rejection;
- root before/after constitutional fence ordering;
- final whole-composition constitutional fence ordering;
- final freshness lease narrowing;
- old-constitution replay and wrong-adoption denial;
- verifier outage and constitutional-race denial;
- source/policy/transition substitution denial;
- direct-source and witness-quorum paths; and
- multi-agent Sweettests for concurrent constitutional advancement during adoption and operational evidence collection.

No external effects are enabled.
