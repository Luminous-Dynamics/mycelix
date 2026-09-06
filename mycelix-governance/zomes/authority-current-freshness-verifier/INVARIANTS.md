# Authority Current Freshness Verifier Runtime v0.4 — Normative Invariants

Status: **implemented lease-complete composition candidate; deliberately unprovisioned in the binding governance DNA**

## 1. Binding constitutional authority is reused

Current constitutional truth comes only from `constitution_transition::get_verified_current_constitution(())`. The coordinator locally rechecks the statement digest and rejects legacy constitutional authority.

The constitutional head is fenced before root adoption, after root adoption, and immediately before positive deployment qualification.

## 2. Root discovery and root authority remain separate

The root-manifest provider returns candidate semantics only. The runtime builds #148's adoption claim locally, accepts only `VerifiedBootstrapRootAdoptionProof` from the independent proof verifier, rechecks constitutional currentness, runs #148 locally and only then runs #111.

The resulting root's own `verified_at_ms` / `valid_until_ms` is an explicit member of the end-to-end evidence lease. Root/adoption/constitutional evidence therefore cannot disappear merely because later semantic qualifiers also consume the root.

## 3. Dynamic evidence lease is not authority identity

`EvidenceLease` is a transport/composition constraint only. It does not decide subject identity, institutional trust, policy currentness, quorum, transition completeness, execution authority or external-effect permission.

The global lease obeys:

`verified_at = max(all dependency verification times)`

`valid_until = min(all dependency validity horizons)`

No intersection, projection or deployment step may widen that interval.

## 4. Leased source-head evidence is mandatory

The current-freshness coordinator calls only:

`authority_state_source_head_verifier::verify_source_head_leased`.

It does not call the legacy unleased source-head endpoint.

The source-head envelope carries #130's exact challenge/attestation/cryptographic-verifier horizon. The coordinator independently re-verifies the probe and requires the leased source head to bind the exact same challenge digest and subject.

Source authentication still does not establish institutional source trust or completeness.

## 5. Leased witness evidence is mandatory

The coordinator calls only:

`authority_state_witness_verifier::verify_witness_evidence_leased`.

It does not call the legacy unleased witness endpoint.

The witness envelope carries the minimum of challenge/source/context semantics plus every observation-authentication and trust-classification verifier horizon. That lease remains separate from quorum authority; #94/#96 still decide count, domain diversity and concentration.

The source-head cryptographic horizon remains a separate lease and is intersected with the witness lease locally.

## 6. Transition discovery is not transition truth

`authority_state_transition_candidate_provider` may return candidate `AuthorityStateTransition` bytes only.

It cannot return `VerifiedAuthorityStateTransition`, record-proof validity, institutional-authorization validity or a qualified transition.

A malicious or incomplete candidate set must fail later #159/#91 qualification rather than becoming current by observation.

## 7. Transition proof domains remain separate and their leases are explicit

For every candidate transition, the coordinator computes the exact transition identity itself and calls separate immutable-record and institutional-authority proof verifiers.

After #159 qualifies the exact transition, the coordinator constructs an `EvidenceLease` from #159's `verified_at_ms` and `valid_until_ms` and intersects every transition lease in the candidate lineage.

The authoritative source consumed by #159 comes only from the independently authenticated source head.

Thus:

`transition location ≠ record authenticity ≠ institutional authorization ≠ authoritative source identity ≠ proof lifetime`.

## 8. Complete lineage remains #91's authority boundary

Per-transition verification does not make a candidate set complete/current.

#91 still requires one contiguous generation chain, exact parent-transition digest linkage and endpoint equality with independently authenticated/covered source head.

Provider order, highest generation, newest timestamp, DHT arrival order and absence of a later locally observed record have no authority meaning.

## 9. Every authority-evidence bundle returns one lease

For each probe, `resolve_evidence` intersects:

- leased source-head authentication;
- leased witness/trust evidence; and
- every #159 transition-proof lease.

The returned `ResolvedAuthorityEvidence` cannot exist without a live intersection.

The control-plane path additionally intersects each probe evidence lease with the resulting #115 control-plane semantic freshness lease, then intersects all required control-plane probes.

## 10. Operational policy proof leases remain separate from generation currentness

The coordinator calls only:

`authority_operational_policy_leased_provider::resolve_operational_policy_candidates_leased`.

That envelope carries immutable-record and institutional-adoption proof horizons for the exact coverage/context policies.

The lease does not make those policies current. #115 still proves generation-bound `Active` state, and #116 still joins that currentness to the exact verified semantic identities.

## 11. Global evidence lease is closed before #117

Before operational #117 qualification, the coordinator has already intersected:

- bootstrap-root evidence lifetime;
- operational policy record/adoption lease;
- complete control-plane evidence/semantic lease; and
- operational source/witness/transition evidence lease.

After #117 succeeds, its own semantic freshness lease is also intersected.

Therefore no adapter verifier horizon may disappear merely because a legacy-compatible receipt was passed through #94/#96/#115/#116/#117.

## 12. Host DNA and final constitution are independent deployment evidence

Only after #117 succeeds does the coordinator derive local DNA through `dna_info()?.hash.to_string()`, project the binding constitution again, and require exact equality with the root constitutional epoch.

The constitutional DNA and host-local DNA are separate facts and must be exactly equal.

## 13. Deployment is capped by the global evidence lease

Immediately before #154, the global evidence lease must still validate at current host time.

The requested deployment return horizon is five seconds, but the actual host context receives:

`min(global_evidence_valid_until, final_now + 5 seconds)`.

#154 then independently intersects that host context with #117 semantic freshness.

The coordinator explicitly rejects any deployment result whose validity exceeds the final evidence lease or whose verification time predates the global evidence verification time.

The wire `current_freshness.lease_until_ms` is checked against the same final horizon.

## 14. Wire receipt exposes the composition horizon

`VerifiedCurrentOperationalFreshnessReceipt` carries:

- `composition_evidence_lease_protocol`;
- `composition_evidence_verified_at_ms`; and
- `composition_evidence_valid_until_ms`.

These fields are audit evidence only. They do not replace the deployment authority/evidence digests or the embedded deployment-bound freshness receipt.

## 15. Semantic and deployment identities remain separate

The wire receipt preserves #117 semantic authority/evidence identity and separately exposes #154 deployment authority/evidence identity. Live consumers must use the deployment identity.

The dynamic lease is not folded into semantic authority identity merely because it constrains reuse.

## 16. No latest-record or model heuristic

Highest generation, newest timestamp, arrival order, absence of a later DHT record, author identity, reputation, Phi or model output cannot establish current authority.

## 17. Fail closed

Missing constitution/adoption/probe/source/witness/policy/transition-discovery/record-proof/authority-proof verifier, malformed or expired lease, empty lease intersection, host `dna_info` failure, DNA mismatch, changed constitutional head, failed pure qualification or any attempted lease widening denies.

## 18. Deliberately unprovisioned

The current-freshness verifier and all new leased/composition roles remain absent from `dna.yaml`. No external effect is enabled and `operational = false` remains explicit.

## 19. Required qualification before provisioning

At minimum:

- native/Clippy/WASM qualification;
- #181 evidence-lease algebra tests;
- #148/#154/#159 pure tests;
- proof that only leased source/witness/policy endpoints are consumed;
- source-head challenge equality across the coordinator and #130 adapter;
- transition leases intersected across the full lineage;
- control-plane leases intersected across every required probe;
- root/policy/control-plane/operational/semantic lease intersection;
- final #154 host-context horizon capped by the global lease;
- stale member lease causing whole-request denial;
- proof-verifier outage/substitution/stale-window denial;
- #91 truncated-prefix, fork, parent-digest and endpoint mismatch denial;
- direct/witness coverage paths;
- constitutional race and host-DNA mismatch denial; and
- adversarial multi-agent transition omission/insertion/reordering/replay tests.
