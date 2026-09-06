# Authority Current Freshness Verifier Runtime v0.2 — Normative Invariants

Status: **implemented composition candidate; deliberately unprovisioned in the binding governance DNA**

## 1. Binding constitutional authority is reused

Current constitutional truth comes only from `constitution_transition::get_verified_current_constitution(())`.

The coordinator locally rechecks the statement digest and requires `legacy_constitution_authoritative = false`.

## 2. Root manifest discovery is not root authority

`authority_state_bootstrap_root_manifest_provider` returns only one semantic manifest candidate. It cannot supply constitutional currentness, root-adoption proof validity, #111 adoption or a qualified root.

## 3. Root-adoption proof verification is not root-adoption authority

The coordinator builds `BootstrapRootAdoptionClaim` locally through #148. The proof verifier receives that exact claim and returns only `VerifiedBootstrapRootAdoptionProof`.

After the post-proof constitutional fence, #148 is run locally and only then is #111's adoption ABI projected and passed into `qualify_bootstrap_root`.

## 4. Positive semantic authority is reconstructed locally

The semantic positive path is:

`binding constitution → #148 adoption → #111 root → #115 policy currentness → #116 operational policy context → #117 operational currentness`.

No provider-supplied serialized `Qualified*` value is accepted as authority.

## 5. Host DNA is independent deployment evidence

After #117 succeeds, the coordinator MUST obtain the local cell DNA directly through:

`dna_info()?.hash.to_string()`.

It MUST NOT source local DNA from:

- caller input;
- the constitutional receipt itself;
- a provider response;
- a DHT record; or
- cached application state.

The binding constitutional plane's DNA hash and host-local DNA hash are separate facts. Exact equality is required inside the pure deployment fence.

## 6. Constitutional TOCTOU spans host-DNA binding

The final ordering is:

`#117 semantic qualification → host dna_info → host observation time → binding constitution FINAL → exact root-epoch equality → final host time → deployment qualification`.

Thus a constitutional transition during semantic evidence collection or host-DNA observation denies before a live deployment receipt can be returned.

## 7. Deployment authority is constructed locally

The coordinator constructs non-deserializable `HostLocalDnaContext` in-process from the host DNA observation and calls `qualify_operational_freshness_for_deployment` with the non-deserializable #117 result.

The deployment fence exact-binds:

- #117 stable semantic authority;
- exact root qualification;
- exact operational context;
- exact current constitutional statement digest/profile; and
- exact host DNA hash.

The wire response preserves semantic authority/evidence identities separately while exposing distinct deployment authority/evidence identities.

Live consumers MUST use the deployment identity, not the semantic-only identity.

## 8. Old omnibus deployment adapter is not reintroduced

The runtime MUST NOT reconstruct a #120 `CurrentOperationalAuthorityEvidence` bundle or call an evidence-shaped #123 adapter.

Those historical pure layers established the host-DNA invariant, but the active runtime now has stronger independent verifier boundaries. Reopening an omnibus evidence bundle would collapse those trust domains.

Deployment qualification therefore consumes `&QualifiedOperationalSubjectFreshness` directly inside the same process.

## 9. Leases only narrow

The host context is bounded to at most `DEPLOYMENT_RETURN_LEASE_MS` after the final fence.

The deployment result cannot outlive #117 semantic freshness or the host-context lease. #148/#111/#116 already impose their own earlier ceilings.

No composer may lengthen evidence or authority lifetime.

## 10. Probe/source/witness/transition roles remain independent

Evidence planning is discovery only. #114 owns probe provenance. #131 independently re-verifies probe provenance before source authentication. Witness/trust and transition verification remain separate roles. #91 owns exact covered endpoint projection.

A valid transition prefix is not current authority.

## 11. No latest-record heuristic

Highest generation, newest timestamp, DHT arrival order, absence of a later record, author identity, reputation, Phi or model output cannot establish current authority.

## 12. Fail closed

Missing constitution/adoption-proof/probe/source/witness/transition verifier, host `dna_info` failure, DNA mismatch, stale proof, changed constitutional head, failed pure qualification or expired lease denies.

## 13. Deliberately unprovisioned

The current-freshness verifier remains absent from `dna.yaml`. No external effect is enabled and `operational = false` remains explicit.

## 14. Required qualification before provisioning

At minimum:

- native/Clippy/WASM qualification;
- #148 adoption proof qualification;
- direct binding-constitution calls;
- exact host `dna_info()?.hash.to_string()` derivation;
- constitutional/host DNA mismatch denial;
- final constitutional recheck after host observation;
- deployment identity changing across DNA and constitutional epochs;
- semantic identity preserved separately from deployment identity;
- short deployment lease narrowing;
- proof/source/witness/transition substitution denial; and
- adversarial multi-agent constitutional-race, DNA-mismatch and replay tests.
