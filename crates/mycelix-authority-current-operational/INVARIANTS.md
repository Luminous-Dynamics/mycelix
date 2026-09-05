# Current Operational Authority v0.1 — Normative Invariants

Status: **pure end-to-end composition candidate; no runtime provider or DNA provisioning**

This layer exists to solve an important adapter-boundary problem:

> positive authority objects are intentionally non-deserializable, but runtime providers still need a way to supply evidence.

The answer is not to make qualified authority deserializable. The answer is to transport only evidence-shaped candidate data and re-run the entire pure qualification chain locally.

## 1. End-to-end positive path

The only positive path is:

`candidate constitutional/root evidence`
→ `#111 qualify_bootstrap_root`
→ `candidate root-covered policy-state evidence`
→ `#115 qualify_control_plane_subject_freshness`
→ `candidate semantic operational policy receipts`
→ `#116 qualify_operational_policy_context`
→ `candidate target probe/source/witness/transition evidence`
→ `#117 qualify_operational_subject_freshness`
→ `QualifiedCurrentOperationalAuthority`.

No qualified positive object is accepted from a remote/runtime provider as an authority assertion.

## 2. Adapter inputs are evidence-shaped, not authority-shaped

`CurrentOperationalAuthorityEvidence`, `OperationalPolicyEvidence`, and `CoveredSubjectEvidence` derive `Deserialize` because they are transport candidates.

Their existence or successful deserialization does not establish:

- current constitution;
- bootstrap-root authority;
- policy currentness;
- source-head completeness;
- target currentness; or
- effect permission.

Every positive fact is recomputed by a pure qualifier.

## 3. Positive output is non-deserializable

`QualifiedCurrentOperationalAuthority` derives `Serialize` but not `Deserialize`.

A runtime cannot manufacture a positive current-authority object by sending bytes matching its shape.

## 4. Bootstrap root is always requalified

The composer accepts:

- exact `AuthorityStateBootstrapRootManifest`;
- independently verified current constitution receipt; and
- independently verified root-adoption receipt.

It always calls #111 `qualify_bootstrap_root`.

A serialized root qualification is not an input.

## 5. Control-plane policy currentness is always requalified

The composer accepts 1-3 candidate evidence bundles for control-plane policy subjects.

Each bundle is passed independently through #115 `qualify_control_plane_subject_freshness` under the exact newly qualified root.

The exact required 2/3 policy set is not selected by the evidence provider. #116 reconstructs it from the semantic operational policies and rejects missing, extra, wrong-root or wrong-identity control-plane proofs.

## 6. Operational policy context is always requalified

The exact semantic coverage/context receipts are passed to #116 together with the internally produced #115 policy-currentness proofs.

#116 independently checks:

- root epoch;
- semantic policy proof echoes;
- institution/rulebook binding;
- exact context→coverage digest binding;
- target namespace/kind coverage;
- witness mode/trust policy; and
- exact current policy subject set.

No serialized `QualifiedOperationalPolicyContext` is accepted as input.

## 7. Target currentness is always requalified

The target evidence bundle is passed through #117 using the newly constructed operational policy context.

That path requires:

- exact probe target and policy-context binding;
- exact #96 challenge/source/witness coverage;
- exact #91 complete covered transition lineage; and
- current-only freshness conversion.

A valid history prefix, DHT absence, or latest observed record is never enough.

## 8. Bounded evidence fan-in

Before expensive composition, v0.1 rejects:

- zero or more than 3 control-plane evidence bundles;
- more than 64 witness receipts per covered subject;
- more than 64 trust bindings per covered subject;
- zero transition receipts; or
- more than 256 transition receipts per covered subject.

Deeper semantic layers retain their own stricter validation and quorum/lineage bounds.

## 9. Exact target/root closure

After #117 qualification, the result must still echo:

- the exact requested target subject;
- the exact newly qualified bootstrap-root digest; and
- the exact newly qualified bootstrap-root profile.

This is defense in depth against accidental cross-root or cross-target composition.

## 10. Stable authority vs dynamic evidence remains separated

`qualification_digest` is stable current-authority identity and commits:

- exact bootstrap-root qualification identity;
- exact operational-policy-context qualification identity; and
- exact #117 stable operational-subject authority identity.

It excludes challenge/source/witness proof-instance identity.

`evidence_digest` separately commits:

- the stable end-to-end qualification digest; and
- #117's exact fresh evidence digest.

Re-probing the same root/policy/generation may change evidence provenance without changing stable authority identity.

Root, policy, lineage or generation changes do change stable authority identity.

## 11. Conservative final currentness

The final current horizon is the conservative minimum across:

- target freshness lease;
- bootstrap-root lease; and
- operational-policy-context lease.

The final verification time is the corresponding maximum.

The target `VerifiedAuthorityFreshness` is revalidated before output.

## 12. Existing freshness ABI remains projection-only

`QualifiedCurrentOperationalAuthority::to_verified_freshness()` exists only for existing downstream consumers such as executor/current-authority composition.

The projected freshness receipt retains:

- stable generation/state snapshot identity; and
- #117 evidence-bound verification ref.

Downstream runtimes should obtain this projection only after local end-to-end qualification, never deserialize it as proof by itself.

## 13. Historical state cannot enter the live path

No historical/as-of projection API is used or accepted.

Historical verification remains a separate audit operation and cannot be converted into this live current-authority object.

## 14. No observation heuristic or hidden provider authority

This crate contains no HDK/Holochain code and cannot infer currentness from:

- DHT arrival order;
- latest timestamp;
- highest observed generation;
- absence of a later record;
- provider confidence;
- local cache state;
- identity/reputation score;
- Phi/consciousness; or
- model output.

Providers may supply evidence candidates. They do not decide what constitutes current authority.

## 15. No effect authority

This result establishes currentness for one operational authority subject only.

It does not itself:

- execute actions;
- create a threshold signature;
- mint a delegation;
- create an executor designation;
- claim lifecycle work; or
- authorize an external effect.

Those later layers must consume the exact current-authority result under their own fail-closed semantics.

## 16. Pre-runtime qualification

Before implementing or provisioning `authority_current_freshness_verifier`, at minimum:

- rustfmt;
- native tests;
- warnings-denied Clippy;
- downstream compile of #111/#115/#116/#117;
- clean-genesis bootstrap test;
- missing/extra control-plane subject denial;
- wrong-root policy evidence denial;
- wrong target policy coverage denial;
- hidden later revocation denial;
- source-head omission denial;
- fork/gap denial;
- stale probe/context/root denial;
- same-generation re-probe stable-authority test;
- root/policy/generation rotation stable-identity change test; and
- proof that no qualified positive object appears in any deserializable input type.

The eventual HDK runtime should be a thin evidence collector around this pure theorem, not an alternative currentness implementation.
