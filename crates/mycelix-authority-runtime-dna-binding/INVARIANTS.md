# Runtime DNA Binding v0.1 — Normative Invariants

Status: **pure deployment-binding candidate; no HDK provider or DNA provisioning**

This layer closes one deployment-specific gap above the substrate-neutral authority stack.

The semantic constitution and authority model remain portable across runtimes. Live Holochain authority, however, must also prove that the verified constitutional receipt belongs to the exact DNA in which the decision is being made.

## 1. Exact local DNA equality is mandatory

A positive `QualifiedRuntimeDnaAuthority` requires:

`evidence.current_constitution.dna_hash == local_dna.dna_hash`

String non-emptiness is not enough.

Wrong DNA, prior DNA after migration, same institution IDs on another DNA, or same constitutional statement on another DNA must deny live deployment authority.

## 2. Expected DNA identity is an independent adapter input

`VerifiedLocalDnaContext` is evidence-shaped data, not authority by deserialization.

A production HDK adapter MUST construct it from the current local host/cell context.

The expected DNA hash MUST NOT come from:

- the external authorization caller;
- `CurrentOperationalAuthorityEvidence.current_constitution.dna_hash` itself;
- a cached remote peer assertion;
- DHT consensus;
- model output; or
- a text field copied from the authority request.

If the runtime cannot independently obtain/verify its local DNA identity, qualification denies.

## 3. Constitutional statements stay substrate-neutral

The exact DNA hash is not added to every `ConstitutionStatement` or amendment digest.

The governance constitution remains semantically portable. The deployment layer binds a semantically current authority to the concrete runtime in which it is being exercised.

This preserves a useful distinction:

- **semantic current authority** — portable audit/institutional identity;
- **deployment-bound current authority** — semantic authority + exact local DNA hash.

## 4. The complete current-authority chain is re-run

This layer does not accept a serialized `QualifiedCurrentOperationalAuthority` from a provider.

After checking exact local DNA equality it calls `qualify_current_operational_authority` over the complete evidence-shaped #120 input.

Therefore the deployment result still depends on all lower-layer checks:

- current constitution/root;
- covered control-plane policy currentness;
- current operational policy context;
- fresh source/witness coverage; and
- complete current target lineage.

DNA equality is an additional necessary condition, never a replacement for those conditions.

## 5. Deployment identity commits exact DNA hash

`deployment_qualification_digest` commits:

- the exact #120 semantic qualification digest/profile; and
- the exact local DNA hash.

Changing DNA hash changes deployment authority identity even when all semantic constitutional/policy/authority bytes remain identical.

This is intentional during DNA migration or code/modifier changes.

## 6. Semantic identity remains separately available

The positive result retains the #120 semantic qualification digest/profile separately from the deployment-bound digest/profile.

Historical audit may therefore answer:

> Was this authority semantically valid under the institution at time T?

without implying:

> Is it currently live in this different DNA deployment?

Old-DNA semantic authority cannot be converted into new-DNA live authority by reusing the old deployment proof.

## 7. Dynamic local-DNA verification is evidence, not stable authority identity

The stable deployment identity includes the DNA hash but excludes dynamic verifier timestamps and provider refs.

`deployment_evidence_digest` separately commits:

- deployment qualification digest;
- #120 current evidence digest;
- local DNA source ref;
- local DNA verification ref; and
- local DNA verification window.

Refreshing local-host proof metadata for the same DNA may therefore change evidence provenance without changing deployment authority identity.

## 8. Conservative currentness

Deployment currentness is bounded by the minimum of:

- #120 current-authority validity; and
- verified local-DNA-context validity.

Verification time is the maximum of both evidence boundaries.

The projected `VerifiedAuthorityFreshness` is revalidated after this narrowing.

A stale local DNA context cannot keep an otherwise-current authority live.

## 9. Downstream freshness projection carries deployment proof

`to_verified_freshness()` preserves the generation-bound authority snapshot but replaces/narrows dynamic verification metadata with the deployment-bound verification ref/window.

Consumers that require live Holochain authority should consume the deployment-qualified projection, not the earlier #120 projection directly.

## 10. Positive result is non-deserializable

`QualifiedRuntimeDnaAuthority` derives `Serialize` but not `Deserialize`.

Transporting its JSON/bytes does not recreate positive authority in another process. A runtime must re-run qualification against its own independently obtained local DNA context.

## 11. No caller override path

A future HDK entrypoint MUST NOT expose `expected_dna_hash`, `local_dna_hash`, or equivalent as an externally authoritative request field.

The caller may carry informational deployment expectations, but the local host-derived identity is the only comparison authority.

## 12. No advisory shortcuts

Phi, reputation, stake, Guardian status, model confidence, Symthaea recommendations, user role, caller identity, or source popularity cannot waive exact DNA equality.

## 13. Containment

This crate is pure Rust.

It performs no:

- HDK/Holochain calls;
- DNA-info lookup itself;
- persistence;
- DHT query;
- remote call;
- signature verification;
- lifecycle mutation; or
- external effect.

The later HDK adapter owns local DNA discovery and must remain unprovisioned until this pure gate and the upstream stack have executable green evidence.

## 14. Required qualification before production provisioning

At minimum:

- exact match => allow qualification to continue;
- same statement + wrong DNA => deny;
- same institution/network text + wrong DNA => deny;
- prior DNA after migration => deny new live authority;
- stale local-context lease => deny;
- empty/malformed local hash => deny;
- stable semantic digest + changed DNA => changed deployment digest;
- refreshed local proof + same DNA => same deployment digest, changed evidence digest;
- positive output remains non-deserializable; and
- future HDK audit proves expected DNA comes from local host context, not caller input.

Related: #121, #120, #117, #116, #115, #114, #111, #109.
