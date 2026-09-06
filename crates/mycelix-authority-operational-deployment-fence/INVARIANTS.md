# Operational Deployment Fence v0.1 — Normative Invariants

Status: **pure qualification kernel; no runtime provisioning and no external effects**

This layer answers only:

> Is one already locally qualified, non-deserializable operational-currentness proof live in this exact host DNA and exact current constitutional epoch?

## 1. Semantic currentness must already exist locally

The positive input is `&QualifiedOperationalSubjectFreshness` from #117.

The deployment fence MUST NOT accept a transportable `VerifiedAuthorityFreshness`, provider-created `Qualified*` bytes, or an evidence-shaped reconstruction bundle as a substitute.

This is intentionally different from the older #120/#123 adapter line: the current Holochain coordinator already reconstructs #111 → #115 → #116 → #117 inside one process through stronger role-separated verifier boundaries. The deployment layer must preserve that architecture rather than reopen an omnibus evidence adapter.

## 2. Local DNA and constitutional DNA are different evidence

The exact local DNA identity is a host-runtime fact.

The constitutional plane's `dna_hash` is an independently produced constitutional binding. A positive deployment requires exact equality:

`binding_constitution_dna_hash == host_local_dna_hash`.

Neither value may substitute for the other.

## 3. Exact constitutional epoch is part of live deployment authority

Stable deployment identity commits the exact current `ConstitutionStatement` digest/profile in addition to the local DNA hash.

Thus two requests with identical semantic operational authority but different constitutional statements cannot share the same live deployment authority identity.

## 4. Semantic and deployment identities remain separate

The #117 semantic `authority_digest/profile` remains unchanged and suitable for substrate-neutral audit.

`deployment_authority_digest/profile` additionally commits:

- exact #117 semantic authority digest/profile;
- exact bootstrap-root qualification digest/profile;
- exact operational-context digest/profile;
- exact current constitutional statement digest/profile; and
- exact host DNA hash.

A DNA migration or constitutional transition changes live deployment identity even if a lower semantic object were otherwise byte-identical.

## 5. Dynamic evidence is separate from stable authority

`deployment_evidence_digest/profile` commits the stable deployment identity plus:

- exact #117 fresh evidence digest/profile;
- fixed host-DNA source identity;
- host observation time; and
- host-context validity horizon.

Re-observing the same DNA under the same constitution may refresh evidence without minting a new stable deployment authority identity.

## 6. Host DNA context is non-deserializable

`HostLocalDnaContext` derives `Serialize` but not `Deserialize`.

A runtime may construct it only in-process after obtaining the local DNA hash from its host/cell context. A caller cannot recreate this positive host-context object by sending wire bytes.

The runtime integration must statically bind construction to `dna_info()?.hash`.

## 7. Positive deployment result is non-deserializable

`QualifiedDeploymentOperationalFreshness` derives `Serialize` but not `Deserialize`.

A different cell or runtime must re-run local deployment qualification against its own host DNA and current constitutional projection.

## 8. Leases only narrow

Deployment verification time is the maximum of #117 verification and host observation time.

Deployment validity is the minimum of #117 freshness lease and host-context validity.

The projected `VerifiedAuthorityFreshness` is revalidated after its verification reference/window are replaced/narrowed to the deployment proof.

No composer may lengthen authority or evidence lifetime.

## 9. Pure separation

This crate contains no HDK/Holochain calls, DHT lookup, persistence, policy discovery, cryptographic proof service, lifecycle mutation, execution action, Phi, reputation, stake, Guardian override, or model-score authority.

Host retrieval belongs to the runtime adapter; semantic currentness belongs to #111/#115/#116/#117.

## 10. Runtime acceptance target

Before the current-freshness coordinator may emit a live result it must, in order:

1. locally construct #117 `QualifiedOperationalSubjectFreshness`;
2. obtain `dna_info()?.hash` from the local host;
3. re-project the binding current constitution and confirm it still equals the root epoch;
4. sample host time after those evidence-producing calls;
5. construct non-deserializable `HostLocalDnaContext`;
6. run `qualify_operational_freshness_for_deployment`; and
7. return only deployment-fenced freshness/identities.

No external effect is authorized by this qualification alone.
