# IG-007D0 — Holochain governance source-intent profile

## Purpose

This is the first, repository-provable layer of #1313's deployment-currentness ladder.

It freezes **what the current source tree intends to build** for the Mycelix governance DNA. It does not claim that those artifacts were built, installed, loaded, running, fresh, continuously executing, or authorized as the current production deployment.

The central invariant is:

```text
source intent
!= built identity
!= installed Holochain identity
!= running runtime
!= fresh/continuous witness
!= authorized current deployment
```

## Identity

```text
profile   mycelix-governance-source-intent-20452d0c-v1
authority SourceIntentBound
ceiling   SourceIntentOnly
SHA-256   557282ebfdf98c14194b34db3412a6eadd97fabab06073cc5772b87101561003
```

Exact source subject:

```text
20452d0cde9448c424d708d6307aa33a3d1901e3
tree 4d24feadca2f076fac8e781f16350ce313f88995
```

## Bound source inputs

```text
mycelix-governance/dna/dna.yaml
  blob 972a9f32d43ce645aa71900eda11d1756d2e7055

mycelix-governance/Cargo.toml
  blob 219cbd48e02692c75a49f0b04a4130bf897aaa5c

mycelix-governance/Cargo.lock
  blob f872b74f13264e7345be02b64270179eed56c703
```

The exact tree has no `rust-toolchain` or `rust-toolchain.toml` file. This is preserved as an **absence of a repository-local compiler pin**, not as permission to choose an arbitrary toolchain or as a reproducible-build theorem.

## DNA source intent

The bound `dna.yaml` declares:

- manifest version `0`;
- DNA name `mycelix_governance_dna`;
- network seed `mycelix-governance-v1`;
- nine integrity zomes;
- nine coordinator zomes;
- explicit WASM paths and coordinator dependency edges.

The profile preserves the ordered zome names, paths, and dependency lists.

## Declared Holochain dependency intent

The governance workspace declares the Holochain 0.6-family dependency surface including:

```text
hdk                       =0.6.1
hdi                       =0.7.1
holochain_integrity_types =0.6.1
holochain_zome_types      =0.6.1
holo_hash                 =0.6.1
hdk_derive                =0.6.1
holochain_serialized_bytes=0.0.57
```

These Cargo declarations are source intent only. They do not prove the actual conductor/core version used at build or deployment time.

## Positive controls

D0 establishes only that the exact source subject, DNA manifest, Cargo manifest, lockfile, network seed, zome set, and declared Holochain dependency versions are content-bound.

## Explicitly unqualified

D0 does not establish:

```text
RustToolchainPinQualified
BuiltWasmIdentityQualified
BuiltDnaBundleIdentityQualified
DnaHashQualified
HappBundleIdentityQualified
InstalledAppCellIdentityQualified
RuntimeProcessAssociationQualified
FreshRuntimeWitnessQualified
ContinuousRuntimeCurrentnessQualified
AuthorizedDeploymentCurrentnessQualified
DeploymentSafetyQualified
```

In particular, WASM filenames in `dna.yaml` are not artifact digests.

## Successor boundary

D1 must consume this exact D0 commitment and add reproducible build evidence: exact toolchain/environment, zome WASM digests, DNA bundle/content identity and resulting `DnaHash`, plus app/hApp identity where one exists.

D2-D5 require actual conductor/runtime/policy evidence and must not be synthesized from repository source.

## Nonclaims

No built DNA, installed app, live conductor, deployment presence/absence, runtime freshness, continuous execution, authorized deployment, or deployment safety is claimed.
