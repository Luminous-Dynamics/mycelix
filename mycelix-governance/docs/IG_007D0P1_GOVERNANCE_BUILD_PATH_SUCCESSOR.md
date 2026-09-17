# IG-007D0P1 — repository-contained governance build-path successor

## Purpose

D0P1 is a monotonic successor to the qualified D0 source-intent profile. It records one newly discovered build-portability property without rewriting historical D0 evidence.

Historical D0 correctly froze this source intent:

```text
mycelix-governance/flake.nix
  import ../../nix/modules/holochain-base.nix
```

In a standalone `mycelix` checkout that path escapes the repository root. The repaired product subject changes only that import to:

```text
import ../nix/modules/holochain-base.nix
```

which resolves to the repository-tracked module:

```text
nix/modules/holochain-base.nix
```

## Identity

```text
profile   mycelix-governance-source-intent-ee3582af-v2
authority SourceIntentSuccessorBound
ceiling   RepositoryContainedGovernanceBuildPathBound
SHA-256   d2cd07e94274ba9ce8395ab5213db656cd33aa022005de065f8ca3715c0099bd
```

Repaired product subject:

```text
ee3582afdf7866d8cf1096cf4996e127d35a8a5e
```

## Predecessor

Qualified historical D0:

```text
profile   mycelix-governance-source-intent-20452d0c-v1
SHA-256   557282ebfdf98c14194b34db3412a6eadd97fabab06073cc5772b87101561003
source    20452d0cde9448c424d708d6307aa33a3d1901e3
evidence  fe3e9097ed19ff1ee29fd49b908e2b5086f06dd1
run       35154741664 PASS
```

D0P1 does not modify those bytes or reinterpret that PASS.

## Exact product delta

Relative to the historical source subject, the repaired product is exactly one commit and one file with one import-path replacement.

Unchanged capsule/source bindings:

```text
mycelix-governance/flake.lock
  faae625ed2fb9a7ecac236a99e342f7bdc94a644

nix/modules/holochain-base.nix
  e9015df8f82520d8c3607026de126216412727b6

mycelix-governance/Cargo.lock
  f872b74f13264e7345be02b64270179eed56c703

mycelix-governance/dna/dna.yaml
  972a9f32d43ce645aa71900eda11d1756d2e7055
```

Repaired flake blob:

```text
70006c71f02a560bd873526dd8b882cbba9e639f
```

## Qualification theorem

The qualifier independently proves:

```text
historical ../../nix/... path -> escapes standalone checkout
successor  ../nix/... path    -> resolves inside checkout
resolved target               -> exact bound shared module
```

It then checks out the repaired product into an arbitrary nested directory whose parent has no `nix/` compatibility shim, installs Nix through a pinned action, and requires the governance flake to evaluate with the committed lockfile unchanged.

This is deliberately a build-path/evaluation theorem, not a build-output theorem.

## Claim ceiling

A PASS may establish only:

`RepositoryContainedGovernanceBuildPathBound`

It does not establish a successful zome build, reproducible WASM bytes, Holochain `WasmHash`, DNA bundle identity, `DnaHash`, installed/runtime identity, or deployment currentness.

## Successor boundary

D1A must consume the exact qualified D0P1 product subject rather than the historical escaped-path subject. D1A is responsible for clean double-build byte reproducibility; D1B separately qualifies Holochain-native per-zome identities.
