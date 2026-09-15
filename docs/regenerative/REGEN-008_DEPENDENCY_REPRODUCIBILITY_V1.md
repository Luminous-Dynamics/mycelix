# REGEN-008 — Dependency Reproducibility Contract v1

Status: normative architecture draft; no scientific, agronomic, climate, governance, market, or physical-action authority.

## 1. Purpose

REGEN qualification must identify the dependency graph actually used by the executed implementation.

An exact Git subject and exact compiler version are not enough when the build still resolves dependency versions from mutable semver ranges or a changing registry index.

The core theorem is:

```text
exact source SHA
+ exact toolchain
!= exact dependency graph
```

and therefore:

```text
source-qualified execution
!= reproducible build target
```

unless dependency resolution is also sufficiently identified.

## 2. Dependency states

Every consequential REGEN software qualification SHOULD classify its dependency state as one of the following.

### 2.1 ProductFrozen

The exact dependency graph is part of the immutable candidate or otherwise cryptographically bound to it before execution.

Examples include:

- checked-in `Cargo.lock` used with `--locked`;
- an immutable Nix flake/lock closure;
- an exact vendor tree digest;
- a signed dependency manifest whose complete graph is independently resolvable;
- an equivalent exact package graph with cryptographic package identities.

`ProductFrozen` is the preferred state for scientific/modeling qualification and release candidates.

### 2.2 ExecutionResolved

The dependency graph is resolved during the qualification run, but the resulting graph is recorded with an exact identity such as a lockfile SHA-256.

This is useful evidence for one execution, but it is weaker than ProductFrozen because a later run of the same source commit may resolve differently.

```text
ExecutionResolved lock digest
= identity of what this run resolved
!= promise that future runs resolve the same graph
```

### 2.3 Unresolved

The execution neither freezes nor records enough information to identify the dependency graph.

This state MUST NOT be described as reproducible merely because the source commit and language toolchain are known.

## 3. Promotion rule

The bootstrap identity/interop tranches may temporarily use `ExecutionResolved` if the generated dependency-lock digest is recorded in exact-head CI.

Before REGEN depends on a software result for scientific, field, climate-accounting, or consequential recommendation claims, the relevant implementation SHOULD graduate to `ProductFrozen` or document why an equivalent exact closure is sufficient.

Recommended gate:

```text
identity/bootstrap qualification
    may use ExecutionResolved

scientific/model qualification
    SHOULD use ProductFrozen

release / consequential integration
    SHOULD require ProductFrozen
```

## 4. Rust profile

For standalone Rust qualification crates, the preferred contract is:

```text
Cargo.toml
+ Cargo.lock
+ exact Rust/Cargo toolchain
+ cargo ... --locked
```

The checked-in lockfile is part of the candidate evidence.

If repository policy intentionally avoids committing a library lockfile, REGEN SHOULD use a dedicated qualification manifest/lock pair rather than silently returning to floating resolution.

## 5. Nix profile

Where Nix is the qualification environment, the exact flake/lock identity and relevant derivation/environment identity SHOULD be recorded.

A Nix lock can strengthen dependency/tool identity, but:

```text
Nix closure identity
!= scientific validity
```

and it does not replace software tests or experimental evidence.

## 6. Registry/package integrity

A dependency graph identity should distinguish package version from package content where the ecosystem allows it.

Useful evidence may include:

- registry/source identity;
- package name/version;
- package checksum;
- source commit for Git dependencies;
- feature selection;
- target-specific resolution where material.

`package@version` alone may not be sufficient if the underlying source artifact is mutable or insufficiently authenticated.

## 7. Feature identity

The dependency graph and compiled behavior can change with feature selection.

Qualification receipts SHOULD bind the feature/profile used by the campaign, for example:

```text
default features
serde features
holochain compatibility features
backend features
```

A PASS under one feature set MUST NOT silently qualify another.

## 8. Target identity

Where target-specific dependency edges or behavior are material, the qualification record SHOULD include target triple/architecture.

```text
same source
+ same lock
+ different target
```

may still be a different qualification proposition.

## 9. Generated lock handling

If an `ExecutionResolved` bootstrap run creates a lockfile that is not part of the product candidate, CI SHOULD:

1. record the lockfile cryptographic digest;
2. clearly label it execution-resolved;
3. optionally record other useful lock metadata;
4. remove only the known generated lock artifact before checkout-hygiene enforcement;
5. preserve any unexpected mutation as a failure.

It SHOULD NOT simply add `Cargo.lock` to a broad ignore rule to hide dependency drift.

## 10. Clean-checkout semantics

A qualification lane may intentionally generate build artifacts in ignored/isolated locations.

The clean-checkout theorem applies to unexpected source-tree mutation, not to the metaphysical absence of all temporary files.

Expected generated artifacts must be:

- isolated;
- explicitly identified;
- removed before final hygiene if they appear in Git status;
- or frozen into the candidate when they are part of the qualification contract.

## 11. Lock digest evidence

For `ExecutionResolved`, a run SHOULD print or emit a machine-readable field such as:

```text
dependency_state=execution_resolved
cargo_lock_sha256=<digest>
```

A stronger future receipt may include the full lock artifact or attach it as an immutable CI artifact.

The digest proves only which bytes were resolved for that execution.

## 12. Dependency drift detection

Repeated qualification of the same source/toolchain under ExecutionResolved MAY compare lock digests.

If they differ:

```text
same source
+ same compiler
+ different lock
```

must be treated as environment/dependency drift, not as the same reproducibility lineage.

## 13. Scientific lineage rule

Once a scientific experiment begins, dependency drift should follow the same evidence-lineage discipline as compiler/environment drift.

Conceptually:

```text
dependency drift before evidence
    -> reprepare / new qualification capsule

dependency drift after evidence begins
    -> do not silently mix lineages
```

A new exact dependency graph creates a new execution lineage unless equivalence is independently established.

## 14. Cross-language rule

REGEN-008 is not Rust-specific.

Equivalent exact dependency identity should be preserved for Python, TypeScript, WASM, native libraries, model runtimes, numerical backends, and external solvers where they materially affect the proposition.

Examples include lockfiles, environment manifests, container image digests, model/backend digests, or Nix closures.

## 15. Model/data dependencies

For Symthaea/modeling tranches, executable code dependencies and model/data dependencies are separate.

A qualified model experiment may require exact identity for:

- source code;
- package graph;
- model weights;
- configuration;
- input dataset/fixture;
- numerical backend;
- hardware-sensitive runtime where relevant.

Missing one component should remain explicit.

## 16. External services

A remote API or mutable external service can be a dependency even when it does not appear in a package lock.

Scientific qualification SHOULD either:

- eliminate the mutable service from the experiment;
- bind an immutable returned artifact/fixture;
- or explicitly classify the external service as an unresolved dependency.

## 17. Supply-chain boundary

Exact dependency identity improves reproducibility and narrows supply-chain ambiguity, but:

```text
frozen dependency
!= secure dependency
```

Security review, vulnerability assessment, provenance/signature verification, and maintainership risk remain separate concerns.

## 18. Relationship to REGEN-007

REGEN-007 answers:

> What exact subject executed?

REGEN-008 adds:

> What exact dependency graph/runtime closure did that subject execute with?

The combined qualification shape is:

```text
subject identity
+ toolchain identity
+ dependency state/identity
+ command contract
+ fixtures
+ result
```

Each component remains separately inspectable.

## 19. Immediate application

The current REGEN-002/003 bootstrap workflows may use `ExecutionResolved` while they record generated `Cargo.lock` SHA-256 values and enforce clean postflight state.

That state MUST be stated as weaker than a frozen dependency capsule.

Before REGEN-010+ becomes the basis for scientific soil-response/model claims, the relevant Rust qualification environment should graduate to a checked-in/frozen lock or equivalent exact dependency closure.

## 20. Deliberate non-claims

REGEN-008 does not establish dependency security, package authenticity, scientific correctness, agronomic efficacy, regulatory compliance, carbon authority, governance authority, or physical-action permission.

It freezes the requirement that dependency resolution itself has evidence semantics and cannot remain an invisible ambient property of a green build.
