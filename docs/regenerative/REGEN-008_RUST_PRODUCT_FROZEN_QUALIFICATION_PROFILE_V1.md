# REGEN-008 — Rust ProductFrozen Qualification Profile v1

Status: normative refinement of REGEN-008. This profile defines what REGEN means by a frozen Rust package graph and what that state does **not** prove.

## 1. Purpose

REGEN-008 distinguishes `ProductFrozen`, `ExecutionResolved`, and `Unresolved` dependency identity. This refinement makes the Rust `ProductFrozen` state executable enough for REGEN-010+ qualification.

The key refinement is:

```text
frozen Cargo package graph
!= hermetic build environment
```

A qualification receipt must preserve those propositions separately.

## 2. Rust ProductFrozen minimum

A Rust qualification subject may claim `dependency_state=ProductFrozen` only when all of the following are true before execution:

1. the exact `Cargo.toml` bytes are part of the immutable ProductHead;
2. the exact applicable `Cargo.lock` bytes are part of the ProductHead or are cryptographically bound by an immutable qualification manifest in that ProductHead;
3. qualification invokes Cargo with `--locked` for every dependency-resolving command;
4. the exact Rust/Cargo toolchain identity is recorded;
5. the qualified feature set is explicit;
6. the qualified target is explicit when target-dependent resolution/behavior may matter;
7. path dependencies are inside the same immutable source subject or have their own exact source identity;
8. no qualification step runs `cargo update`, deletes/recreates the lock, or otherwise changes the dependency graph.

If any of these conditions is absent, the result must be classified more narrowly.

## 3. Preferred standalone-crate layout

For dependency-light REGEN crates, the preferred first profile is a checked-in crate-local lock:

```text
crates/<regen-crate>/Cargo.toml
crates/<regen-crate>/Cargo.lock
crates/<regen-crate>/src/...
```

The lockfile is evidence, not build litter.

Qualification commands become conceptually:

```text
cargo test --manifest-path crates/<regen-crate>/Cargo.toml --locked
cargo test --manifest-path crates/<regen-crate>/Cargo.toml --locked --features serde
cargo clippy --manifest-path crates/<regen-crate>/Cargo.toml --locked --all-targets --features serde -- -D warnings
```

`cargo fmt` does not resolve dependencies and therefore does not require `--locked`.

## 4. Lock mutation is a product mutation

Under ProductFrozen:

```text
Cargo.lock changes
=> ProductHead changes
=> prior qualification does not automatically transfer
```

CI must not regenerate the lock and then delete it under a ProductFrozen claim.

That behavior is appropriate only for explicitly `ExecutionResolved` bootstrap lanes.

## 5. Lock identity evidence

The qualification lane SHOULD print at least:

```text
dependency_state=product_frozen
cargo_manifest_sha256=<digest>
cargo_lock_sha256=<digest>
```

and SHOULD verify that both files are present in `HEAD` before executing Cargo.

The receipt should also retain the ProductHead SHA so the path dependency bytes remain bound to the same source candidate.

## 6. Toolchain identity

At minimum record:

```text
rustc --version --verbose
cargo --version --verbose
```

A simple `rustc 1.xx` string is useful but weaker than the verbose identity containing host/commit/release information.

Toolchain identity remains separate from dependency identity.

## 7. Feature identity

A PASS qualifies only the executed feature set.

For example:

```text
features=[]
```

and:

```text
features=[serde]
```

are distinct propositions if both are executed.

A receipt SHOULD enumerate the exact intended feature sets rather than describe them only in prose.

If `--all-features` is used, that command is its own qualification proposition and does not necessarily replace explicitly important minimal/default feature campaigns.

## 8. Target identity

Record the host/target triple whenever target-specific behavior or dependency resolution could matter.

A ProductFrozen lock may contain target-conditional dependencies; therefore:

```text
same source + same lock + different target
```

can still produce a different executable proposition.

## 9. Path dependencies

Path dependencies are not fully identified by `Cargo.lock` alone.

For REGEN, they are acceptable under ProductFrozen only when the path dependency is bound by the same immutable ProductHead tree or by another exact immutable source identity.

Thus:

```text
Cargo.lock
+ ProductHead tree
```

jointly identify crate-local external packages and local path-source bytes.

## 10. Registry and Git dependencies

For registry dependencies, the lockfile package version/source/checksum information is part of the frozen graph.

For Git dependencies, the resolved exact source revision must be present in the lock/closure. A floating branch/tag declaration in `Cargo.toml` does not weaken ProductFrozen if the executed `--locked` graph resolves through an already-frozen exact lock revision; changing that revision requires changing the product lock.

## 11. Native/system dependency boundary

Cargo package freezing does not necessarily freeze:

- system libraries discovered by build scripts;
- C/C++ toolchains;
- `pkg-config` results;
- linkers;
- libc/kernel behavior;
- GPU drivers;
- external command-line tools;
- mutable remote services.

Therefore a receipt SHOULD classify system/environment closure independently.

Suggested vocabulary:

```text
system_closure=unfrozen
system_closure=nix_frozen
system_closure=container_digest_frozen
system_closure=other_exact
```

This vocabulary is descriptive, not an assertion that all mechanisms have equivalent strength.

## 12. ProductFrozen is not Hermetic

The minimum Rust ProductFrozen claim means:

> the intended Cargo dependency graph is immutable before the run.

It does **not** mean:

```text
no network access
all source artifacts already local
bit-for-bit reproducible binary
identical linker output
identical host kernel
identical native libraries
supply-chain security
scientific validity
```

Those require additional propositions.

## 13. Optional stronger fetch isolation

A stronger campaign may use a pre-populated immutable dependency source closure and execute with no dependency-network access.

Examples may include:

- Nix store closure;
- vendored crates with exact digest;
- immutable container/source cache.

If used, the closure itself needs an exact identity in the qualification receipt.

`--offline` alone is not a complete provenance theorem; it only constrains Cargo's network behavior relative to the local cache.

## 14. Cargo metadata evidence

A ProductFrozen campaign MAY record `cargo metadata --locked` output or a normalized digest of the resolved graph as supplemental evidence.

This is useful for review but does not replace the checked-in lock.

If metadata is hashed, path-dependent/environment-dependent fields should be normalized by a documented deterministic procedure before claiming cross-run identity.

## 15. Post-command lock immutability

Every Cargo command executed under ProductFrozen SHOULD be followed, directly or at campaign end, by proof that the checked-in lock bytes remain unchanged.

The final checkout-hygiene step remains mandatory.

Unexpected lock mutation is a qualification failure, not something CI should automatically repair.

## 16. Qualification receipt minimum

A mature machine-readable Rust qualification receipt SHOULD contain at least:

```text
subject_kind=ProductHead
subject_sha=<git sha>
dependency_state=ProductFrozen
manifest_path=<path>
manifest_sha256=<digest>
lock_path=<path>
lock_sha256=<digest>
rustc_identity=<verbose identity or digest>
cargo_identity=<verbose identity or digest>
host_target=<triple>
feature_campaigns=<explicit list>
system_closure=<classification>
fixture_digests=<where applicable>
commands=<exact command contract or digest>
result=<PASS|FAIL|UNEXECUTED>
```

A receipt is evidence about one campaign. It is not authority to reinterpret what the campaign proved.

## 17. REGEN-010 requirement

The first executable soil-evidence implementation SHOULD use this ProductFrozen Rust profile unless a reviewed exception explicitly states a weaker dependency proposition.

The preferred REGEN-010 gate is therefore:

```text
qualified REGEN-009 ProductHead
+ REGEN-010 ProductHead
+ checked-in qualification Cargo.lock
+ Rust/Cargo 1.96.0 exact identity
+ cargo --locked campaigns
+ explicit feature set
+ explicit system-closure classification
```

This strengthens software evidence only. It does not establish any real soil measurement or agronomic conclusion.

## 18. Drift rule

Changing any of the following after evidence collection begins creates a new software execution lineage unless equivalence is separately established:

- ProductHead source;
- Cargo manifest;
- Cargo lock;
- qualified feature set;
- material target/toolchain identity;
- frozen system closure where claimed.

Evidence from distinct execution lineages must not be silently pooled as though they came from one unchanged implementation.

## 19. Supply-chain boundary

A fully frozen graph may contain a vulnerable or malicious dependency.

```text
ProductFrozen
!= trusted
!= vulnerability-free
!= audited
```

Xenia/Nix/software-supply-chain work may later strengthen provenance/security around the closure, but those remain separate claims.

## 20. Deliberate non-claims

This profile establishes no bit-for-bit build reproducibility, package security, native-library closure unless separately declared, scientific correctness, agronomic efficacy, climate/carbon authority, governance authority, or physical-action authority.

Its theorem is intentionally narrower:

> a REGEN Rust qualification may call its Cargo dependency graph ProductFrozen only when the exact graph is immutable before execution and every dependency-resolving qualification command is forced to use that frozen graph.
