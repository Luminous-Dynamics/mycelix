# REGEN-019D — ProductFrozen Shared-Admission Promotion Preregistration v1

Status: preregistration only. This document freezes the promotion and qualification boundary for the domain-neutral REGEN-019 shared PEF admission kernel. It does not itself promote, qualify, merge, deploy, or authorize any product.

## 1. Purpose

REGEN-019C prepares the independent `mycelix-regenerative-admission` crate. REGEN-019D will promote one exact successful machine-prepared capsule into an immutable ProductFrozen subject and qualify only those exact bytes.

Core theorem:

```text
successful preparation
+ exact prepared-byte promotion
+ exact ProductFrozen execution
+ machine-validated qualification receipt
= bounded shared-admission software theorem
```

not:

```text
workflow green
= intended toolchain used
= artifact promoted correctly
= scientific truth
= downstream domain validity
= authority
```

## 2. Governing upstream subjects

REGEN-019D is constrained by:

- REGEN-009 / #973 — exact PEF + regenerative identity source convergence;
- REGEN-008 / #972 — ProductFrozen dependency semantics;
- REGEN-008A / #1280 — material CI stages must explicitly bind/prove toolchain selection;
- REGEN-Q001 / #996 — machine-readable qualification receipt v1;
- REGEN-019 / #1237 — domain-neutral PEF admission semantics;
- REGEN-010B / #1262 and REGEN-010C / #1274 — qualified behavioral oracle for the existing soil consumer;
- REGEN-019C / #1277 — shared-admission source+dependency preparation.

The REGEN-019D product subject should be materialized over qualified REGEN-009 source convergence rather than inheriting preparation workflow/history as product ancestry.

## 3. Preparation prerequisite

Promotion MUST consume one exact REGEN-019C preparation run whose own evidence establishes all of the following:

1. exact intended preparation ProductHead;
2. exact REGEN-009 parent;
3. exact authored scope;
4. explicit Rust/Cargo/rustfmt 1.96.0 command selection at every material stage;
5. machine-generated `Cargo.lock` under that exact toolchain;
6. exact formatted source/test bytes;
7. locked tests PASS;
8. strict Clippy PASS;
9. clean authored checkout restoration;
10. exact uploaded preparation artifact identity.

A queued/in-progress/no-step run is not promotable.

A workflow marked `success` whose receipt records a different toolchain than the intended profile is not promotable into the 1.96 lineage.

The historical REGEN-019C artifact from run `35124775213` / artifact `10459191233` is explicitly excluded from 1.96 promotion because its receipt recorded Rust/Cargo 1.98.1.

## 4. Archive identity is not enough

The preparation ZIP/archive digest is transport evidence, not by itself the semantic prepared-file theorem.

Promotion MUST verify both:

```text
expected archive identity
AND
expected internal file map
```

The internal map MUST bind at least:

- `Cargo.toml` SHA-256;
- `Cargo.lock` SHA-256;
- `src/lib.rs` SHA-256;
- `tests/admission_matrix.rs` SHA-256.

If `PREPARED_FILES.sha256` is present, promotion MUST parse and verify its exact declared path set and digests rather than merely retaining the file.

Therefore:

```text
archive sha256 matches
!= every promoted member is correct
```

and:

```text
member digest matches
+ wrong path / extra member / missing member
!= valid promotion
```

## 5. Exact promotion set

The ProductFrozen product commit should materialize only the reviewed qualification plane plus the exact prepared crate bytes.

Expected product files are conceptually:

```text
.github/workflows/regen-shared-admission-product-frozen.yml
qualification/regen-019d/verify-regen-qualification-receipt.py
crates/mycelix-regenerative-admission/Cargo.toml
crates/mycelix-regenerative-admission/Cargo.lock
crates/mycelix-regenerative-admission/src/lib.rs
crates/mycelix-regenerative-admission/tests/admission_matrix.rs
```

No REGEN-019C preparation workflow belongs in ProductFrozen product ancestry.

No formatter-only, lock-update, repair, or promoter commit may sit between the declared product parent and ProductHead.

If any prepared crate byte needs manual correction, REGEN-019C must produce a new preparation ProductHead/run/capsule first. REGEN-019D must not repair prepared product bytes during promotion.

## 6. Qualified Q001 validator fixture

REGEN-019D must validate its emitted receipt with the exact already-qualified REGEN-Q001 semantic validator, not a reimplementation that merely resembles it.

Qualified REGEN-Q001 ProductHead:

```text
9a9dfea07d5b19707df5f5c7c70c5b8bfe68092a
```

Exact validator Git blob:

```text
653949df6a5dd6f37cb32eff57611a308846cdc7
```

Exact schema Git blob for reference:

```text
d7e6552c47f9fbea7033bdaa59d2078dee1b286b
```

The preferred ProductFrozen construction copies the validator bytes unchanged into the qualification-only path above and proves its Git blob equals `653949df...` before use.

Because Git blob identity is content-based, copying the exact bytes to a qualification-only path preserves the semantic byte identity while avoiding a live cross-branch/network dependency during qualification.

The copied validator is qualification tooling, not product runtime API.

## 7. ProductFrozen dependency theorem

Before any dependency-resolving command, the ProductHead MUST already contain the exact checked-in `Cargo.lock` promoted from the successful preparation capsule.

Every Cargo command must use both explicit toolchain selection and `--locked`, for example:

```text
cargo +1.96.0 test --manifest-path ... --locked
cargo +1.96.0 clippy --manifest-path ... --all-targets --locked -- -D warnings
```

Qualification MUST NOT regenerate or update the lock.

The receipt records:

```text
dependency_state=ProductFrozen
system_closure=unfrozen
```

unless a stronger independently proven system-closure profile is actually used.

ProductFrozen Cargo identity remains distinct from hermetic system closure.

## 8. Cross-step toolchain binding

REGEN-008A applies to every material stage.

The qualifier MUST NOT rely solely on `rustup override set` performed in an earlier workflow step.

At minimum the following operations must explicitly select or immediately prove Rust 1.96.0:

- `rustc` identity recording;
- `cargo` identity recording;
- `rustfmt` identity recording;
- formatting check;
- tests;
- Clippy;
- any Cargo metadata/resolution operation used by the theorem.

The qualifier fails if the recorded receipt/toolchain identity disagrees with the intended profile, even if all tests otherwise pass.

## 9. Historical qualifier routing

A ProductFrozen qualifier is an immutable-subject theorem, not a generic CI workflow for every descendant.

The REGEN-019D workflow should therefore combine:

1. ordinary path/event triggering so the ProductFrozen PR can execute; and
2. a head-branch-specific job condition for the dedicated ProductFrozen branch.

Conceptually:

```text
run qualification job only when head_ref == dedicated REGEN-019D product branch
```

Inside that job, exact ProductHead/parent/path guards remain mandatory.

A later child branch should observe the historical qualifier as skipped/not-applicable rather than intentionally failing its exact-parent theorem.

This routing changes CI signal semantics only. It does not weaken the exact-subject guard for REGEN-019D itself.

## 10. Exact upstream source anchors

Qualification should bind the exact PEF source identities already qualified by REGEN-009, including:

- `planetary_evidence.rs` blob `2069d6e56336d05c7bc293abfc5ccdc421ab53cb`;
- `planetary_lineage.rs` blob `e0fda7c4c9e83c46761dae1e4fff2c43e3d6d41b`;
- `planetary_product.rs` blob `dedc20b201759883b9c58b39905dfc700c1cfb21`;
- `mycelix-core-types/src/lib.rs` resolved blob `3761f5cfcb6a7c78c7ce829ba4283f7e522aff72`.

This proves the shared admission crate is executing against the intended PEF semantics rather than a coincidentally compiling different source surface.

## 11. Qualification campaign

The dedicated exact-head campaign should require, in order:

1. exact ProductHead checkout;
2. exact sole product parent;
3. exact changed-path census;
4. exact promoted prepared-file SHA-256 identities;
5. exact Q001 validator blob identity;
6. exact qualified upstream PEF anchors;
7. explicit Rust/Cargo/rustfmt 1.96.0 identities;
8. format check without mutation;
9. locked shared-admission matrix execution;
10. strict all-target Clippy with warnings denied;
11. post-execution manifest/lock/source/test/validator immutability;
12. clean checkout;
13. Q001 receipt generation;
14. exact Q001 semantic validation of that receipt;
15. receipt artifact upload.

Receipt generation/validation occurs only after all fallible product qualification gates have succeeded.

## 12. Shared-admission assertions

The receipt should contain uniquely named assertions covering at least:

```text
subject.exact-head
subject.exact-parent
subject.exact-scope
preparation.exact-promoted-bytes
upstream.pef-source-anchors
qualification.q001-validator-exact
runtime.toolchain-rust-1.96
runtime.toolchain-cargo-1.96
dependencies.product-frozen-lock
tests.admission-matrix
clippy.strict-all-targets
checkout.immutable-postflight
```

If any required assertion fails, the receipt cannot claim `pass`.

## 13. Q001 receipt proposition

The ProductFrozen PASS proposition should remain narrow, approximately:

> The exact REGEN-019D `mycelix-regenerative-admission` ProductHead satisfies the frozen raw-vs-lineaged PEF admission contract and independent admission matrix under the recorded Rust 1.96.0 ProductFrozen Cargo graph.

It must not claim downstream biomass, specimen, contamination, trial, agronomic, climate, governance, or physical-action correctness.

## 14. Receipt dependency identity

The Q001 `dependencies` object should use:

```text
state = ProductFrozen
```

and include at least the exact promoted Cargo.lock SHA-256 as an algorithm-qualified dependency identity.

Additional exact dependency/source identities may be included when they add a distinct proposition, but redundant identities should not become noise.

## 15. Receipt fixtures/evidence

The receipt should preserve the successful preparation capsule as evidence, including its archive SHA-256 and run/artifact references.

Where useful, the checked-in admission matrix may also be represented as a frozen fixture with both SHA-256 and Git blob identity.

The receipt must distinguish:

```text
preparation artifact identity
!= ProductHead identity
!= ProductFrozen lock identity
!= Q001 validator identity
```

## 16. API behavioral target

REGEN-019D must preserve the REGEN-019 theorem already exercised by the REGEN-010B/010C behavioral oracle:

```text
Raw(Reported | Observed)
-> owning PEF validation
-> exact expectation matching
-> admit

Raw(Derived | Inferred | Forecast | Scenario)
-> reject: lineage required

Lineaged(Derived | Inferred | Forecast | Scenario)
-> owning PEF product validation
-> exact expectation matching
-> admit

Lineaged(Reported | Observed)
-> reject through owning PEF anti-laundering validation
```

Qualification should also retain explicit tests for ID/phenomenon/class substitution, nested-invalid evidence, unknown time, absent scalar, spatial-support preservation, and valid-lineage-vs-complete-reproducibility distinction.

## 17. No consumer migration inside REGEN-019D

REGEN-019D qualifies the reusable waist itself.

It should not simultaneously mutate soil, biomass, specimen, contamination, trial, or other consumers to use the new crate.

Consumer migration is a separate ProductHead/theorem.

This preserves:

```text
shared kernel qualified
!= every consumer correctly integrated
```

## 18. Biomass gate

Executable REGEN-011 biomass may consume the shared admission crate only after REGEN-019D earns its own exact-head PASS.

The intended dependency direction is:

```text
mycelix-core-types
        ^
mycelix-regenerative-admission
        ^
mycelix-regenerative-biomass
```

Biomass must not import soil merely to obtain generic PEF provenance validation.

## 19. Failure semantics

A REGEN-019D failure is evidence about the exact 019D ProductHead only.

It does not retroactively erase:

- REGEN-009 source-convergence PASS;
- REGEN-010B ProductFrozen soil PASS;
- REGEN-010C adversarial-completeness PASS;
- a valid REGEN-019C preparation result.

Likewise, those predecessor PASSes do not manufacture a 019D PASS.

## 20. Deliberate non-claims

Even a successful REGEN-019D campaign establishes no:

- hermetic or bit-for-bit reproducible build environment unless separately proven;
- dependency safety or vulnerability freedom;
- truth of any environmental observation;
- model/scientific validity;
- evidence currentness;
- sampling representativeness;
- spatial containment;
- specimen authenticity;
- contamination safety;
- biomass sustainability or allocability;
- agronomic suitability;
- treatment efficacy;
- climate/carbon claim;
- land/title/access right;
- governance legitimacy;
- recommendation correctness;
- process-execution authority;
- physical actuation authority.

Its intended proposition is deliberately narrow: exact, reusable, ProductFrozen PEF admission semantics suitable for later domain-specific composition.
