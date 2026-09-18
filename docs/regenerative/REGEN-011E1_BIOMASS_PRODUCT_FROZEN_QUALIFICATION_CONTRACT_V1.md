# REGEN-011E1 — Biomass ProductFrozen Qualification Contract v1

Status: preregistration hardening only

Parent: REGEN-011E biomass ProductFrozen promotion preregistration

Program: Luminous-Dynamics/mycelix#940

## Purpose

REGEN-011E freezes the promotion theorem. REGEN-011E1 freezes the concrete qualification obligations for the ProductFrozen biomass candidate, with particular emphasis on the reservation type-state boundary introduced by REGEN-011D1.

The qualifier must demonstrate the API boundary from an **external consumer's perspective**, not merely inspect internal source text.

## Governing theorem

```text
exact D1 preparation capsule
+ exact-byte reconstruction over qualified REGEN-019D
+ checked-in ProductFrozen lock
+ external-consumer API boundary proof
+ exact-head tests / strict Clippy / immutability
+ exact Q001 validator
+ validated machine-readable receipt
= bounded biomass planning-software theorem
```

not ecological truth, rights validity, contamination safety, physical consumption, process execution, biochar qualification, agronomic efficacy, carbon removal, governance authority, or physical actuation.

## 1. Promotion input

Only a successful exact-head REGEN-011D1 preparation capsule is eligible.

The promoter must bind at minimum:

```text
preparation ProductHead
preparation run
preparation job
preparation artifact ID
preparation archive SHA-256
PREPARED_FILES.sha256 contents
Cargo.toml SHA-256
Cargo.lock SHA-256
src/lib.rs SHA-256
tests/biomass_core.rs SHA-256
```

An archive digest alone is insufficient.

## 2. Qualified parent

The ProductFrozen biomass subject must be reconstructed directly over the qualified shared-admission ProductHead:

```text
872151cae7bee995b6bfff7867fc40178ad4759e
```

The D1 preparation branch ancestry must not enter the ProductFrozen lineage.

## 3. Intended ProductFrozen shape

Conceptually the one-commit ProductFrozen subject should contain only the exact promoted biomass crate bytes plus its qualification assets:

```text
.github/workflows/regen-biomass-product-frozen.yml
qualification/regen-011e/verify-regen-qualification-receipt.py
crates/mycelix-regenerative-biomass/Cargo.toml
crates/mycelix-regenerative-biomass/Cargo.lock
crates/mycelix-regenerative-biomass/src/lib.rs
crates/mycelix-regenerative-biomass/tests/biomass_core.rs
```

The exact Q001 validator copy must preserve the already-qualified validator blob identity.

No preparation workflow, materializer workflow, promoter workflow, temporary repair script, or staging ancestry belongs in the ProductFrozen subject.

## 4. Exact byte theorem

The ProductFrozen qualifier must verify both SHA-256 content digests and Git blob identities for every promoted biomass crate path.

The promoted crate bytes must exactly match the successful D1 preparation capsule.

No formatting, source repair, lock regeneration, dependency update, or test rewrite is allowed during promotion.

If any promoted product byte requires correction, the correct response is a new D1 preparation lineage.

## 5. ProductFrozen Cargo graph

The successful D1-prepared `Cargo.lock` becomes checked-in ProductFrozen product state.

Every dependency-resolving material command in qualification must use:

```text
+1.96.0
--locked
```

No qualification step may regenerate or update the lock.

## 6. Exact upstream shared-admission anchor

The qualifier must prove that the ProductFrozen parent carries the exact qualified REGEN-019D shared-admission bytes expected by the D1 biomass crate.

At minimum the exact shared-admission source blob already qualified by REGEN-019D must be checked.

Relevant qualified source blob:

```text
crates/mycelix-regenerative-admission/src/lib.rs
294b777c95be79a13ca7d0e1e3463d32ff25c8f0
```

Additional core/PEF anchors should be bound where the D1 preparation contract relies on them.

## 7. External-consumer type-state theorem

Internal unit/integration tests are necessary but not sufficient for the most important D1 API claim.

The ProductFrozen campaign should compile temporary external consumer crates against the exact local ProductFrozen biomass crate.

The qualifier must prove both a positive and negative surface.

### Positive external surface

An external crate must be able to:

- import `ReservationRequest`;
- construct a valid request through the public request API;
- call `evaluate_reservation_requests`;
- receive `ReservationAcceptance`;
- inspect returned `AcceptedReservation` values only through the public read-only API.

This compilation must succeed.

### Negative external surface

Separate intentionally-invalid external crates/snippets must attempt at least:

1. direct struct-literal construction of `AcceptedReservation`;
2. calling any internal request-to-accepted minting function if one exists privately;
3. direct struct-literal construction of `ReservationAcceptance` if its fields are intentionally private and direct fabrication would bypass evaluator semantics.

Each negative consumer must fail compilation for the expected privacy/API reason.

The test harness must fail if an invalid consumer unexpectedly compiles.

## 8. Why compiler-negative tests matter

Source grep can show that a constructor name is absent, but it does not prove the public Rust API is unforgeable from another crate.

Compiler-negative tests establish the property at the language visibility boundary actually consumed downstream.

```text
internal implementation convention
!= externally enforced API boundary
```

## 9. Negative compile test hygiene

The negative API tests should live only in temporary qualification workspace state unless a separately reviewed compile-fail harness is promoted as product evidence.

They must not mutate the ProductFrozen crate.

The qualifier should capture the compiler exit status and enough bounded diagnostic evidence to prove the failure corresponds to privacy/non-public construction rather than unrelated dependency failure.

## 10. Positive compile test hygiene

The positive external consumer should depend on the ProductFrozen biomass crate by exact local path in the checked-out ProductHead and build under Rust/Cargo 1.96 with an independently generated temporary consumer lock if needed.

That temporary consumer lock is qualification harness state, not product dependency state.

The ProductFrozen biomass crate itself remains `--locked` against its frozen lock.

## 11. Batch atomicity theorem

Product tests must preserve:

```text
complete request batch valid
-> AcceptedReservation values may be minted

any request in batch invalid
-> Err(...)
-> no partial ReservationAcceptance escapes
```

The qualification receipt should name this assertion explicitly rather than subsume it under a generic `tests.pass` line.

## 12. Duplicate-reference theorem

The ProductFrozen campaign must explicitly retain the regression that duplicate reservation references are rejected before accepted state is returned.

## 13. Exact scope theorem

Accepted state must remain scoped to the exact:

- lot identity;
- feedstock assessment identity;
- state snapshot;
- process profile;
- mass basis;
- validated capacity envelope.

The ProductFrozen receipt should not merely say “reservation tests passed.” It should bind these semantic assertion classes individually or through a named frozen fixture set.

## 14. Reservation != consumption

The ProductFrozen campaign must retain and name:

```text
AcceptedReservation
!= physical consumption
```

No successful D1 reservation path may mutate the physical biomass lot into a consumed state.

Consumption accounting remains REGEN-011C.

## 15. Consumption != execution

Likewise:

```text
consumption evidence
!= process execution authority
```

The ProductFrozen biomass crate contains no reactor/process controller or actuator authority.

## 16. Ecology/evidence invariants preserved

D1 is a type-state successor, not permission to weaken prior biomass protections.

Qualification must preserve the already-prepared regressions for:

- exact shared PEF admission;
- ecological allocation separate from material partition;
- ecological evidence snapshot binding;
- material partition reference binding;
- rights/custody/removal/processing references;
- tri-state feedstock assessment;
- basis mismatch rejection;
- exact checked mass arithmetic;
- positive non-zero reservation requirements.

## 17. Enum-indirection regression

The corrected pre-D1 preparation changed `FeedstockAssessment::Eligible` to use boxed indirection to satisfy strict Clippy without suppression.

The exact promoted source digest automatically binds this representation, but strict ProductFrozen Clippy must independently confirm no regression or suppression is needed.

## 18. Rust/toolchain proof

The qualifier must explicitly install/select/prove:

```text
rustc 1.96.0
cargo 1.96.0
rustfmt from the 1.96.0 toolchain
```

Every material Rust/Cargo invocation must select `+1.96.0` explicitly or run under a proven local override established in the same step.

Explicit selection is preferred for reviewability.

## 19. Formatting

ProductFrozen formatting is checked without mutation:

```text
cargo +1.96.0 fmt --manifest-path ... -- --check
```

The qualification workflow must not run mutating `cargo fmt` on product files.

## 20. Tests

The exact ProductFrozen crate runs:

```text
cargo +1.96.0 test --manifest-path ... --locked
```

Any future feature campaign must be explicitly named and frozen rather than silently enabling all features.

## 21. Strict Clippy

The exact ProductFrozen crate runs:

```text
cargo +1.96.0 clippy \
  --manifest-path ... \
  --all-targets \
  --locked \
  -- -D warnings
```

No lint suppression is introduced during qualification merely to earn PASS.

## 22. Postflight immutability

After all product tests and Clippy:

- every promoted file SHA-256 must still equal the expected digest;
- `git diff --check` must pass;
- checkout status must be clean.

Any mutated source or lock invalidates the campaign.

## 23. Exact Q001 validator

The ProductFrozen subject should include the exact already-qualified Q001 validator bytes rather than depending on a mutable copy elsewhere in the repository.

The qualifier must verify the validator Git blob and execute its self-test before using it to validate the biomass qualification receipt.

## 24. Machine-readable receipt

The receipt should use:

```text
schema = mycelix.regen.qualification-receipt-v1
subject.class = ProductHead
dependencies.state = ProductFrozen
result = pass
```

only after all material qualification gates have succeeded.

## 25. Receipt dependency identities

The Q001 receipt should bind at least:

- ProductFrozen biomass `Cargo.lock` digest;
- exact qualified REGEN-019D ProductHead or exact shared-admission source identity;
- D1 preparation capsule digest;
- biomass test fixture/source identity;
- Q001 validator identity where representable as fixture/evidence.

## 26. Receipt assertions

Recommended named assertions include at least:

```text
subject.exact-head
subject.exact-parent
subject.exact-scope
preparation.exact-promoted-bytes
upstream.shared-admission-product-frozen
qualification.q001-validator-exact
runtime.rust-1.96
runtime.cargo-1.96
dependencies.product-frozen-lock
api.request-public
api.accepted-direct-construction-rejected
api.accepted-minting-private
reservation.batch-atomic
reservation.duplicate-reference-rejected
reservation.scope-bound
reservation.basis-bound
reservation.capacity-bound
ecology.snapshot-bound
ecology.partition-bound
tests.biomass-core
clippy.strict-all-targets
checkout.immutable-postflight
```

Exact naming may evolve before implementation, but generic PASS assertions should not erase the D1 theorem.

## 27. Receipt proposition

The final proposition should remain narrow, along the lines of:

> The exact REGEN-011 biomass ProductHead satisfies the frozen dependency-light biomass planning and evaluator-minted reservation type-state contract under the recorded Rust 1.96.0 ProductFrozen Cargo graph, including external enforcement that accepted reservation state cannot be directly constructed by ordinary downstream consumers.

## 28. Receipt non-claims

At minimum preserve explicit non-claims for:

- environmental/ecological truth;
- current biomass availability;
- ownership/right validity;
- custody authenticity;
- contamination safety;
- pyrolysis suitability or safety;
- physical consumption;
- process execution;
- biochar output qualification;
- agronomic efficacy;
- carbon removal/credit;
- economic value;
- governance authority;
- physical actuation;
- hermetic system closure.

## 29. Preparation evidence != ProductFrozen PASS

The successful D1 preparation/materialization campaign is necessary evidence but does not transfer its PASS to the reconstructed ProductFrozen ProductHead.

The ProductFrozen head must independently execute every material gate.

## 30. Qualification receipt upload

Only the already-validated Q001 JSON is uploaded as the qualification receipt artifact.

Receipt upload success remains separately observable from the scientific/software qualification result.

## 31. No overwrite promoter rule

The promoter must refuse to overwrite an existing ProductFrozen target branch.

A second attempt with different bytes uses a new branch/revision rather than force-updating historical evidence.

## 32. Promoter privilege boundary

The promoter requires write permission solely to construct the exact ProductFrozen branch.

The ProductFrozen qualifier itself is read-only.

This keeps branch materialization authority separate from qualification logic.

## 33. Promoter cannot qualify

A promoter workflow concluding success means only that the intended exact bytes were reconstructed and committed under the frozen topology.

```text
promotion success
!= ProductFrozen qualification PASS
```

The new ProductHead's own read-only PR workflow must run independently.

## 34. First implementation order

Once D1 earns exact-head preparation PASS:

```text
1. record preparation run/job/artifact/archive identities
2. record exact internal file digests
3. stage exact Q001 validator + biomass ProductFrozen workflow
4. construct one-shot promoter with all expected digests frozen
5. execute promoter once
6. open ProductFrozen PR over qualified REGEN-019D
7. execute read-only exact-head qualification
8. validate/upload Q001 receipt
9. only then record ProductFrozen PASS
```

## 35. Deliberate non-claims

REGEN-011E1 is a qualification-contract hardening artifact. It does not itself qualify biomass software, create a ProductFrozen branch, establish environmental truth, confer ownership or processing authority, authorize consumption, operate equipment, certify biochar, recommend an agricultural intervention, establish a carbon claim, or permit physical actuation.
