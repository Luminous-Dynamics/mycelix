# PSI-002B3A3K0V-F2S1 — Validator Gate Preflight

Status: **IMPLEMENTATION GATE / NOT EXECUTED / NOT A PASS**

## Purpose

This artifact records the bounded implementation gate for the structural JSON Schema validator required by qualification issue #3439.

It does **not** establish semantic currentness, issuer-key admission, token validity, replay authority, PSI qualification, or contact-discovery security.

## Candidate validator

- Crate: `jsonschema`
- Candidate release: `0.58.2`
- Upstream release commit: `3d48b9026c6518e7f3a2ecc6c9c94a9f77f6083c`
- Rust floor declared upstream: `1.85.0`
- Edition: `2021`
- Cargo resolver: `3`
- Required dialect: JSON Schema Draft 2020-12
- Required meta-schema URI: `https://json-schema.org/draft/2020-12/schema`

The candidate is evidence for implementation selection only. It is not yet the qualification lock.

## Required hardening before execution

1. Repository-owned complete dependency/source lock.
2. Immutable Rust toolchain identity.
3. Offline invocation with reference resolution disabled or otherwise provably bounded.
4. Explicit raw JSON duplicate-member disposition before semantic instance validation.
5. Exact canonical validation-report encoding and digest contract.
6. Independent structural vectors covering all frozen #3439 negative cases.

## Non-delegation boundary

The structural validator must not import, invoke, subprocess, or otherwise delegate acceptance to:

- the currentness oracle;
- the candidate currentness evaluator;
- issuer-directory parser/admission logic;
- freshness logic;
- history/rollback evaluator;
- token verification;
- replay/nonce authority.

## Claim ceiling

A successful run can establish only that the frozen schema/structural vectors were processed deterministically by the frozen validator environment.

It cannot establish:

- current issuer key status;
- authenticated issuer-directory freshness;
- RFC9578 token validity;
- replay authorization;
- PSI/contact-discovery security.

## External standards basis

JSON Schema currently publishes Draft 2020-12 as its current specification version. The schema contract should therefore identify that dialect explicitly rather than relying on an implicit/default draft.

## Execution disposition

**NOT EXECUTED.**

No PASS or production-security claim may be inferred from this preflight artifact.


## Dialect/runtime lock preflight

The implementation boundary is now pinned as follows, pending generation and review of the complete dependency lock:

- JSON Schema dialect: Draft 2020-12.
- Required schema dialect identifier: `https://json-schema.org/draft/2020-12/schema`.
- Validator crate: `jsonschema 0.58.2`.
- Validator release source identity: `3d48b9026c6518e7f3a2ecc6c9c94a9f77f6083c` (release commit).
- Cargo feature policy: `default-features = false`; no HTTP or filesystem reference-resolution features.
- Validator construction must use the explicit Draft 2020-12 API and a repository-bounded schema/resource set. External reference retrieval is outside the qualification boundary.
- Qualification runtime: Rust `1.96.0`, matching the repository-owned `mycelix-workspace/rust-toolchain.toml`; the candidate validator's upstream floor is lower and therefore does not weaken this runtime pin.
- Cargo resolver: 3.
- Edition: 2024 for the repository-owned qualification package; the external validator itself declares edition 2021.
- Remaining pre-implementation lock item: a committed standalone `Cargo.lock` generated from the exact qualification package, plus its SHA-256 and dependency/source inventory. This cannot be represented by a version-only dependency declaration.

The validator's upstream documentation explicitly supports Draft 2020-12 and explicitly documents that external reference resolution can be disabled with `default-features = false`. citeturn1search0turn1search4

This is a **preflight lock proposal, NOT EXECUTED / NOT A PASS**. No semantic validator result or currentness/admission claim follows from this artifact.

## Standalone package boundary advanced

A dedicated package boundary has now been added at:

`qualification/psi-002b3a3k0v-f2s1-validator/`

The package declares:

- Rust `1.96.0`;
- edition `2024`;
- Cargo resolver `3`;
- exact direct dependency `jsonschema = 0.58.2`;
- `default-features = false`;
- `publish = false`;
- a local Cargo `[workspace]` boundary so it does not inherit the existing repository workspace.

This advances #3450's package-isolation gate but deliberately does **not** claim that dependency resolution has been executed. The standalone `Cargo.lock` is still absent and therefore the package must not yet be built with a qualification claim.

Cargo documents that `--locked` fails when the lockfile is missing or would change, while `--offline` prevents network access; these controls are reserved for the subsequent lock-generation/verification gate. citeturn0search0turn0search1

**Current disposition: PACKAGE BOUNDARY CREATED / LOCK NOT GENERATED / NOT EXECUTED / NOT A PASS.**
