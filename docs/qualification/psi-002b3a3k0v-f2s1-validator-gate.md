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
