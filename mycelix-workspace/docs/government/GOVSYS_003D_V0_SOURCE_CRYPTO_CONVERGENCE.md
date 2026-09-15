# GOVSYS-003D-V0 — Root-A / EvidenceLease / strict Ed25519 verifier convergence

Status: **prerequisite convergence only**

## Purpose

Create one exact qualified ancestry for the first concrete GOVSYS-003D-V source/coverage verifier without copying a cryptographic trust boundary.

This tranche converges:

1. qualified Rust Root-A + EvidenceLease convergence #1063; and
2. qualified production Rust predecessor-owned strict Ed25519 verifier #899.

It imports only the #899 production verifier crate into the exact #1063 tree.

## Exact ancestry

First parent — #1063:
`fb1673017acf4b40d2343df14eb5c7f062f7fe74`

Second parent — #899:
`9ac0ac1408b39c8ad9e61cd83a41f0889565fcf0`

Selective merge:
`b6a2359279ee1d288d545db5146cf9572163843d`

Pinned merge tree:
`f569dc13b44835562a2f10e468df86af2b8fcad8`

Relative to #1063 the merge imports exactly:

- `crates/mycelix-constitutional-transition-verifier/Cargo.toml`
- `crates/mycelix-constitutional-transition-verifier/src/lib.rs`
- `crates/mycelix-constitutional-transition-verifier/tests/conformance.rs`

No #899 workflow is copied. Its hosted qualification remains attached through real second-parent Git ancestry.

## Trust-role boundary

The imported verifier is qualified precedent for canonical Ed25519 SPKI parsing and strict A/R/S acceptance. It does **not** turn a rotation signature into root-source currentness.

`rotation_authority_anchor_digest != root_source_anchor_digest`

and

`VerifiedRootTransition != VerifiedRootSourceCoverage`.

A later #1133 source verifier must authenticate its own exact Root-A-selected source-verifier material, complete snapshot transcript, bounded EvidenceLease and schedule-coverage result. It may reuse the strict cryptographic acceptance implementation only under the source-verification trust role.

## Qualification theorem

A hosted PASS proves only:

- exact two-parent ancestry and exact merge tree;
- exact three-file import from qualified #899;
- exact preservation of qualified #1063 Rust Root-A and EvidenceLease product blobs;
- executable compatibility of Rust Root-A, EvidenceLease and the strict transition verifier in one tree;
- the imported verifier remains no-currentness/no-effect-authority; and
- immutable qualification checkout.

## Permanent nonclaims

```text
qualified Root-A
+ qualified EvidenceLease
+ qualified strict Ed25519 verifier implementation
+ exact selective convergence
!= authenticated root source
!= complete source snapshot
!= closed-world source coverage
!= current constitutional root
!= administrative authority
!= external-effect authority
```

This convergence is the intended implementation ancestry for #1133. The network remains infrastructure for institutions. It is not the sovereign.
