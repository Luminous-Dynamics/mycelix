# GOVSYS-003D-V0 — Rust Root-A + EvidenceLease Convergence

Status: **prerequisite convergence only**

This tranche establishes one exact executable tree containing:

- qualified Rust Root-A semantics from #862; and
- qualified monotone `EvidenceLease` algebra from #181.

It deliberately does **not** implement source authentication, closed-world coverage, constitutional currentness, policy currentness, administrative authority, or external-effect authority.

## Exact parents

1. Rust Root-A #862: `9ce4f905d74ab53e414a0f9cce63e8122c84c8cd`
2. EvidenceLease #181: `58fa357e53d7e529362c5f766965498ee557d6ce`

Selective merge: `59174c73a506706bd9e68afa508b57454f147252`

Pinned merge tree: `e346f07b5700fff45b653555f28fb46aaaf0cab9`

The merge tree equals the exact #862 tree plus only:

- `crates/mycelix-authority-evidence-lease/Cargo.toml`
- `crates/mycelix-authority-evidence-lease/INVARIANTS.md`
- `crates/mycelix-authority-evidence-lease/src/lib.rs`

## Theorem boundary

```text
qualified Rust Root-A
+ qualified EvidenceLease
+ exact selective convergence
!= authenticated root source
!= closed-world source coverage
!= current constitutional root
```

This convergence exists so a later GOVSYS-003D-V implementation can derive source-verification semantics from an exact private-origin Root-A token and carry only a conservative already-qualified lease algebra.

## Required separation

`root_source_anchor_digest` and `rotation_authority_anchor_digest` remain distinct trust roles.

The later source verifier must use only the endpoint Root-A source-verification profile/anchor and must not accept the rotation-authority anchor as source authority.

The network remains infrastructure for institutions. It is not the sovereign.
