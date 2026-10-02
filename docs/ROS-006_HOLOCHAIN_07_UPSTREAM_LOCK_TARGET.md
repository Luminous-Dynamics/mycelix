# ROS-006 Holochain 0.7 upstream lock target

Audit date: 2026-10-02  
Scope: upstream holochain/holonix main-0.7 lock graph, compared with Mycelix branch feat/ros-006-qualification-boundary-v2.  
Purpose: record the concrete upstream 0.7 Nix lock target before changing Mycelix's committed lockfile.

## Why this artifact exists

The Mycelix workspace currently resolves Holochain 0.6-era tooling. The official Holochain 0.6 → 0.7 guide requires the Holonix input and its lockfile to move together, followed by dependency and source migration. This artifact captures the upstream lock inputs that a future Mycelix lock regeneration must reproduce or intentionally supersede.

This is **target evidence, not claim of migration completion**.

## Upstream main-0.7 lock target

The following values were read directly from the holochain/holonix repository's main-0.7 branch on 2026-10-02:

| Input | Locked revision | Upstream original ref |
| --- | --- | --- |
| Holochain | 84cdce7d4df17b95189324d5cecc3f1bfd5db30f | holochain-0.7.0 |
| Kitsune2 | 585151a32f4937c210873ced5aac94503b908d9d | v0.5.0 |
| Lair | 6807971847e4ce0a609aac6d28c405b1c037206d | v0.7.1 |
| hc-scaffold | 83c8b9751afdc5a43d3c4fb43f8a795fdf0db5a0 | v0.700.0-rc.0 |
| nixpkgs | 2f5a153c270b70cb0f8c11f46d96d6d3bc39f4e3 | nixos-26.05 |
| rust-overlay | b99d48435bc3e34309d2c7ae6f7d45e77a156c38 | upstream default |
| crane | 756d6d07c3818ea95d1e2cdac63fa7d02fe3e61b | upstream default |
| flake-parts | 17c9d6cdfc60c64f4ee8d306f9bc0b4ccb51481e | upstream default |
| nixpkgs-lib | db3f255737b94216eb71cce308e2912cf6bc2d7c | upstream default |

The upstream Holonix lock also no longer contains the 0.6-era playground and hc-launch lock nodes present in the Mycelix workspace lock.

## Relevant Nix consequence

The Mycelix workspace currently has:

- Holonix locked to d21b35431e425e615bc05da790987380a84b8280;
- Holochain locked to a holochain-0.6.0 generation;
- Kitsune2 v0.3.2;
- Lair v0.6.3;
- hc-scaffold 0.600.0-dev.0.

Therefore a correct migration is not a single Holonix revision substitution. The entire lock graph must be regenerated in a real Nix environment so dependency relationships, follows, hashes, and any workspace-specific inputs are resolved coherently.

## Required execution

When the 0.7 normalization is executed:

1. change the active Holonix input from the 0.6-era pin to the intended 0.7 ref;
2. regenerate mycelix-workspace/flake.lock with Nix rather than hand-copying this table;
3. update standalone Civic and Commons flakes consistently;
4. inspect the resulting lock graph for Holochain 0.7.0 / Kitsune2 0.5.0 / Lair 0.7.1;
5. regenerate Cargo locks inside the resulting 0.7 environment;
6. only then begin compiler-guided source migration.

## Integrity rule

Do **not** treat the upstream lock values above as permission to hand-edit a generated lockfile. They are an auditable expected target and a reproducibility anchor.

A Mycelix lockfile should be considered migrated only after:

- Nix accepts the generated lock graph;
- the selected dev shell exposes the intended 0.7 toolchain;
- Cargo resolves the intended HDK/HDI generation;
- representative zome compilation succeeds;
- runtime/conductor evidence is captured separately.

## Evidence boundary

This document does not claim:

- that Mycelix currently builds under Holochain 0.7;
- that the Mycelix lockfile has been regenerated;
- that the upstream lock target is immutable;
- that hc-scaffold v0.700.0-rc.0 is the final application-facing CLI target.

The official compatibility table currently recommends Holochain 0.7.0, hc 0.7.0, hc-scaffold 0.700.0, hc-spin 0.700.0, HDK 0.7.0, HDI 0.8.0, and Lair 0.7.1. The upstream Holonix main-0.7 branch is therefore used here as the concrete lock-generation source, while the compatibility table remains the normative version target.

## Sources

- Holochain 0.6 → 0.7 upgrade guide: https://developer.holochain.org/resources/upgrade/upgrade-holochain-0.7/
- Holochain 0.7 compatibility table: https://developer.holochain.org/resources/compatibility/holochain-0.7/
- Upstream Holonix main-0.7 lockfile: https://github.com/holochain/holonix/blob/main-0.7/flake.lock