# Licensing

**The root `LICENSE` file (Apache-2.0) does NOT cover the code in this repository's cluster subdirectories.** This was ambiguous before this document existed — the old README said "see individual cluster directories for license information" while a permissive Apache-2.0 license sat at the root, which a reasonable reader could take as the default. It is not.

## What's actually under what license

Checked directly against each subdirectory's own `LICENSE` file / `Cargo.toml` `license` field as of commit `0c60c66e` (2026-07-18):

| Path | License |
|---|---|
| Repository root (`LICENSE` file itself, this document, top-level tooling) | Apache-2.0 |
| `mycelix-commons/`, `mycelix-civic/`, `mycelix-hearth/`, `mycelix-finance/`, `mycelix-governance/`, `mycelix-identity/`, `mycelix-personal/`, `mycelix-attribution/`, `mycelix-praxis/`, `mycelix-music/`, `mycelix-marketplace/`, `mycelix-climate/`, `mycelix-energy/`, `mycelix-knowledge/`, `mycelix-desci/`, `mycelix-core/`, `mycelix-workspace/`, `mycelix-manufacturing/`, `mycelix-position/`, `mycelix-craft/` | **AGPL-3.0-or-later** (each has its own `LICENSE` file or a `Cargo.toml` `license = "AGPL-3.0-or-later"` field confirming it) |
| `crates/` (shared bridge crates: `mycelix-bridge-common`, `mycelix-bridge-entry-types`, `mycelix-core-types`, `mycelix-zkp-core`, `mycelix-leptos-core`, `mycelix-leptos-client`, `mycelix-zome-helpers`, `feldman-dkg`, `luminous-sim-core`) | **AGPL-3.0-or-later** |
| `mycelix-health/` | Separate repository, included as a **git submodule** — its own license applies; check that repo directly, it is not covered by this repo's root license either way |
| `mycelix-lawful-identity/` | **Currently unspecified** — its root `Cargo.toml` has no `license` field and it has no `LICENSE` file of its own. This is a real gap, not an oversight this document is papering over: **do not assume any license applies** to this directory's code until it's fixed upstream. Flagged for a follow-up fix, not resolved by this document. |
| Any other cluster directory not explicitly listed above | Not yet audited for this document — **do not assume Apache-2.0**; check that directory's own `LICENSE`/`Cargo.toml` before relying on it, and treat AGPL-3.0-or-later as the more likely default given every checked cluster uses it |

**In short: if you're linking against or redistributing code from a cluster directory (which is almost certainly what you actually want to use), assume AGPL-3.0-or-later, not the root Apache-2.0 license, unless that specific directory's own license file says otherwise.**

## Why the split

Same rationale as this org's other dual-licensed projects (see [`xenia-peer`](https://github.com/Luminous-Dynamics/xenia-peer)'s README for the fuller version of this argument): AGPL on the application-layer clusters means a commercial user or SaaS operator who modifies and distributes/serves this code must open-source their changes or negotiate a commercial license, while shared library crates and the root tooling stay permissive so third-party tools can link against them freely.

## Commercial licensing

Dual commercial licensing for AGPL-covered clusters is available on request — the repository author is the sole copyright holder for original work in this repository and can grant exceptions case-by-case. Contact tristan.stoltz@evolvingresonantcocreationism.com.

## If you find another inconsistency

This document was written by auditing the repository's actual license files, not by assuming the intended policy — if you find a cluster whose `LICENSE` file or `Cargo.toml` disagrees with what's written here, or a directory with no license information at all, please open an issue. License ambiguity is a real legal risk for anyone trying to use this code, not just a documentation nicety.
