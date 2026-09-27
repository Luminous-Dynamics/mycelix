# MYC-IP-001A — License declaration census and contradiction matrix

Status: evidence census only. This document records repository declarations observed at the frozen parent. It does not determine legal precedence, relicense code, establish ownership, or grant commercial exceptions.

Parent issue: #884 (`MYC-IP-001`)

Tracking issue: #3155 (`MYC-IP-001A`)

## Purpose

Mycelix currently exposes multiple license declarations through root files, child `LICENSE` files, Cargo manifests, cluster READMEs, and repository-level explanatory prose.

Some of those declarations disagree.

Before changing policy or correcting documentation, freeze what the repository actually says today.

The governing evidence rule is:

```text
observed declaration
!= intended policy
!= legal interpretation
!= verified relicensing authority
```

This census therefore answers only:

1. what declaration was observed;
2. where it was observed;
3. whether observed declarations agree with each other;
4. what remains unresolved.

## Frozen subject

Repository: `Luminous-Dynamics/mycelix`

Parent commit:

```text
4a190a9c6ad8d9f1e291f10916472a183f01eddc
```

Observation date: 2026-09-27.

Changing repository state requires a new census generation rather than silently updating this one.

## Declaration-state vocabulary

These labels describe evidence, not law.

| State | Meaning |
| --- | --- |
| `CONSISTENT_OBSERVED` | Checked declarations materially agree at the audited surfaces. |
| `CONTRADICTORY_DECLARATIONS` | Two or more checked repository declarations materially disagree. |
| `UNSPECIFIED_MANIFEST` | The checked Cargo/workspace manifest does not state a license even though another surface may. |
| `SEPARATE_REPOSITORY` | The path is a git submodule / separately governed repository; this census does not infer its license. |
| `PARTIALLY_AUDITED` | Some declarations were checked, but the component has not received a complete file-level audit. |
| `NOT_YET_AUDITED` | No conclusion should be drawn from this census. |

## Initial contradiction matrix

| Component | Repository boundary | Nearest checked `LICENSE` | Checked Cargo/workspace declaration | Checked README declaration | Root `LICENSING.md` declaration | Root README declaration | Evidence state | Immediate unresolved question |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| Repository root | Same repository | Apache License 2.0 | No root `Cargo.toml` observed at the audited root | Root README says root Apache license does not cover most cluster/shared-crate code | Root/top-level tooling classified Apache-2.0 | Same caveat; most checked clusters/crates described as independently AGPL | `CONSISTENT_OBSERVED` for the narrow root surface | Precisely define which top-level files/tooling are intended to be root-Apache rather than inherited/independent artifacts. |
| `mycelix-identity/` | Same repository | Child `LICENSE` contains GNU AGPL v3 text | `[workspace.package] license = "AGPL-3.0-or-later"` | `Apache 2.0` | Classified `AGPL-3.0-or-later` | Classified among independently AGPL clusters | `CONTRADICTORY_DECLARATIONS` | README says Apache while manifest + child license + repository schedule say AGPL. Human policy decision must precede correction. |
| `mycelix-finance/` | Same repository | Child `LICENSE` contains GNU AGPL v3 text | `[workspace.package] license = "AGPL-3.0-or-later"` | `Apache-2.0` | Classified `AGPL-3.0-or-later` | Classified among independently AGPL clusters | `CONTRADICTORY_DECLARATIONS` | README says Apache while manifest + child license + repository schedule say AGPL. Human policy decision must precede correction. |
| `crates/mycelix-bridge-common/` | Same repository | Not independently checked in this tranche | `license = "AGPL-3.0-or-later"` | Not independently checked in this tranche | Shared `crates/` table says `AGPL-3.0-or-later`; explanatory prose later says shared library crates "stay permissive" | Shared crates described as independently AGPL | `CONTRADICTORY_DECLARATIONS` | Resolve whether shared crates are intended AGPL or permissive; the schedule/manifests and rationale prose currently disagree. |
| `crates/mycelix-core-types/` | Same repository | Not independently checked in this tranche | `license = "AGPL-3.0-or-later"` | Not independently checked in this tranche | Shared `crates/` table says `AGPL-3.0-or-later`; explanatory prose later says shared library crates "stay permissive" | Shared crates described as independently AGPL | `CONTRADICTORY_DECLARATIONS` | Same shared-crate policy contradiction; do not infer intended permissive relicensing from prose alone. |
| `mycelix-lawful-identity/` | Same repository | No child `LICENSE` conclusion frozen by this tranche | No workspace/package `license` field observed in root `Cargo.toml` | Says `AGPL-3.0-or-later (matching the rest of the Mycelix workspace)` | Says license is currently unspecified and explicitly warns not to assume one | Says lawful-identity currently has no license specified | `CONTRADICTORY_DECLARATIONS` + `UNSPECIFIED_MANIFEST` | Newer README claims AGPL while manifest has no declaration and root schedules still say unspecified. Do not add license metadata until intended policy/authority is explicitly chosen. |
| `mycelix-health/` | Git submodule / separate repository | Not audited here | Not audited here | Not audited here | Explicitly described as separate repository whose own license applies | Explicitly described as separate repository | `SEPARATE_REPOSITORY` | Audit `Luminous-Dynamics/mycelix-health` independently before including it in an external dependency/license schedule. |

## Evidence excerpts and file identities

The census deliberately records the file-level facts that create the contradiction rather than paraphrasing a presumed policy.

### Repository root

Checked surfaces:

- `LICENSE` — Apache License Version 2.0 text.
- `README.md` — states the root Apache license does not cover most of the repository and says checked clusters/shared crates are independently AGPL-3.0-or-later.
- `LICENSING.md` — makes the same root-versus-cluster distinction.

The root observation is therefore narrow:

```text
root LICENSE says Apache-2.0
```

not:

```text
all files in repository are Apache-2.0
```

### `mycelix-identity`

Checked surfaces:

```text
mycelix-identity/LICENSE
    GNU AFFERO GENERAL PUBLIC LICENSE
    Version 3

mycelix-identity/Cargo.toml
    [workspace.package]
    license = "AGPL-3.0-or-later"

mycelix-identity/README.md
    ## License
    Apache 2.0
```

This is a direct repository contradiction.

The census does not convert the agreement between the child license and manifest into a legal-precedence rule. It only records that those two declarations agree while the README disagrees.

### `mycelix-finance`

Checked surfaces:

```text
mycelix-finance/LICENSE
    GNU AFFERO GENERAL PUBLIC LICENSE
    Version 3

mycelix-finance/Cargo.toml
    [workspace.package]
    license = "AGPL-3.0-or-later"

mycelix-finance/README.md
    ## License
    Apache-2.0
```

This is another direct repository contradiction.

### Shared crates

Two checked examples:

```text
crates/mycelix-bridge-common/Cargo.toml
    license = "AGPL-3.0-or-later"

crates/mycelix-core-types/Cargo.toml
    license = "AGPL-3.0-or-later"
```

Repository `LICENSING.md` also classifies shared `crates/` as AGPL-3.0-or-later.

However, its explanatory section says:

```text
shared library crates and root tooling stay permissive
```

Those propositions cannot be treated as simultaneously resolved policy without further evidence.

This census therefore classifies the shared-crate surface as contradictory rather than choosing one interpretation.

### `mycelix-lawful-identity`

Checked surfaces:

- root `Cargo.toml` has no observed `[workspace.package] license` declaration;
- cluster README currently says `AGPL-3.0-or-later (matching the rest of the Mycelix workspace)`;
- root `LICENSING.md` says lawful-identity is currently unspecified;
- root README likewise says lawful-identity currently has no license specified.

This is important because documentation freshness differs across surfaces.

```text
newer README assertion
!= completed license-policy reconciliation
```

No `LICENSE` file or manifest declaration should be manufactured merely to make the documents agree. The intended policy and authority to apply it must be established first.

### `mycelix-health`

`.gitmodules` records:

```text
[submodule "mycelix-health"]
    path = mycelix-health
    url = https://github.com/Luminous-Dynamics/mycelix-health.git
```

Accordingly:

```text
parent repository license census
!= child repository license census
```

This document makes no license claim about the health repository.

## Commercial-licensing authority: current statement vs verified state

Repository `LICENSING.md` currently says dual commercial licensing for AGPL-covered clusters is available and states that the repository author is the sole copyright holder for original work in the repository.

Root README also says commercial licensing is available on request.

Those are **observed repository statements**.

They are not, by themselves, a complete contributor/IP provenance audit.

For diligence purposes this census freezes the stronger status as:

```text
commercial exception / relicensing authority for whole offered codebase
= UNVERIFIED_FOR_WHOLE_CODEBASE
```

until #884 establishes contributor provenance sufficient to support a narrower statement.

Required distinctions include:

```text
original author-owned code
!= external contributor code
!= copied/derived third-party code
!= dependencies
!= generated code
!= trademarks / names / logos
!= patent rights
```

A future schedule may prove broad or narrow commercial-licensing authority. This census does not prejudge that result.

## No legal-precedence inference

For avoidance of accidental overclaiming, this program must not create a mechanical legal rule such as:

```text
Cargo.toml > README
child LICENSE > root LICENSE
newest statement > oldest statement
```

as though repository file ordering determined the legal result.

Engineering tooling can detect disagreement. Human/legal review must determine intended and effective policy where that matters.

## Resolution program

The next tranches should remain ordered.

### MYC-IP-001B — intended policy decision record

For each component family, record the intended outbound license policy and the authority/evidence supporting any change.

At minimum distinguish:

- application/domain clusters;
- shared protocol/types crates;
- developer tooling;
- UI/client libraries;
- submodules/separate repositories;
- generated artifacts/examples/documentation.

No bulk "make everything AGPL" or "make shared crates Apache" patch should precede this decision.

### MYC-IP-001C — declaration reconciliation

Only after 001B, make the repository declarations consistent:

```text
LICENSE text
manifest license metadata
README statement
root README schedule
LICENSING.md schedule/rationale
```

Every change should reference the chosen policy rather than infer it.

### MYC-IP-001D — contributor/IP provenance schedule

Create evidence for authorship and inbound contribution boundaries.

The schedule should support, where possible:

- original-author-owned paths/commits;
- external contributors and contribution terms;
- imported/derived code;
- generated code provenance;
- dependency boundaries;
- trademarks and branding assets;
- unresolved provenance.

Unknown is an acceptable state. Silent assumption is not.

### MYC-IP-001E — commercial-authority statement

Write the external-facing commercial licensing statement from verified provenance, not aspirational wording.

Possible outcomes may differ by component/path.

### MYC-IP-001F — drift lint

Once policy is explicit, add regression tooling that detects contradictions such as:

```text
manifest SPDX != declared schedule
README license statement != declared schedule
missing license metadata where policy requires it
separate-repo component accidentally assigned parent policy
```

The lint should check consistency with an explicit policy schedule. It must not invent policy.

## External dependency gate

For the Integral reference-stack work:

```text
protocol evaluation with disclosed uncertainty
```

may proceed independently of a production dependency commitment.

But:

```text
ask Integral to depend on Mycelix code
+ materially contradictory license declarations
```

is not a diligence-ready state.

Before dependency-oriented outreach, the exact components in the proposed profile should have:

1. a consistent license declaration;
2. a bounded commercial-authority statement if commercial exceptions are offered;
3. separate-repository boundaries disclosed;
4. unresolved third-party/contributor limits disclosed.

This requirement applies to the **offered component set**, not necessarily every experimental directory before any conversation can occur.

## Audit expansion backlog

This initial census intentionally does not pretend to cover the whole repository.

The next evidence expansion should inspect, at minimum:

- every cluster listed in root `LICENSING.md`;
- every package under `crates/`;
- Mycelix SDK/client crates;
- Leptos client/core packages;
- top-level scripts/tooling intended for external use;
- each git submodule/separate repository;
- copied/generated/vendor directories if present;
- documentation/assets with distinct copyright/license terms if present.

Each row should be based on direct file evidence at a frozen commit.

## Nonclaims

This document is not legal advice. It does not establish ownership, license enforceability, copyright provenance, freedom to operate, patent rights, trademark rights, or authority to relicense any contribution.

It does not select Apache-2.0, AGPL-3.0-or-later, dual licensing, or any other model as the preferred policy.

Its purpose is narrower: ensure later technical, commercial, and legal decisions begin from a reproducible record of what the repository actually says.