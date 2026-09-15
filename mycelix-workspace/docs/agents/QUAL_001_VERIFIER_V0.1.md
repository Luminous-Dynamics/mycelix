# QUAL-001A Independent Qualification Verifier Bootstrap v0.1

Status: **bootstrap candidate; independent topology inactive until explicit default-branch adoption and repository-protection qualification**

Issues: #866, #938

## Purpose

QUAL-001 separates four identities that earlier candidate-owned theorem lanes could only review together:

```text
TheoremSubject
!= QualificationVerifier
!= QualificationExecutionEnvironment
!= QualificationReceipt
```

This bootstrap is deliberately orthogonal to the AGENT product lineage. Its parent is the exact `main` commit `31ede2365b81365bb119cd9351b2739119974130` rather than AGENT-002.

AGENT-002 exact head `0db977083e89645bb9602f70f56fd2a75ac19877` and receipt SHA-256
`8754d86a90f6d247de9e314578e10d75b180c9d1073f872a45243b76c46d34d0`
are referenced as the motivating qualified theorem, not as an ancestry dependency.

## Finite bootstrap rule

QUAL-001A cannot honestly prove that its own verifier is already independent of itself.

Its candidate-owned bootstrap lane may prove:

- exact source/tree and parent;
- exact verifier implementation bytes;
- exact gate-manifest bytes/profile;
- exact authoritative-workflow bytes;
- exact verifier-bundle manifest bytes;
- bundle-to-component byte consistency;
- hostile-input self-tests;
- static security properties of the future authoritative lane;
- immutable checkout; and
- a postflight bootstrap receipt.

But:

```text
QUAL-001A bootstrap PASS
!= default-branch adoption
!= protected verifier trust root
!= active independent subject/verifier topology
```

The independent topology becomes active only after explicit adoption of this verifier root onto the repository default branch **and** QUAL-001P (#938) proves an enforced repository update policy for that trust root.

## Default-branch authoritative topology

After adoption, GitHub `pull_request_target` selects the workflow from the default branch. The v0.1 authoritative lane therefore uses:

```text
default-branch verifier checkout
        !=
exact hostile subject checkout
        !=
RUNNER_TEMP generated state
```

The workflow grants only `contents: read`, references no secrets, uses pinned `actions/checkout`, disables persisted credentials, and never executes code from the subject checkout.

The verifier implementation, gate manifest, and verifier-bundle manifest are read only from the verifier checkout.

## Stable verifier semantic identity

The whole default-branch commit remains receipt provenance, but it is **not** the semantic identity of QUAL-001 v0.1. Unrelated `main` changes may rotate the default-branch head/tree without changing verifier semantics.

The semantic trust root is the exact verifier-bundle manifest:

`qualification/qual-001/verifier-bundle-v0.1.json`

Verifier-bundle manifest SHA-256:

`b0281549127d0aa892a347ebffd32c5d9e59f8b6c26b3a478db852dc4366a964`

It pins exactly three semantic files:

```text
.github/workflows/qual-001-authoritative.yml
  SHA-256 884f8ea58b5a21d21e9626d463ceabdd2ef01a2df83629a5922ec7945f619ac3

qualification/qual-001/gate-manifest-v0.1.json
  SHA-256 60478ceec4b5e5c542a8a66d6e34f13c94eb2048a4f77773100347ba0e859b61

qualification/qual-001/verifier_v0_1.py
  SHA-256 ef1cf59d7d96e5689fcfe1f4f873a7cfeefa3c365cd813bd019d7c26952f44d2
```

At runtime the verifier re-hashes all three and fails closed if any component disagrees with the bundle manifest. A semantic verifier change therefore requires a new reviewed bundle identity/profile rather than silently inheriting an old profile name.

## v0.1 static subject profile

Profile:

`mycelix.qual.static-subject-independence.v0.1`

Gate manifest SHA-256:

`60478ceec4b5e5c542a8a66d6e34f13c94eb2048a4f77773100347ba0e859b61`

The first profile is intentionally static. It proves the ownership/topology boundary before QUAL-001 expands into build/test execution.

It checks:

- exact default-branch verifier head/tree as execution provenance;
- exact verifier-bundle identity and component bytes;
- exact subject head/tree/base;
- bounded changed-path count;
- bounded changed-file bytes;
- path traversal rejection;
- symlink rejection;
- forbidden verifier-root shadowing; and
- canonical receipt construction.

Candidate-local workflows and candidate-local verifier scripts outside the verifier-owned namespace are non-authoritative data. The authoritative verifier neither imports nor executes them.

## Verifier-owned paths

A hostile subject may not redefine:

- `.github/workflows/qual-001-authoritative.yml`;
- `qualification/qual-001/**`.

Later verifier revisions may widen or version this set only through a new independently reviewed verifier identity/profile.

## Repository-protection prerequisite

Default-branch placement alone is insufficient. Fresh repository evidence before this candidate showed:

```text
main protected = false
required status checks = none
repository rulesets = []
```

QUAL-001P (#938) therefore blocks activation until repository settings externally prove PR-only updates, force-push/delete resistance, appropriate admin/bypass handling, and required independent-verifier checks where supported.

This repository-governance dependency is intentionally visible rather than hidden inside the software theorem.

## Receipt semantics

A successful independent-verifier receipt binds:

- exact subject head/tree/base;
- exact default-branch verifier head/tree as provenance;
- verifier-bundle schema and SHA-256 as semantic verifier identity;
- exact authoritative-workflow SHA-256;
- exact verifier-source SHA-256;
- exact gate-manifest profile and SHA-256;
- changed-path digest/count;
- changed-byte total;
- `subject_code_executed = false`;
- `candidate_local_verifier_authoritative = false`; and
- explicit network-sandbox non-claim.

The receipt is evidence of qualification under one verifier profile. It is not runtime authority.

## Security non-claims

```text
QUAL-001A bootstrap PASS != active independent verifier topology
QUAL-001A bootstrap PASS != protected default branch
QUAL-001P PASS != protection against GitHub/account compromise
QUAL-001 PASS != product formal verification
QUAL-001 PASS != verifier correctness
QUAL-001 PASS != runtime attestation
QUAL-001 PASS != current authority
QUAL-001 receipt != runtime authority
static subject profile != build/test semantic qualification
```

## Next gate

After explicit default-branch adoption **and** QUAL-001P protection PASS, QUAL-001B should open an intentionally hostile subject PR that adds a candidate-local always-PASS checker and weakened local workflow. The default-branch `pull_request_target` verifier must remain authoritative, ignore the weakened checker, preserve the verifier-bundle identity, and emit a receipt binding both exact subject and exact verifier semantics.
