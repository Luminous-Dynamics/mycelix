# AGENT Generic Authority Convergence v0.1

Status: **pre-AGENT-002 selective convergence theorem**

This tranche joins the qualified AGENT lineage to the repaired reusable Mycelix authority waist without importing governance-specific execution semantics and without creating a parallel AI-agent authority stack.

## Exact parents

The final convergence subject is required to be one genuine two-parent commit:

- first parent — AGENT-001: `7b729880b651ab1e476baad3903d4620041223ed`
- second parent — qualified authority repair #809: `5e945700c4c0dba015f571acd2e754475a5143fc`

The second parent is provenance, not whole-tree adoption. The convergence tree is built from the AGENT-001 tree and selectively materializes only the five admitted authority roots.

## Qualified authority-parent evidence

#809 passed exact-head semantic qualification in GitHub Actions run `34833153899`, attempt 1, under Rust 1.98.1.

The checked-in evidence inputs are exact outputs of that passing run:

- `evidence/agent-generic-authority-convergence-v0.1/authority-source-receipt.json`
- `evidence/agent-generic-authority-convergence-v0.1/authority-source-Cargo.lock`

The source receipt is schema `mycelix.authority-delegation-semantic-repair.receipt.v0.2` and has SHA-256:

`9c444ebc06595fcd19b2fcc28fd93af859cac4376f1c0d615faf4c3afc39fa05`

The carried Cargo.lock has SHA-256:

`38889f5045c6e5f6ea3b9427e056429b316021b244cdec6fda4fbd1afb49888d`

The convergence qualification must parse the source receipt strictly, reject duplicate or unknown fields, require its exact canonical compact JSON bytes, bind it to the successful #809 run, and prove its root-tree and lock commitments against the actual Git parents and materialized tree.

## Admitted generic authority waist

Exactly these roots are admitted:

| Root | Exact tree SHA |
|---|---|
| `crates/mycelix-institutional-core` | `6602af340acaa660ffd7e4d46a2f84e67009f396` |
| `crates/mycelix-authority-identity` | `ab98e976ee7acc67e7b5d2af7fc0d16a476f2710` |
| `crates/mycelix-authority-freshness` | `76ec24222cfbd7a7d528996eb27df2f33dcc3c5a` |
| `crates/mycelix-authority-delegation-policy` | `9c41ee69990e0fbb78a58dd82ed693df0889c862` |
| `crates/mycelix-authority-delegation` | `665db95344b9787a377a4e06c74e434914535389` |

For every root, qualification requires:

`manifest tree == #809 receipt tree == convergence HEAD tree == HEAD^2 tree`.

No other #809 product or workflow tree is imported.

## Canonical identity reuse

AGENT-002 MUST reuse the institutional `PrincipalId` already present in the admitted authority waist. A parallel `AgentPrincipalId` is forbidden.

This tranche also freezes reuse of the existing generic authority vocabulary, including `AuthorityGrant`, `Intent`, and `ActionRequest`. Agent-specific identity relationships may later compose these primitives, but must not silently redefine them.

## Dependency continuity

The convergence run does not independently resolve a new dependency graph.

It copies the exact source-qualified Cargo.lock into an isolated temporary five-member workspace, proves the exact package census, fetches that locked graph once, then runs all semantic commands with Cargo resolution offline:

- `cargo test --workspace --all-targets --locked`
- `cargo clippy --workspace --all-targets --locked -- -D warnings`

Generated Cargo state remains outside the evidence checkout.

`CARGO_NET_OFFLINE=true` is a Cargo-resolution claim only. This tranche does not claim an OS-level network sandbox.

## Historical source status remains separate

The historical authority source is #77 at:

`8e4a04f9e03caf36bc58cd30bc5ce92e40c8fe0f`

Its hosted run `33865664837` stopped at `cargo fmt --check` before tests and Clippy. #809 repaired and semantically qualified the selected five-package waist, but that does not retroactively make #77 a full-source PASS.

Therefore:

`AGENT authority convergence PASS != upstream PR #77 full qualification`.

## Excluded semantics

The following remain domain-specific and are not made universal agent semantics by this convergence:

- governance proposal authority;
- governance threshold qualification;
- governance executor designation; and
- governance execution lifecycle.

## Qualification meaning

A PASS establishes only that the AGENT lineage contains the exact source-qualified generic authority waist, with exact Git ancestry, root-tree identity, source receipt/lock continuity, structural primitive reuse, and independent five-package semantic requalification.

It does not establish controller identity, runtime/model attestation, mission interpretation, live current authority, exact action qualification, credential brokerage, durable effect reservation, external provider correctness, or independent end-to-end verification.

The following claims remain mechanically blocked:

`AGENT authority convergence PASS != AGENT-002 identity PASS`.

`AGENT authority convergence PASS != full agent security`.

## Postflight evidence

Only after every fallible convergence gate passes may the workflow emit:

`AGENT_AUTHORITY_CONVERGENCE_RECEIPT_JSON`

with schema `mycelix.agent.generic-authority-convergence.receipt.v0.1`.

That receipt is evidence for later AGENT tranches. It is not itself an authority grant, capability, credential, or execution permission.
